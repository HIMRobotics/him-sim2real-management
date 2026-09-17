"""Explicit SSH setup/transfer operations. No policy execution or credential storage."""
import contextlib
import hashlib
import json
import os
from pathlib import Path
import queue
import re
import shlex
import shutil
import signal
import socket
import subprocess
import tempfile
import threading
import time
from urllib.request import Request, urlopen
import uuid
import zipfile

ROOT = Path(__file__).resolve().parent

# These fixed scripts, not user-entered shell commands, run under the operator's SSH account.
PROBE = r'''
import json,sys,platform,os
from pathlib import Path
if sys.version_info < (3,9): raise SystemExit('This recorder requires Python 3.9+; select the existing SDK interpreter.')
if platform.system() != 'Linux': raise SystemExit('Select the G1 or connected Linux computer as the SSH destination.')
try:
 from unitree_sdk2py.core.channel import ChannelSubscriber
 from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
 sdk=True; detail='SDK imports passed; live DDS still needs Check connection in the robot recorder.'
except Exception as e:
 sdk=False; detail=str(e)
print(json.dumps(dict(home=str(Path.home()),python=sys.executable,sdk=sdk,detail=detail,hostname=platform.node())))
'''

RECEIVE = r'''
import sys,hashlib,json,os,uuid,zipfile,stat,shutil
from pathlib import Path,PurePosixPath
kind,name,size,expected=sys.argv[1:]; size=int(size)
if kind not in ('kit','policy'): raise SystemExit('Invalid transfer kind')
if Path(name).name != name or any(ord(c)<32 for c in name): raise SystemExit('Invalid name')
base=Path.home()/'.local/share/arena-g1-kit'
base.mkdir(parents=True,exist_ok=True)
if shutil.disk_usage(base).free < size+2*1024**3: raise SystemExit('Remote disk needs the upload size plus 2 GiB free')
dest=base/('releases' if kind=='kit' else 'policies')/uuid.uuid4().hex
dest.mkdir(parents=True,exist_ok=False)
tmp=dest/'incoming.partial'; h=hashlib.sha256(); left=size
try:
 with tmp.open('xb') as f:
  while left:
   chunk=sys.stdin.buffer.read(min(left,1024*1024))
   if not chunk: raise ValueError('Transfer interrupted')
   f.write(chunk); h.update(chunk); left-=len(chunk)
   if shutil.disk_usage(base).free<2*1024**3: raise ValueError('Remote disk reserve reached')
  f.flush(); os.fsync(f.fileno())
 if h.hexdigest()!=expected: raise ValueError('Remote SHA-256 mismatch')
 if kind=='kit':
  with zipfile.ZipFile(tmp) as z:
   for info in z.infolist():
    path=PurePosixPath(info.filename)
    if path.is_absolute() or '..' in path.parts or '\\' in info.filename or stat.S_ISLNK(info.external_attr>>16): raise ValueError('Unsafe kit path')
   if sum(i.file_size for i in z.infolist())>20*1024*1024: raise ValueError('Kit too large')
   z.extractall(dest)
  tmp.unlink(); final=dest
 else:
  final=dest/name; os.replace(tmp,final)
 print(json.dumps(dict(path=str(final),sha256=h.hexdigest(),bytes=size)))
except Exception:
 tmp.unlink(missing_ok=True)
 raise
'''

SUPERVISE = r'''
import sys,subprocess,threading,signal
from pathlib import Path
python,script,browser_port=sys.argv[1:]
p=subprocess.Popen([python,'-u',script,'--no-browser','--port','0','--browser-port',browser_port,'--onboard','--announce-json','--data-dir',str(Path.home()/'.local/share/arena-g1-kit/data')],stdin=subprocess.DEVNULL,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,start_new_session=True)
def close_recorder():
 if p.poll() is None:
  try: p.send_signal(signal.SIGINT)
  except ProcessLookupError: pass
def input_closed():
 sys.stdin.buffer.read(); close_recorder()
threading.Thread(target=input_closed,daemon=True).start()
signal.signal(signal.SIGTERM,lambda *_:close_recorder())
signal.signal(signal.SIGHUP,lambda *_:close_recorder())
signal.signal(signal.SIGINT,lambda *_:close_recorder())
try:
 for line in p.stdout:
  print(line,end='',flush=True)
finally:
 close_recorder()
 try: p.wait(timeout=20)
 except subprocess.TimeoutExpired: pass
'''


def config(body):
    destination = body.get('destination', '').strip()
    if not re.fullmatch(r'(?:[A-Za-z0-9_][A-Za-z0-9_.-]*@)?[A-Za-z0-9][A-Za-z0-9_.-]{0,252}', destination):
        raise ValueError('Enter an existing SSH alias or user@hostname / user@IPv4 address')
    python = body.get('python', 'python3').strip() or 'python3'
    if not (re.fullmatch(r'python3(?:\.[0-9]+)?', python) or python.startswith('/')) or any(ord(c)<32 for c in python):
        raise ValueError('Remote Python must be python3 or an absolute interpreter path')
    port = body.get('port', '')
    port = int(port) if port is not None and str(port).strip() else None
    if port is not None and not 1 <= port <= 65535:
        raise ValueError('SSH port must be between 1 and 65535')
    return dict(destination=destination, python=python, port=port)


def ssh_args(cfg):
    ssh = shutil.which('ssh')
    if not ssh:
        raise ValueError('OpenSSH client is missing on this computer')
    args = [ssh, '-T', '-o', 'BatchMode=yes', '-o', 'StrictHostKeyChecking=yes',
            '-o', 'ConnectTimeout=8', '-o', 'ServerAliveInterval=10', '-o', 'ServerAliveCountMax=3']
    if cfg['port']:
        args += ['-p', str(cfg['port'])]
    return args


def command(cfg, script, *args):
    return ssh_args(cfg) + [cfg['destination'], shlex.join([cfg['python'], '-u', '-c', script, *map(str,args)])]


def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''): h.update(b)
    return h.hexdigest()


def result_json(process):
    if process.returncode:
        detail=(process.stderr or process.stdout or '')[-6000:]
        raise ValueError('SSH operation failed. First verify SSH in your normal terminal (trusted host and an unlocked key/agent). ' + detail)
    try:
        return json.loads(process.stdout.strip().splitlines()[-1])
    except (ValueError,IndexError) as exc:
        raise ValueError('The remote command did not return a valid result. Check the selected Python and SSH login scripts.') from exc


class Remote:
    def __init__(self, private):
        self.private=Path(private)
        self.lock=threading.RLock()
        self.checked=None
        self.checked_at=None
        self.info={}
        self.release=None
        self.last_policy=None
        self.session=None
        self.tunnel=None
        self.url=None
        self.logs=[]
        try:
            saved=json.loads((self.private/'remote.json').read_text())
            self.checked=config(saved['config'])
            self.release=saved.get('release')
            self.last_policy=saved.get('policy')
        except (OSError,ValueError,KeyError,TypeError):
            pass

    def save(self):
        with self.lock:
            data=dict(config=self.checked,release=self.release,policy=self.last_policy)
            tmp=self.private/'remote.json.tmp'
            with tmp.open('w') as f: json.dump(data,f,indent=2)
            os.chmod(tmp,0o600)
            os.replace(tmp,self.private/'remote.json')

    def status(self):
        with self.lock:
            alive=bool(self.session and self.session.poll() is None and self.tunnel and self.tunnel.poll() is None)
            return dict(config=self.checked,info=self.info,release=self.release,policy=self.last_policy,
                        url=self.url if alive else None,logs='\n'.join(self.logs[-20:]),
                        checked=bool(self.checked and self.checked_at is not None and time.monotonic()-self.checked_at<600),
                        disconnected=bool(self.url and not alive))

    def check(self, body):
        cfg=config(body)
        with self.lock:
            if self.session and self.session.poll() is None and cfg!=self.checked:
                raise ValueError('Close the robot recorder connection before selecting a different SSH target')
            self.checked_at=None
        p=subprocess.run(command(cfg,PROBE),capture_output=True,text=True,timeout=25)
        info=result_json(p)
        with self.lock:
            if cfg!=self.checked:
                self.release=None; self.last_policy=None
            self.checked=cfg; self.checked_at=time.monotonic(); self.info=info
            self.save()
        return self.status()

    def require(self):
        with self.lock:
            if not self.checked or self.checked_at is None or time.monotonic()-self.checked_at>600:
                raise ValueError('Check SSH access again first; its result is valid for ten minutes')
            return dict(self.checked)

    def transfer(self,path,kind,name):
        cfg=self.require()
        expected=sha(path)
        with Path(path).open('rb') as f:
            p=subprocess.run(command(cfg,RECEIVE,kind,name,Path(path).stat().st_size,expected),
                             stdin=f,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,timeout=900)
        result=result_json(p)
        if result.get('sha256')!=expected:
            raise ValueError('Remote verification did not match the local file')
        return result

    def install(self):
        self.require()
        with self.lock:
            if self.session and self.session.poll() is None:
                raise ValueError('Finish the run and close the recorder connection before copying a new recorder version')
        # Explicit code/docs allowlist; never copy local runs, exports, SSH credentials or app state.
        paths=[ROOT/name for name in ('app.py','remote.py','g1_session.py','policy_hook.py','OPEN_APP.sh','kit.sh','START_HERE.md','CONNECTION_GUIDE.md','REMOTE_SETUP.md','TROUBLESHOOTING.md')]
        paths+=sorted((ROOT/'web').glob('*'))
        with tempfile.TemporaryDirectory(dir=self.private,prefix='kit-') as temp:
            archive=Path(temp)/'recorder.zip'
            with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED) as z:
                for path in paths:
                    if path.is_file() and not path.is_symlink(): z.write(path,path.relative_to(ROOT).as_posix())
            result=self.transfer(archive,'kit','recorder.zip')
        with self.lock:
            self.release=result
            self.save()
        return result

    def send_policy(self,path,name):
        result=self.transfer(path,'policy',name)
        result['name']=name
        with self.lock:
            self.last_policy=result
            self.save()
        return result

    def open(self):
        cfg=self.require()
        with self.lock:
            if self.session and self.session.poll() is None and self.tunnel and self.tunnel.poll() is None:
                return {'url':self.url}
            if not self.release:
                raise ValueError('Copy the recorder first')
            if not self.info.get('sdk'):
                raise ValueError('The selected remote Python could not import the Unitree SDK. Select its existing SDK interpreter and check SSH again.')
            self.close()
            with socket.socket() as s:
                s.bind(('127.0.0.1',0)); local_port=s.getsockname()[1]
            script=str(Path(self.release['path'])/'app.py')
            self.session=subprocess.Popen(command(cfg,SUPERVISE,cfg['python'],script,local_port),stdin=subprocess.PIPE,
                                          stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,bufsize=1)
            session=self.session
        ready=queue.Queue()
        def read_output():
            for line in session.stdout:
                if line.startswith('ARENA_APP_READY '):
                    try: ready.put(json.loads(line[len('ARENA_APP_READY '):]))
                    except ValueError: pass
                else:
                    safe=re.sub(r'(#key=)[^\s]+',r'\1[local-access-key]',line.strip())
                    with self.lock:
                        self.logs.append(safe); self.logs=self.logs[-40:]
            session.stdout.close()
            ready.put(None)
        threading.Thread(target=read_output,daemon=True).start()
        try:
            message=ready.get(timeout=25)
            if not message:
                raise ValueError('Remote recorder did not start: '+'\n'.join(self.logs[-10:]))
            remote_port=int(message['port']); token=message['token']
            if not 1<=remote_port<=65535 or not re.fullmatch(r'[A-Za-z0-9_-]+',token):
                raise ValueError('Invalid remote startup response')
            tunnel_log=(self.private/'ssh-tunnel.log').open('wb')
            try:
                self.tunnel=subprocess.Popen(ssh_args(cfg)+['-N','-o','ExitOnForwardFailure=yes','-L',
                    '127.0.0.1:{}:127.0.0.1:{}'.format(local_port,remote_port),cfg['destination']],
                    stdin=subprocess.DEVNULL,stdout=tunnel_log,stderr=subprocess.STDOUT)
            finally: tunnel_log.close()
            origin='http://127.0.0.1:'+str(local_port)
            deadline=time.monotonic()+15
            while time.monotonic()<deadline:
                if self.tunnel.poll() is not None:
                    raise ValueError('SSH tunnel did not open. See connection details.')
                try:
                    with urlopen(Request(origin+'/api/state',headers={'X-G1-Token':token}),timeout=1) as r:
                        data=json.load(r)
                        if data.get('onboard'):
                            with self.lock: self.url=origin+'/#key='+token
                            return {'url':self.url}
                except (OSError,ValueError):
                    time.sleep(.15)
            raise ValueError('Could not verify the forwarded recorder page')
        except Exception:
            self.close()
            raise

    def close(self):
        with self.lock:
            # EOF tells the remote supervisor to finish its recorder app. No policy is signaled.
            if self.session:
                with contextlib.suppress(OSError): self.session.stdin.close()
                with contextlib.suppress(subprocess.TimeoutExpired): self.session.wait(timeout=22)
                if self.session.poll() is None:
                    self.session.terminate()
                self.session=None
            if self.tunnel:
                self.tunnel.terminate()
                with contextlib.suppress(subprocess.TimeoutExpired): self.tunnel.wait(timeout=5)
                self.tunnel=None
            self.url=None
