import hashlib
from http.client import HTTPConnection
import io
import json
import os
from pathlib import Path
import queue
import shlex
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
import zipfile

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
import remote


class RemoteTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.home=Path(self.temp.name)
        self.private=self.home/'private';self.private.mkdir()
        self.env=dict(os.environ,HOME=str(self.home))

    def tearDown(self):
        self.temp.cleanup()

    def receive(self,data,kind='policy',name='policy.zip',checksum=None):
        return subprocess.run([sys.executable,'-c',remote.RECEIVE,kind,name,str(len(data)),checksum or hashlib.sha256(data).hexdigest()],input=data,capture_output=True,env=self.env)

    def test_strict_ssh_and_shell_quoting(self):
        cfg=remote.config({'destination':'operator@g1.local','python':'/a path/python3'})
        args=remote.command(cfg,remote.PROBE)
        self.assertIn('BatchMode=yes',args)
        self.assertIn('StrictHostKeyChecking=yes',args)
        self.assertEqual(shlex.split(args[-1]),[cfg['python'],'-u','-c',remote.PROBE])
        for target in ('-oProxyCommand=anything','user@host;touch bad','host\ncommand','user@host:22'):
            with self.assertRaises(ValueError):remote.config({'destination':target})
        with self.assertRaises(ValueError):remote.config({'destination':'robot','port':70000})

    def test_policy_transfer_bytes_hash_and_no_execution(self):
        data=b'#!/bin/sh\ntouch SHOULD_NEVER_RUN\n'
        result=self.receive(data,name='policy.sh')
        self.assertEqual(result.returncode,0,result.stderr)
        info=json.loads(result.stdout)
        self.assertEqual(Path(info['path']).read_bytes(),data)
        self.assertEqual(info['sha256'],hashlib.sha256(data).hexdigest())
        self.assertFalse((self.home/'SHOULD_NEVER_RUN').exists())
        other=json.loads(self.receive(data,name='policy.sh').stdout)
        self.assertNotEqual(info['path'],other['path'])

    def test_hash_failure_does_not_leave_ready_policy(self):
        result=self.receive(b'bad data',checksum='0'*64)
        self.assertNotEqual(result.returncode,0)
        self.assertFalse(list(self.home.rglob('policy.zip')))
        self.assertFalse(list(self.home.rglob('*.partial')))

    def test_truncated_input_is_rejected(self):
        result=subprocess.run([sys.executable,'-c',remote.RECEIVE,'policy','policy.bin','100','0'*64],input=b'short',capture_output=True,env=self.env)
        self.assertNotEqual(result.returncode,0)
        self.assertFalse(list(self.home.rglob('policy.bin')))

    def test_kit_extract_and_traversal_rejection(self):
        data=io.BytesIO()
        with zipfile.ZipFile(data,'w') as z:z.writestr('app.py','print("fixture")')
        result=self.receive(data.getvalue(),'kit','kit.zip')
        self.assertEqual(result.returncode,0,result.stderr)
        self.assertEqual((Path(json.loads(result.stdout)['path'])/'app.py').read_text(),'print("fixture")')
        data=io.BytesIO()
        with zipfile.ZipFile(data,'w') as z:z.writestr('../escaped','bad')
        result=self.receive(data.getvalue(),'kit','kit.zip')
        self.assertNotEqual(result.returncode,0)
        self.assertFalse(list(self.home.rglob('escaped')))

    def test_transfer_orchestration_and_profile_persistence(self):
        helper=remote.Remote(self.private)
        actual_run=subprocess.run
        calls=[]
        def transport(args,**kwargs):
            calls.append(args)
            python_args=shlex.split(args[-1])
            script=python_args[3]
            if script==remote.PROBE:
                return subprocess.CompletedProcess(args,0,json.dumps({'home':str(self.home),'python':sys.executable,'sdk':True,'hostname':'fake-g1','detail':'mock SDK probe'}),'')
            return actual_run([sys.executable,*python_args[1:]],env=self.env,**kwargs)
        with patch.object(remote.subprocess,'run',side_effect=transport):
            helper.check({'destination':'operator@fake-g1'})
            installed=helper.install()
            self.assertTrue((Path(installed['path'])/'app.py').is_file())
            self.assertTrue((Path(installed['path'])/'ADVANCED_TERMINAL.md').is_file())
            self.assertTrue((Path(installed['path'])/'start.sh').is_file())
            self.assertFalse((Path(installed['path'])/'runs').exists())
            self.assertFalse((Path(installed['path'])/'.git').exists())
            f=self.home/'model.bin';f.write_bytes(b'model bytes')
            policy=helper.send_policy(f,'model.bin')
            self.assertEqual(Path(policy['path']).read_bytes(),b'model bytes')
        restored=remote.Remote(self.private)
        self.assertEqual(restored.release['path'],installed['path'])
        self.assertEqual(restored.last_policy['path'],policy['path'])
        self.assertFalse(restored.status()['checked'])
        self.assertFalse(restored.status()['url'])
        self.assertTrue(all('StrictHostKeyChecking=yes' in c for c in calls))

    def test_live_sdk_missing_blocks_open(self):
        helper=remote.Remote(self.private)
        helper.checked=remote.config({'destination':'fake'});helper.checked_at=time.monotonic()
        helper.release={'path':'/unused'};helper.info={'sdk':False}
        with patch.object(remote.subprocess,'Popen') as spawn:
            with self.assertRaisesRegex(ValueError,'SDK'):helper.open()
            spawn.assert_not_called()

    def test_supervisor_starts_only_recorder_and_closes_on_eof(self):
        # Executes the real remote supervisor locally, over pipes instead of a real SSH transport.
        proc=subprocess.Popen([sys.executable,'-u','-c',remote.SUPERVISE,sys.executable,str(ROOT/'app.py'),'29991'],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,env=self.env)
        lines=queue.Queue()
        def reader():
            for line in proc.stdout:lines.put(line)
            proc.stdout.close()
            lines.put(None)
        threading.Thread(target=reader,daemon=True).start()
        info=None;output=[]
        try:
            deadline=time.monotonic()+10
            while time.monotonic()<deadline:
                line=lines.get(timeout=max(.1,deadline-time.monotonic()))
                if line is None:break
                output.append(line)
                if line.startswith('ARENA_APP_READY '):
                    info=json.loads(line[len('ARENA_APP_READY '):]);break
            self.assertIsNotNone(info,''.join(output))
            conn=HTTPConnection('127.0.0.1',info['port'],timeout=5)
            conn.request('GET','/api/state',headers={'Host':'127.0.0.1:29991','X-G1-Token':info['token']})
            response=conn.getresponse();state=json.loads(response.read());conn.close()
            self.assertEqual(response.status,200)
            self.assertTrue(state['onboard'])
            self.assertFalse(state['demo'])
            self.assertIsNone(state['active'])
            proc.stdin.close();proc.wait(timeout=8)
            self.assertEqual(proc.returncode,0)
        finally:
            if proc.poll() is None:
                with __import__('contextlib').suppress(OSError):proc.stdin.close()
                try:proc.wait(timeout=8)
                except subprocess.TimeoutExpired:proc.kill();proc.wait()


if __name__=='__main__':unittest.main()
