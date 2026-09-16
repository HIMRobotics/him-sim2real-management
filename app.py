#!/usr/bin/env python3
"""Local-only operator panel. Python standard library; never starts a robot policy."""
import argparse
import contextlib
import datetime as dt
import fcntl
import hmac
from http import cookies
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import re
import secrets
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
from urllib.parse import parse_qs, urlparse
import uuid
import webbrowser
import zipfile

import g1_session as kit

ROOT = Path(__file__).resolve().parent


def read_json(path, default=None):
    try:
        return json.loads(path.read_text(encoding='utf-8'))
    except (OSError, ValueError):
        return {} if default is None else default


def short_text(value, limit=4000):
    if not isinstance(value, str) or len(value) > limit or '\x00' in value:
        raise ValueError('Invalid or excessively long text')
    return value.strip()


class App:
    def __init__(self, data_root=ROOT, demo=False):
        self.root = Path(data_root).resolve()
        self.runs = self.root / 'runs'
        self.exports = self.root / 'exports'
        self.private = self.root / '.app'
        for p in (self.runs, self.exports, self.private):
            p.mkdir(parents=True, exist_ok=True)
        self.lock_file = (self.private / 'server.lock').open('a')
        try:
            fcntl.flock(self.lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            self.lock_file.close()
            raise RuntimeError('The app is already running for this folder. Use its existing browser tab/terminal.')
        self.demo = demo
        self.token = secrets.token_urlsafe(32)
        self.lock = threading.RLock()
        self.process = None
        self.log_handle = None
        self.current = None
        self.stopping = False
        self.closing = False
        self.connection = {'state': 'unchecked', 'message': 'Check the connection before your first run.'}
        self.connection_config = None
        self.checked_at = 0
        self.job = None
        self.last_error = None
        self.settings = read_json(self.private / 'settings.json')

    def path(self, run_id):
        if not isinstance(run_id, str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,150}', run_id):
            raise ValueError('Invalid run selection')
        path = self.runs / run_id
        if path.is_symlink() or path.resolve().parent != self.runs or not (path / 'manifest.json').is_file():
            raise ValueError('Run not found')
        return path

    def config(self, body):
        interface = short_text(body.get('interface', 'auto'), 100) or 'auto'
        available = ['auto'] + [name for _, name in socket.if_nameindex()]
        if interface not in available:
            raise ValueError('Select one of this computer\'s network interfaces')
        domain = int(body.get('domain', 0))
        if not 0 <= domain <= 232:
            raise ValueError('DDS domain must be between 0 and 232')
        python = short_text(body.get('python', ''), 1000) or sys.executable
        if not Path(python).is_absolute() or not Path(python).is_file() or not os.access(python, os.X_OK):
            raise ValueError('Robot Python must be the full path to an executable Python interpreter')
        return {'interface': interface, 'domain': domain, 'python': python}

    def reap(self):
        if self.process is not None and self.process.poll() is not None:
            self.log_handle.close()
            run = self.runs / self.current
            if (run / 'CLOSED.json').exists():
                try:
                    kit.copy_evidence(run, self.private / (self.current + '.log'), 'recorder-log')
                    kit.report(run)
                except Exception as exc:
                    self.last_error = 'Run saved; recorder log/report needs attention: ' + str(exc)
            else:
                log = self.private / (self.current + '.log')
                with log.open('rb') as f:
                    f.seek(max(0, log.stat().st_size - 4000))
                    detail = f.read().decode('utf-8', errors='replace')
                self.last_error = 'Recorder ended before finalization. Check the selected Python. ' + detail
            self.process = None
            self.log_handle = None
            self.stopping = False

    def busy(self):
        self.reap()
        if self.closing:
            raise ValueError('The app is closing')
        if self.job and self.job['state'] == 'working':
            raise ValueError('Please wait for the current operation to finish')

    def start_job(self, name, function):
        self.busy()
        self.job = {'name': name, 'state': 'working', 'message': name}
        job = self.job
        def worker():
            try:
                result = function()
                with self.lock:
                    job.update(state='done', result=result)
            except Exception as exc:
                with self.lock:
                    job.update(state='error', message=str(exc))
                    self.last_error = str(exc)
        threading.Thread(target=worker, daemon=True).start()

    def check(self, body):
        with self.lock:
            self.busy()
            if self.process:
                raise ValueError('Finish the current recording before checking the connection again')
            config = self.config(body)
            self.connection_config = None
            self.connection = {'state': 'checking', 'message': 'Listening for robot state…'}
            def work():
                try:
                    if self.demo:
                        result = {'state': 'ready', 'message': 'Demo ready — synthetic data only.'}
                    else:
                        cmd = [config['python'], '-u', str(ROOT / 'g1_session.py'), 'doctor',
                               '--interface', config['interface'], '--domain', str(config['domain']), '--timeout', '5']
                        p = subprocess.run(cmd, capture_output=True, text=True, timeout=20)
                        output = (p.stdout + p.stderr)[-12000:]
                        passed = p.returncode == 0 and 'PASS: live G1 state received' in p.stdout
                        result = {'state': 'ready' if passed else 'error',
                                  'message': 'Connected. Live G1 state received.' if passed else
                                             'Could not read G1 state. Check the selected Python and network interface.',
                                  'details': output}
                    with self.lock:
                        self.connection = result
                        self.connection_config = config if result['state'] == 'ready' else None
                        self.checked_at = time.monotonic()
                        self.settings.update(config)
                        kit.atomic_json(self.private / 'settings.json', self.settings)
                    return result
                except Exception as exc:
                    with self.lock:
                        self.connection = {'state': 'error', 'message': str(exc)}
                    raise
            self.start_job('Checking connection', work)

    def start(self, body):
        with self.lock:
            self.busy()
            if self.process:
                raise ValueError('A run is already recording')
            if any(p.is_file() for p in self.runs.glob('*/ACTIVE.json')):
                raise ValueError('Another run is active or unfinalized. Resolve it before starting a new run.')
            config = self.config(body)
            if config != self.connection_config or time.monotonic() - self.checked_at > 300:
                raise ValueError('Check the connection with these settings first (check is valid for five minutes).')
            name = short_text(body.get('name', ''), 100)
            operator = short_text(body.get('operator', ''), 100)
            if not name or not operator:
                raise ValueError('Enter a run name and operator name')
            policy = short_text(body.get('policy', ''), 200) or name
            slug = re.sub(r'[^A-Za-z0-9_-]+', '-', name).strip('-')[:60]
            if not slug:
                raise ValueError('Use a run name containing letters or numbers')
            run_id = dt.datetime.now(dt.timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '_' + slug + '_' + uuid.uuid4().hex[:6]
            cmd = [config['python'], '-u', str(ROOT / 'g1_session.py'), 'record', '--root', str(self.runs),
                   '--run-id', run_id, '--name', name, '--policy', policy, '--operator', operator,
                   '--robot', short_text(body.get('robot', ''), 200) or 'unspecified',
                   '--notes', short_text(body.get('notes', '')), '--interface', config['interface'],
                   '--domain', str(config['domain']), '--reserve-gib', '2']
            if self.demo:
                cmd.append('--demo')
            log = (self.private / (run_id + '.log')).open('wb')
            try:
                process = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            except Exception:
                log.close()
                raise
            self.process = process
            self.log_handle = log
            self.current = run_id
            self.last_error = None
            self.job = None
            self.settings.update(operator=operator, robot=body.get('robot', ''), **config)
            kit.atomic_json(self.private / 'settings.json', self.settings)
            return {'run': run_id}

    def stop(self, body):
        with self.lock:
            self.reap()
            if not self.process:
                return {'message': 'Recording is already stopped'}
            if body.get('run') != self.current:
                raise ValueError('The selected run is not the current recording')
            if not self.stopping:
                self.process.send_signal(signal.SIGINT)
                self.stopping = True
            return {'message': 'Finishing recording; the policy is controlled separately.'}

    def status(self):
        with self.lock:
            self.reap()
            rows = []
            for run in sorted(self.runs.iterdir(), reverse=True):
                if not run.is_dir() or run.is_symlink():
                    continue
                manifest = read_json(run / 'manifest.json')
                if not manifest:
                    continue
                summary = read_json(run / 'summary.json')
                live = read_json(run / 'status.json')
                active = bool(self.process and self.current == run.name)
                age = live.get('last_state_age_s')
                if age is not None:
                    age += max(0, time.time() - dt.datetime.fromisoformat(live['utc']).timestamp())
                phase = ('stopping' if self.stopping else
                         'starting' if not (run / 'READY.json').exists() else
                         'stale' if age is not None and age > 3 else 'recording') if active else (
                         'incomplete' if (run / 'ACTIVE.json').exists() or not (run / 'CLOSED.json').exists() else
                         'error' if manifest.get('error') else 'saved')
                labels = [read_json(p) for p in sorted((run / 'events').glob('*.json'))]
                trials = {}
                for label in labels:
                    if label.get('kind') == 'label':
                        trials[label['trial']] = label
                rows.append({'id': run.name, 'name': manifest['name'], 'policy': manifest.get('policy_label'),
                             'source': manifest['source'], 'phase': phase, 'error': manifest.get('error'),
                             'state_samples': live.get('state_samples', 0) if active else summary.get('state_samples', 0),
                             'command_samples': live.get('command_samples', 0) if active else sum(summary.get('command_samples_by_topic', {}).values()),
                             'state_age_s': round(age, 1) if active and age is not None else None,
                             'warnings': summary.get('warnings', []), 'labels': list(trials.values()),
                             'evidence': [e for e in labels if e.get('kind') == 'evidence'],
                             'has_report': (run / 'report.html').exists() and not active,
                             'path': str(run)})
            return {'demo': self.demo, 'version': kit.VERSION, 'connection': self.connection,
                    'connection_valid': self.connection_config is not None and time.monotonic() - self.checked_at <= 300,
                    'active': self.current if self.process else None, 'starting': bool(self.process),
                    'runs': rows, 'job': self.job, 'settings': self.settings,
                    'interfaces': [name for _, name in socket.if_nameindex()], 'python': sys.executable,
                    'free_gib': round(shutil.disk_usage(self.root).free / 1024**3, 1), 'error': self.last_error}

    def label(self, body):
        with self.lock:
            self.busy()
            run = self.path(body.get('run'))
            trial = short_text(body.get('trial', ''), 100)
            outcome = body.get('outcome')
            operator = short_text(body.get('operator', ''), 100)
            if not trial or not operator or outcome not in kit.OUTCOMES:
                raise ValueError('Trial number, operator and a valid outcome are required')
            kit.mark(SimpleNamespace(run=run, kind='label', trial=trial, outcome=outcome,
                                     operator=operator, notes=short_text(body.get('notes', ''))))
            if (run / 'CLOSED.json').exists():
                kit.report(run)
            self.job = None
            return {'message': 'Outcome saved. Download again if you previously made a return file.'}

    def package(self, body):
        with self.lock:
            self.busy()
            run = self.path(body.get('run'))
            kit.ensure_closed(run)
            def work():
                archive = kit.export(run, self.exports)
                sidecar = archive.with_name(archive.name + '.sha256')
                filename = archive.stem.replace('.tar', '') + '_RETURN.zip'
                target = self.exports / filename
                temp = target.with_suffix('.partial')
                try:
                    with zipfile.ZipFile(temp, 'x', zipfile.ZIP_STORED) as z:
                        z.write(archive, archive.name)
                        z.write(sidecar, sidecar.name)
                        z.writestr('READ_ME.txt', 'G1 SESSION RETURN\n\nSend this whole ZIP to Arena.\nIt contains the run archive and its SHA-256 transfer checksum.\nExtract the ZIP, verify the .tar.gz checksum, then unpack the .tar.gz.\nThe run contains report.html, summary.json, raw telemetry, labels and attachments.\nSynthetic demos remain explicitly marked. Task outcomes are operator labels.\n')
                    with zipfile.ZipFile(temp) as z:
                        if z.testzip() is not None:
                            raise ValueError('Return ZIP failed verification')
                    os.replace(temp, target)
                finally:
                    temp.unlink(missing_ok=True)
                return {'run': run.name, 'download': '/download/' + filename, 'filename': filename}
            self.start_job('Preparing return file', work)

    def recover(self, body):
        with self.lock:
            self.busy()
            if not body.get('confirmed'):
                raise ValueError('Confirm that the recorder and other file writers have stopped')
            run = self.path(body.get('run'))
            if self.process and self.current == run.name:
                raise ValueError('Stop this recording first')
            kit.recover(run)
            return {'message': 'Recovered as incomplete. Original raw data preserved.'}

    def shutdown(self):
        with self.lock:
            self.closing = True
            process = self.process
            if process and process.poll() is None:
                process.send_signal(signal.SIGINT)
        if process:
            with contextlib.suppress(subprocess.TimeoutExpired):
                process.wait(timeout=15)
        with self.lock:
            self.reap()
            if self.process:
                print('Recorder is still finishing. Keep this terminal and use its original run folder; no policy was stopped.', flush=True)
            self.lock_file.close()


class Server(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address, app):
        self.app = app
        super().__init__(address, Handler)
        self.origin = 'http://127.0.0.1:' + str(self.server_address[1])


class Handler(BaseHTTPRequestHandler):
    server_version = 'G1Local/1.1'
    protocol_version = 'HTTP/1.0'

    def log_message(self, fmt, *args):
        pass

    def setup(self):
        super().setup()
        self.connection.settimeout(60)

    def guard(self, auth=True, mutation=False):
        allowed_host = '127.0.0.1:' + str(self.server.server_address[1])
        if self.headers.get('Host') != allowed_host:
            raise PermissionError('Use the exact local address printed by the app')
        if self.headers.get('Origin') not in (None, self.server.origin):
            raise PermissionError('Cross-site requests are not allowed')
        token = self.headers.get('X-G1-Token', '')
        if not token and not mutation:
            jar = cookies.SimpleCookie()
            jar.load(self.headers.get('Cookie', ''))
            if 'g1_session' in jar:
                token = jar['g1_session'].value
        if auth and not hmac.compare_digest(token, self.server.app.token):
            raise PermissionError('Reopen the full app link from its terminal')

    def headers_for(self, status, kind, length, attachment=None):
        self.send_response(status)
        self.send_header('Content-Type', kind)
        self.send_header('Content-Length', str(length))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('Referrer-Policy', 'no-referrer')
        self.send_header('Content-Security-Policy', "default-src 'self'; script-src 'self'; style-src 'self' 'unsafe-inline'; connect-src 'self'; img-src 'self' data:; object-src 'none'; frame-ancestors 'none'; base-uri 'none'; form-action 'self'")
        if attachment:
            self.send_header('Content-Disposition', 'attachment; filename="' + attachment + '"')
        self.end_headers()

    def send_json(self, data, status=200, set_cookie=False):
        raw = json.dumps(data, allow_nan=False).encode()
        if set_cookie:
            # Set manually before shared headers complete the response.
            self.send_response(status)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(raw)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Set-Cookie', 'g1_session=' + self.server.app.token + '; HttpOnly; SameSite=Strict; Path=/')
            self.end_headers()
        else:
            self.headers_for(status, 'application/json', len(raw))
        self.wfile.write(raw)

    def error(self, exc):
        with contextlib.suppress(BrokenPipeError, ConnectionResetError):
            self.send_json({'error': str(exc)}, 403 if isinstance(exc, PermissionError) else 400)

    def file(self, path, mime, download=False):
        if not path.is_file() or path.is_symlink():
            raise ValueError('File is unavailable')
        with path.open('rb') as f:
            self.headers_for(200, mime, os.fstat(f.fileno()).st_size, path.name if download else None)
            shutil.copyfileobj(f, self.wfile, 1024 * 1024)

    def do_GET(self):
        try:
            path = urlparse(self.path).path
            if path in ('/', '/app.js', '/style.css', '/connect'):
                self.guard(auth=False)
                filename, mime = {'/': ('index.html', 'text/html; charset=utf-8'),
                                  '/app.js': ('app.js', 'text/javascript; charset=utf-8'),
                                  '/connect': ('connect.html', 'text/html; charset=utf-8'),
                                  '/style.css': ('style.css', 'text/css; charset=utf-8')}[path]
                self.file(ROOT / 'web' / filename, mime)
                return
            self.guard()
            app = self.server.app
            if path == '/api/state':
                self.send_json(app.status(), set_cookie=True)
            elif path.startswith('/report/'):
                run = app.path(path.removeprefix('/report/'))
                self.file(run / 'report.html', 'text/html; charset=utf-8')
            elif path.startswith('/download/'):
                name = path.removeprefix('/download/')
                if not re.fullmatch(r'[A-Za-z0-9_-]+_RETURN\.zip', name):
                    raise ValueError('Invalid download')
                self.file(app.exports / name, 'application/zip', True)
            elif path.startswith('/api/log/'):
                run_id = path.removeprefix('/api/log/')
                app.path(run_id)
                log = app.private / (run_id + '.log')
                text = ''
                if log.is_file():
                    with log.open('rb') as f:
                        f.seek(max(0, log.stat().st_size - 12000))
                        text = f.read().decode('utf-8', errors='replace')
                self.send_json({'text': text})
            else:
                raise ValueError('Page not found')
        except (BrokenPipeError, ConnectionResetError):
            pass
        except Exception as exc:
            self.error(exc)

    def do_POST(self):
        try:
            self.guard(mutation=True)
            path = urlparse(self.path).path
            if path == '/api/upload':
                self.upload()
                return
            size = int(self.headers.get('Content-Length', 0))
            if not 0 < size <= 32768 or self.headers.get('Content-Type') != 'application/json':
                raise ValueError('Expected a small JSON request')
            body = json.loads(self.rfile.read(size))
            app = self.server.app
            action = {'/api/check': app.check, '/api/start': app.start, '/api/stop': app.stop,
                      '/api/label': app.label, '/api/export': app.package, '/api/recover': app.recover}.get(path)
            if action is None:
                raise ValueError('Action not found')
            result = action(body)
            self.send_json(result or {'ok': True})
        except (BrokenPipeError, ConnectionResetError):
            pass
        except Exception as exc:
            self.error(exc)

    def upload(self):
        app = self.server.app
        query = parse_qs(urlparse(self.path).query)
        run_id = query.get('run', [''])[0]
        name = query.get('name', [''])[0]
        if not name or Path(name).name != name or len(name) > 200 or '\\' in name or any(ord(c) < 32 for c in name):
            raise ValueError('Invalid attachment filename')
        size = int(self.headers.get('Content-Length', -1))
        if not 0 <= size <= 16 * 1024**3:
            raise ValueError('Attachment must be at most 16 GiB')
        with app.lock:
            app.busy()
            run = app.path(run_id)
            kit.ensure_closed(run)
            if shutil.disk_usage(app.root).free < size + 2 * 1024**3:
                raise ValueError('Not enough free space for this attachment plus the 2 GiB reserve')
            job = {'name': 'Adding attachment', 'state': 'working'}
            app.job = job
        target = run / 'evidence' / (uuid.uuid4().hex[:10] + '_' + name)
        temp = app.private / (uuid.uuid4().hex + '.upload')
        try:
            remaining = size
            with temp.open('xb') as f:
                while remaining:
                    chunk = self.rfile.read(min(1024 * 1024, remaining))
                    if not chunk:
                        raise ValueError('Attachment transfer was interrupted; retry the file')
                    f.write(chunk)
                    remaining -= len(chunk)
                    if shutil.disk_usage(app.root).free < 2 * 1024**3:
                        raise ValueError('Disk reserve reached during attachment transfer')
                f.flush()
                os.fsync(f.fileno())
            checksum = kit.digest(temp)
            os.replace(temp, target)
            event = {'schema': 'g1-session-event.v1', 'kind': 'evidence', 'role': 'operator-attachment',
                     'path': str(target.relative_to(run)), 'sha256': checksum, 'utc': kit.utc(),
                     'original_name': name, 'host_monotonic_s': time.monotonic()}
            kit.atomic_json(run / 'events' / (str(time.time_ns()) + '_' + uuid.uuid4().hex + '.json'), event)
            kit.report(run)
            with app.lock:
                job.update(state='done', result={'run': run_id, 'filename': name})
            self.send_json({'ok': True})
        except Exception as exc:
            with app.lock:
                job.update(state='error', message=str(exc))
            raise
        finally:
            temp.unlink(missing_ok=True)


def main():
    p = argparse.ArgumentParser(description='Local G1 recorder control panel. Never starts or stops a robot policy.')
    p.add_argument('--port', type=int, default=8765)
    p.add_argument('--data-dir', type=Path, default=ROOT)
    p.add_argument('--demo', action='store_true')
    p.add_argument('--no-browser', action='store_true')
    args = p.parse_args()
    app = App(args.data_dir, args.demo)
    try:
        try:
            server = Server(('127.0.0.1', args.port), app)
        except OSError as exc:
            if exc.errno not in (48, 98) or args.port == 0:
                raise
            server = Server(('127.0.0.1', 0), app)
        url = server.origin + '/#key=' + app.token
        print('\nG1 Session Kit — ' + ('DEMO ONLY' if args.demo else 'LOCAL RECORDER'), flush=True)
        print('Open this exact link on this computer:\n' + url, flush=True)
        print('\nKeep this terminal open. Browser refresh/closure does not stop recording.\nCtrl+C here finishes the recorder; robot policies remain under operator control.\n', flush=True)
        if not args.no_browser:
            with contextlib.suppress(Exception):
                webbrowser.open(url)
        def stop_server(_signum, _frame):
            threading.Thread(target=server.shutdown, daemon=True).start()
        signal.signal(signal.SIGINT, stop_server)
        signal.signal(signal.SIGTERM, stop_server)
        try:
            server.serve_forever(poll_interval=.2)
        finally:
            server.server_close()
    finally:
        app.shutdown()
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except Exception as exc:
        print('Could not start the local app: ' + str(exc), file=sys.stderr)
        sys.exit(1)
