import hashlib
from http.client import HTTPConnection
import io
import json
from pathlib import Path
import signal
import sys
import tarfile
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
import zipfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import app as webapp


class AppTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.app = webapp.App(self.root, demo=True)
        self.server = webapp.Server(('127.0.0.1', 0), self.app)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.app.shutdown()
        self.thread.join(timeout=3)
        self.temp.cleanup()

    def request(self, path, body=None, headers=None, auth=True, raw=False):
        connection = HTTPConnection('127.0.0.1', self.server.server_address[1], timeout=10)
        h = {'X-G1-Token': self.app.token} if auth else {}
        if body is not None:
            if not raw:
                body = json.dumps(body)
                h['Content-Type'] = 'application/json'
        if headers:
            h.update(headers)
        connection.request('GET' if body is None else 'POST', path, body, h)
        response = connection.getresponse()
        data = response.read()
        result = (response.status, dict(response.getheaders()), data)
        connection.close()
        return result

    def json(self, path, body=None, expected=200):
        status, _, raw = self.request(path, body)
        self.assertEqual(status, expected, raw.decode(errors='replace'))
        return json.loads(raw)

    def wait(self, condition):
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            state = self.json('/api/state')
            if condition(state):
                return state
            time.sleep(.05)
        self.fail('Timed out: ' + json.dumps(state))

    def start_run(self):
        self.json('/api/check', {})
        self.wait(lambda s: s['connection']['state'] == 'ready')
        run = self.json('/api/start', {'name':'policy-a-01', 'operator':'Test person'})['run']
        self.wait(lambda s: any(r['id']==run and r['phase']=='recording' for r in s['runs']))
        return run

    def close_run(self, run):
        self.json('/api/stop', {'run':run})
        self.wait(lambda s: not s['active'])

    def test_http_workflow_download_and_integrity(self):
        run = self.start_run()
        self.json('/api/start', {'name':'duplicate','operator':'Test'}, expected=400)
        self.json('/api/label', {'run':run,'trial':'001','outcome':'failure','operator':'Test','notes':'Synthetic missed target'})
        time.sleep(.1)
        self.close_run(run)
        self.json('/api/stop', {'run':run})
        status, _, data = self.request('/api/upload?run='+run+'&name=notes.txt', b'actual attached bytes', raw=True)
        self.assertEqual(status, 200, data)
        self.json('/api/export', {'run':run})
        state = self.wait(lambda s: s['job']['state'] in ('done','error'))
        self.assertEqual(state['job']['state'],'done',state)
        download = state['job']['result']['download']
        status, headers, raw = self.request(download)
        self.assertEqual(status,200)
        self.assertIn('attachment;',headers['Content-Disposition'])
        with zipfile.ZipFile(io.BytesIO(raw)) as z:
            self.assertIsNone(z.testzip())
            name = next(n for n in z.namelist() if n.endswith('.tar.gz'))
            archive = z.read(name)
            self.assertEqual(hashlib.sha256(archive).hexdigest(),z.read(name+'.sha256').decode().split()[0])
            with tarfile.open(fileobj=io.BytesIO(archive)) as tf:
                summary=json.load(tf.extractfile(run+'/summary.json'))
                self.assertGreater(summary['state_samples'],0)
                self.assertEqual(summary['source'],'synthetic_demo')
                self.assertEqual(summary['operator_labeled_outcomes'],{'failure':1})
                checks=json.load(tf.extractfile(run+'/checksums.json'))
                for rel, sha in checks.items():
                    self.assertEqual(hashlib.sha256(tf.extractfile(run+'/'+rel).read()).hexdigest(),sha)
                self.assertTrue(any('notes.txt' in n for n in checks))
        self.json('/api/label', {'run':run,'trial':'001','outcome':'unknown','operator':'Test','notes':'Correction'})
        self.assertIsNone(self.json('/api/state')['job'], 'An old download must not appear current after editing labels')
        # Report/download work with the HttpOnly cookie used by normal browser links.
        status, _, _=self.request('/report/'+run, auth=False,headers={'Cookie':'g1_session='+self.app.token})
        self.assertEqual(status,200)

    def test_start_requires_fresh_matching_check(self):
        body={'name':'policy','operator':'Tester'}
        self.json('/api/start',body,expected=400)
        self.json('/api/check',{})
        self.wait(lambda s:s['connection']['state']=='ready')
        self.json('/api/start',dict(body,domain=2),expected=400)
        self.app.checked_at-=301
        self.json('/api/start',body,expected=400)

    def test_auth_origin_host_and_traversal(self):
        self.assertEqual(self.request('/api/state',auth=False)[0],403)
        self.assertEqual(self.request('/api/state',headers={'Host':'evil.test'})[0],403)
        self.assertEqual(self.request('/api/check',{},headers={'Origin':'https://evil.test'})[0],403)
        self.assertEqual(self.request('/api/check',{},auth=False,headers={'Cookie':'g1_session='+self.app.token})[0],403)
        self.assertEqual(self.request('/report/../../app.py')[0],400)
        self.assertEqual(self.request('/download/../../app.py')[0],400)
        self.assertEqual(self.request('/')[0],200)

    def test_duplicate_app_rejected(self):
        with self.assertRaises(RuntimeError):
            webapp.App(self.root)

    def test_no_policy_launch_in_web_app(self):
        run=self.start_run()
        manifest=json.loads((self.app.runs/run/'manifest.json').read_text())
        self.assertEqual(manifest['policy_command'],[])
        self.close_run(run)

    def test_active_export_upload_recovery_refused(self):
        run=self.start_run()
        self.json('/api/export',{'run':run},expected=400)
        status,_,_=self.request('/api/upload?run='+run+'&name=note.txt',b'hi',raw=True)
        self.assertEqual(status,400)
        self.json('/api/recover',{'run':run,'confirmed':True},expected=400)
        self.json('/api/stop',{'run':'wrong'},expected=400)
        self.close_run(run)

    def test_app_shutdown_finishes_recorder(self):
        run=self.start_run()
        self.app.shutdown()
        self.assertIsNone(self.app.process)
        self.assertTrue((self.app.runs/run/'CLOSED.json').exists())
        self.assertFalse((self.app.runs/run/'ACTIVE.json').exists())

    def test_missing_sdk_check_cannot_enable_start(self):
        self.app.demo=False
        with patch.object(webapp.subprocess,'run',return_value=type('Result',(),{'returncode':1,'stdout':'','stderr':'Unitree SDK unavailable'})()):
            self.json('/api/check',{})
            state=self.wait(lambda s:s['job']['state']=='done')
        self.assertEqual(state['connection']['state'],'error')
        self.assertFalse(state['connection_valid'])
        self.json('/api/start',{'name':'no-data','operator':'Tester'},expected=400)

    def test_browser_disconnect_does_not_stop_run(self):
        run=self.start_run()
        time.sleep(.15)
        # All HTTP connections are closed between requests; the independent recorder stays alive.
        self.assertIsNone(self.app.process.poll())
        state=self.json('/api/state')
        self.assertEqual(state['active'],run)
        self.close_run(run)

    def test_crashed_recorder_can_be_recovered_without_discarding_data(self):
        run=self.start_run()
        time.sleep(.15)
        self.app.process.kill()
        self.app.process.wait(timeout=3)
        state=self.json('/api/state')
        self.assertEqual(state['runs'][0]['phase'],'incomplete')
        self.json('/api/recover',{'run':run,'confirmed':True})
        manifest=json.loads((self.app.runs/run/'manifest.json').read_text())
        self.assertEqual(manifest['status'],'recovered_incomplete')
        self.json('/api/export',{'run':run})
        state=self.wait(lambda s:s['job']['state'] in ('done','error'))
        self.assertEqual(state['job']['state'],'done',state)

    def test_upload_rejects_path_names_and_preserves_disk_reserve(self):
        run=self.start_run();self.close_run(run)
        self.assertEqual(self.request('/api/upload?run='+run+'&name=../bad',b'a',raw=True)[0],400)
        with patch.object(webapp.shutil,'disk_usage',return_value=type('Disk',(),{'free':0})()):
            self.assertEqual(self.request('/api/upload?run='+run+'&name=large',b'a',raw=True)[0],400)


if __name__=='__main__':
    unittest.main()
