import hashlib
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tarfile
import tempfile
import threading
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import g1_session as kit
import policy_hook


def state():
    return NS(motor_state=[NS(q=.1, dq=.2, temperature=[31, 42], tau_est=2., mode=1)
                           for _ in range(35)], tick=10, imu_state=NS(rpy=[0., .1, 0.]))


class KitTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)

    def tearDown(self):
        self.temp.cleanup()

    def demo(self):
        result = subprocess.run([sys.executable, str(ROOT / 'g1_session.py'), 'record', '--demo',
            '--name', 'test policy', '--policy', 'Arena test 123', '--seconds', '.25',
            '--reserve-gib', '.1', '--root', str(self.root / 'runs')], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        return next((self.root / 'runs').iterdir())

    def test_hg_arrays_and_missing_fields_are_preserved(self):
        data = kit.state_payload(state())
        self.assertEqual(len(data['motors']), 35)
        self.assertEqual(data['motors'][0]['temperature'], [31, 42])
        self.assertIsNone(data['motors'][0]['lost'])
        self.assertIsNone(data['imu']['quaternion'])
        msg = state()
        msg.motor_state[0].q = float('nan')
        with self.assertRaises(ValueError):
            kit.state_payload(msg)

    def test_queue_pressure_decode_errors_and_readiness(self):
        r = kit.Receiver(hz=500, capacity=1)
        cb = r.callback('rt/lowstate', 'state')
        cb(NS(motor_state=[]))
        self.assertFalse(r.ready.is_set())
        time.sleep(.003)
        cb(state())
        time.sleep(.003)
        cb(state())
        s = r.snapshot()['rt/lowstate']
        self.assertEqual(s['decode_errors'], 1)
        self.assertEqual(s['queue_dropped'], 1)
        self.assertTrue(r.ready.is_set())

    def test_end_to_end_labels_evidence_html_export_integrity(self):
        run = self.demo()
        self.assertFalse((run / 'ACTIVE.json').exists())
        self.assertTrue((run / 'CLOSED.json').exists())
        for trial, outcome in [('1', 'success'), ('2', 'failure'), ('3', 'unknown'), ('4', 'aborted'), ('5', 'invalid')]:
            kit.main(['mark', str(run), '--kind', 'label', '--trial', trial,
                      '--outcome', outcome, '--operator', '<operator>'])
        kit.main(['mark', str(run), '--kind', 'trial-start', '--trial', '6', '--operator', 'tester'])
        evidence = self.root / 'policy.json'
        evidence.write_text('{"version": 1}')
        kit.main(['attach', str(run), str(evidence), '--role', 'policy-config'])
        summary = kit.report(run)
        self.assertGreater(summary['state_samples'], 2)
        self.assertEqual(summary['source'], 'synthetic_demo')
        self.assertEqual(summary['operator_success_denominator'], 3)
        self.assertAlmostEqual(summary['operator_success_fraction_among_resolved_valid_attempts'], 1/3)
        self.assertEqual(summary['operator_labeled_outcomes']['unknown'], 2)
        self.assertEqual(summary['motors_first_29'][0]['temperature']['max'], 33)
        self.assertIn('&lt;operator&gt;', (run / 'report.html').read_text())
        archive = kit.export(run, self.root / 'exports')
        checksum = archive.with_name(archive.name + '.sha256').read_text().split()[0]
        self.assertEqual(checksum, kit.digest(archive))
        with tarfile.open(archive) as tf:
            manifest = json.load(tf.extractfile(run.name + '/checksums.json'))
            for rel, sha in manifest.items():
                self.assertEqual(hashlib.sha256(tf.extractfile(run.name + '/' + rel).read()).hexdigest(), sha)
        self.assertGreater(kit.verify(run), 5)
        (run / 'telemetry' / 'state.jsonl').write_text('tampered')
        with self.assertRaises(ValueError):
            kit.verify(run)

    def test_no_data_timeout_never_launches_policy(self):
        marker = self.root / 'launched'
        args = kit.parser().parse_args(['record', '--name', 'timeout', '--root', str(self.root),
            '--reserve-gib', '.1', '--timeout', '.1', '--', sys.executable, '-c',
            'from pathlib import Path; Path({!r}).touch()'.format(str(marker))])
        with patch.object(kit, 'connect', return_value=([], [])):
            self.assertEqual(kit.collect(args), 1)
        self.assertFalse(marker.exists())
        run = next(self.root.iterdir())
        self.assertEqual(json.loads((run / 'manifest.json').read_text())['status'], 'closed_with_error')
        self.assertEqual(json.loads((run / 'summary.json').read_text())['state_samples'], 0)

    def test_wrapper_environment_exit_code_and_hook(self):
        def connect(receiver, *args):
            stop = threading.Event()
            thread = threading.Thread(target=kit.demo_source, args=(receiver, stop), daemon=True)
            thread.start()
            return [NS(Close=lambda: (stop.set(), thread.join()))], []
        program = 'import os,time,json; from pathlib import Path; p=Path(os.environ["G1_RUN_DIR"]); (p/"policy"/"env.json").write_text(json.dumps(dict(run=os.environ["G1_RUN_DIR"],log=os.environ["G1_POLICY_LOG"]))); time.sleep(.1); raise SystemExit(7)'
        args = kit.parser().parse_args(['record', '--name', 'wrapped', '--root', str(self.root),
            '--reserve-gib', '.1', '--', sys.executable, '-c', program])
        with patch.object(kit, 'connect', side_effect=connect):
            self.assertEqual(kit.collect(args), 1)
        run = next(self.root.iterdir())
        manifest = json.loads((run / 'manifest.json').read_text())
        self.assertEqual(manifest['policy_returncode'], 7)
        self.assertEqual(manifest['stop_reason'], 'policy_exited')
        env = json.loads((run / 'policy' / 'env.json').read_text())
        self.assertEqual(env['run'], str(run.resolve()))
        with patch.dict(os.environ, {'G1_POLICY_LOG': env['log']}):
            self.assertTrue(policy_hook.log_event('inference', duration_ms=2.4))
            self.assertFalse(policy_hook.log_event('bad', value=float('nan')))
        self.assertEqual(kit.report(run)['policy_hook_rows'], 1)

    def test_keyboard_interrupt_closes_and_exports(self):
        proc = subprocess.Popen([sys.executable, str(ROOT / 'g1_session.py'), 'record', '--demo',
            '--name', 'interrupt', '--root', str(self.root), '--reserve-gib', '.1'],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        try:
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline and not list(self.root.glob('*/READY.json')):
                time.sleep(.05)
            self.assertTrue(list(self.root.glob('*/READY.json')))
            proc.send_signal(signal.SIGINT)
            out, err = proc.communicate(timeout=5)
            self.assertEqual(proc.returncode, 0, out + err)
            run = next(self.root.iterdir())
            self.assertTrue((run / 'CLOSED.json').exists())
            kit.export(run, self.root / 'return')
        finally:
            if proc.poll() is None:
                proc.kill()
                proc.wait()

    def test_low_disk_fails_without_connecting(self):
        args = kit.parser().parse_args(['record', '--name', 'disk', '--root', str(self.root)])
        with patch.object(kit.shutil, 'disk_usage', return_value=NS(free=0)), patch.object(kit, 'connect') as connect:
            self.assertEqual(kit.collect(args), 1)
            connect.assert_not_called()

    def test_live_run_export_refused_and_recovery_retains_tail(self):
        run = self.demo()
        kit.atomic_json(run / 'ACTIVE.json', {'host': 'another-host', 'pid': 999999})
        with self.assertRaises(ValueError):
            kit.export(run, self.root / 'exports')
        with (run / 'telemetry' / 'state.jsonl').open('a') as f:
            f.write('{truncated')
        kit.recover(run)
        report = kit.report(run)
        self.assertEqual(report['status'], 'recovered_incomplete')
        self.assertTrue(any('invalid JSON' in w for w in report['warnings']))
        self.assertTrue((run / 'telemetry' / 'state.jsonl').read_text().endswith('{truncated'))

    def test_connect_is_subscriber_only(self):
        names = ['unitree_sdk2py', 'unitree_sdk2py.core', 'unitree_sdk2py.core.channel',
                 'unitree_sdk2py.idl', 'unitree_sdk2py.idl.unitree_hg', 'unitree_sdk2py.idl.unitree_hg.msg',
                 'unitree_sdk2py.idl.unitree_hg.msg.dds_']
        modules = {name: NS() for name in names}
        initialized = []
        topics = []
        class Subscriber:
            def __init__(self, topic, message_type):
                topics.append(topic)
            def Init(self, callback, queue_len):
                self.assert_queue = queue_len
        modules[names[2]] = NS(ChannelFactoryInitialize=lambda *a: initialized.append(a), ChannelSubscriber=Subscriber)
        modules[names[-1]] = NS(LowState_=object, LowCmd_=object)
        with patch.dict(sys.modules, modules):
            subs, warnings = kit.connect(kit.Receiver(), 'eth0', 3)
        self.assertEqual(initialized, [(3, 'eth0')])
        self.assertEqual(topics, ['rt/lowstate', 'rt/lowcmd', 'rt/arm_sdk'])
        self.assertEqual(len(subs), 3)
        self.assertEqual(warnings, [])

    def test_guided_launcher_preserves_operator_inputs(self):
        capture = self.root / 'args.json'
        fake = self.root / 'fake-python'
        fake.write_text('#!' + sys.executable + '\nimport json,os,sys\nfrom pathlib import Path\nPath(os.environ["TEST_ARGS"]).write_text(json.dumps(sys.argv[1:]))\n')
        fake.chmod(0o755)
        env = dict(os.environ, PYTHON=str(fake), TEST_ARGS=str(capture))
        result = subprocess.run(['bash', str(ROOT / 'start.sh')], input='policy a\nArena ID 123\nJoe and Sam\nG1 lab\neth0\n2\nstanding test\n', text=True, capture_output=True, env=env)
        self.assertEqual(result.returncode, 0, result.stderr)
        args = json.loads(capture.read_text())
        self.assertEqual(args[1], 'record')
        self.assertEqual(args[args.index('--name') + 1], 'policy a')
        self.assertEqual(args[args.index('--operator') + 1], 'Joe and Sam')
        self.assertEqual(args[args.index('--domain') + 1], '2')

    def test_label_script_and_corrections(self):
        run = self.demo()
        result = subprocess.run(['bash', str(ROOT / 'label.sh'), str(run)], input='Joe\n001\nfailure\nmissed\n001\nsuccess\ncorrected after video review\n\n', text=True, capture_output=True, env=dict(os.environ, PYTHON=sys.executable))
        self.assertEqual(result.returncode, 0, result.stderr)
        report = kit.report(run)
        self.assertEqual(report['annotated_trials'], 1)
        self.assertEqual(report['operator_labeled_outcomes'], {'success': 1})
        self.assertEqual(len(list((run / 'events').glob('*.json'))), 2)

    def test_unique_run_paths(self):
        self.assertNotEqual(kit.new_run(self.root, 'same'), kit.new_run(self.root, 'same'))


if __name__ == '__main__':
    unittest.main()
