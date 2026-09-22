"""Deterministic tests without SDK imports or robot communication."""
from dataclasses import replace
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from robot_safety.core import Config, Supervisor
from robot_safety.config import JointCommandLimits


class SupervisorTests(unittest.TestCase):
    def test_custom_command_limits_are_enforced(self):
        limits = replace(JointCommandLimits(), position_rad=(-.1, .1))
        supervisor = self.make(command_limits=limits)
        message = self.arm(supervisor)
        self.assertTrue(supervisor.accept(message, .01))
        message['seq'] = 1
        message['q'][0] = .100001
        self.assertFalse(supervisor.accept(message, .02))
        self.assertEqual(supervisor.stop_reason, 'invalid_action')

    def test_invalid_limit_configuration_is_rejected(self):
        for bounds in ((1., -1.), (0., float('inf')), (False, 1.), [0., 1.]):
            with self.subTest(bounds=bounds), self.assertRaises(ValueError):
                JointCommandLimits(position_rad=bounds)
        with self.assertRaises(ValueError):
            JointCommandLimits(stiffness_nm_per_rad=(-1., 1.))
        with self.assertRaises(ValueError):
            Config(observation_history_size=0)

    def test_configured_observation_limits_and_history(self):
        supervisor = self.make(observation_history_size=1,
                               robot_position_bounds_rad=(-1., 1.))
        message = self.arm(supervisor)
        supervisor.state({'q': [0., 0.], 'dq': [0., 0.]}, False, .01)
        self.assertFalse(supervisor.accept(message, .02))
        self.assertEqual(len(supervisor.observation_received_times_secs), 1)

        supervisor = self.make(robot_position_bounds_rad=(-1., 1.))
        supervisor.state({'q': [2., 0.], 'dq': [0., 0.]}, False, .01)
        self.assertEqual(supervisor.stop_reason, 'invalid_robot_state')

    def make(self, **overrides):
        config = replace(
            Config(joints=2, action_timeout=10., observation_timeout=10.,
                   robot_timeout=10., loop_timeout=10.),
            **overrides)
        supervisor = Supervisor(config)
        supervisor.state({'q': [0., 0.], 'dq': [0., 0.]}, False, 0.)
        supervisor.cycle(0.)
        return supervisor

    def make_policy_command(self, supervisor, seq=0):
        return {
            'session': supervisor.policy_session_id,
            'seq': seq,
            'state_id': supervisor.latest_observation_id,
            'q': [.1, -.1],
            'dq': [0., 0.],
            'kp': [5., 5.],
            'kd': [1., 1.],
            'tau': [0., 0.],
        }

    def arm(self, supervisor):
        self.assertNotIn('error', supervisor.control('arm', 0.))
        return self.make_policy_command(supervisor)

    def test_startup_and_restart_are_inhibited(self):
        first, restarted = self.make(), self.make()
        self.assertEqual(first.supervisor_phase, 'INHIBITED')
        self.assertEqual(restarted.supervisor_phase, 'INHIBITED')
        self.assertIsNone(restarted.policy_session_id)
        self.assertNotEqual(first.supervisor_instance_id, restarted.supervisor_instance_id)
        status = first.status(0.)
        self.assertTrue(status['dry_run'])
        self.assertFalse(status['motor_publisher'])
        self.assertFalse(status['can_enforce_motor_damping'])
        self.assertIsNone(status['motor_application_latency'])
        self.assertIsNone(status['controller_radio_live'])

    def test_arm_alone_never_selects_policy(self):
        supervisor = self.make()
        self.arm(supervisor)
        self.assertEqual(supervisor.cycle(.01)['kind'], 'damping')
        self.assertEqual(supervisor.supervisor_phase, 'ARMED')

    def test_watchdog_boundaries(self):
        cases = (
            ('action_timeout', 'action_timeout', False),
            ('robot_timeout', 'robot_state_timeout', False),
            ('observation_timeout', 'observation_timeout', True),
            ('loop_timeout', 'loop_progress', False),
        )
        for setting, reason, running in cases:
            for cur_time_secs in (.199999, .2, .200001):
                with self.subTest(setting=setting, cur_time_secs=cur_time_secs):
                    supervisor = self.make(**{setting: .2})
                    message = self.arm(supervisor)
                    if running:
                        self.assertTrue(supervisor.accept(message, 0.))
                    output = supervisor.cycle(cur_time_secs)
                    if cur_time_secs < .2:
                        self.assertNotEqual(supervisor.supervisor_phase, 'STOP_LATCHED')
                    else:
                        self.assertEqual(supervisor.supervisor_phase, 'STOP_LATCHED')
                        self.assertEqual(supervisor.stop_reason, reason)
                        self.assertEqual(output['kind'], 'damping')

    def test_late_state_cannot_hide_timeout(self):
        supervisor = self.make(robot_timeout=.2)
        self.arm(supervisor)
        supervisor.state({'q': [0., 0.], 'dq': [0., 0.]}, False, .2)
        self.assertEqual(supervisor.stop_reason, 'robot_state_timeout')

    def test_late_action_cannot_hide_timeout(self):
        supervisor = self.make(action_timeout=.2)
        message = self.arm(supervisor)
        self.assertFalse(supervisor.accept(message, .2))
        self.assertEqual(supervisor.stop_reason, 'action_timeout')

    def test_stop_in_every_phase_and_held_reset(self):
        for phase in ('INHIBITED', 'ARMED', 'RUNNING', 'STOP_LATCHED'):
            with self.subTest(phase=phase):
                supervisor = self.make()
                if phase in ('ARMED', 'RUNNING'):
                    message = self.arm(supervisor)
                    if phase == 'RUNNING':
                        supervisor.accept(message, 0.)
                elif phase == 'STOP_LATCHED':
                    supervisor.stop('operator_stop', 0.)
                supervisor.state({'q': [0., 0.], 'dq': [0., 0.]},
                                 True, .01, stop_requested=True)
                self.assertEqual(supervisor.supervisor_phase, 'STOP_LATCHED')
                self.assertIsNone(supervisor.policy_session_id)
                self.assertIsNone(supervisor.accepted_policy_command)
                self.assertIn('error', supervisor.control('reset', .02))
                self.assertIn('error', supervisor.control('arm', .02))
                output = supervisor.cycle(.02)
                self.assertEqual(output['kind'], 'damping')
                for key in ('kp', 'dq', 'tau'):
                    self.assertEqual(output[key], [0., 0.])

    def test_same_cycle_stop_wins_over_reset_and_arm(self):
        supervisor = self.make()
        supervisor.control('stop', .01)
        self.assertIn('error', supervisor.control('reset', .01))
        self.assertIn('error', supervisor.control('arm', .01))
        self.assertEqual(supervisor.supervisor_phase, 'STOP_LATCHED')

    def test_reset_requires_separate_arm_and_revokes_session(self):
        supervisor = self.make()
        old = self.arm(supervisor)
        supervisor.accept(old, 0.)
        supervisor.control('stop', .01)
        supervisor.control('reset', .02)
        self.assertEqual(supervisor.supervisor_phase, 'INHIBITED')
        self.assertIsNone(supervisor.policy_session_id)
        self.assertFalse(supervisor.accept(old, .03))
        supervisor.control('arm', .04)
        self.assertNotEqual(supervisor.policy_session_id, old['session'])
        self.assertFalse(supervisor.accept(old, .05))
        self.assertEqual(supervisor.cycle(.05)['kind'], 'damping')

    def test_invalid_actions_latch_without_refreshing_deadline(self):
        mutations = (
            {'seq': -1},
            {'seq': True},
            {'session': 'unknown'},
            {'state_id': 999},
            {'state_id': []},
            {'q': [float('nan'), 0.]},
            {'dq': [float('inf'), 0.]},
            {'kp': [-1., 0.]},
            {'kd': [51., 0.]},
            {'tau': [201., 0.]},
            {'q': [0.]},
        )
        for mutation in mutations:
            with self.subTest(mutation=mutation):
                supervisor = self.make()
                message = self.arm(supervisor)
                message.update(mutation)
                self.assertFalse(supervisor.accept(message, .01))
                self.assertEqual(supervisor.time_action_watchdog_refreshed_secs, 0.)
                self.assertEqual(supervisor.supervisor_phase, 'STOP_LATCHED')

    def test_duplicate_sequence_and_stale_observation(self):
        supervisor = self.make()
        message = self.arm(supervisor)
        self.assertTrue(supervisor.accept(message, .01))
        self.assertFalse(supervisor.accept(message, .02))
        self.assertEqual(supervisor.time_action_watchdog_refreshed_secs, .01)

        supervisor = self.make(observation_timeout=.2)
        message = self.arm(supervisor)
        self.assertFalse(supervisor.accept(message, .2))
        self.assertEqual(supervisor.supervisor_phase, 'STOP_LATCHED')

    def test_latest_valid_action_selected_and_latch_persists(self):
        supervisor = self.make()
        message = self.arm(supervisor)
        supervisor.accept(message, .01)
        newer = self.make_policy_command(supervisor, seq=1)
        newer['q'] = [.2, -.2]
        supervisor.accept(newer, .02)
        self.assertEqual(supervisor.cycle(.02)['q'], newer['q'])
        supervisor.stop('policy_disconnected', .03)
        for seq in range(2, 100):
            newer['seq'] = seq
            self.assertFalse(supervisor.accept(newer, .04))
            self.assertEqual(supervisor.cycle(.04)['kind'], 'damping')

    def test_status_cannot_mutate_internal_state(self):
        supervisor = self.make()
        status = supervisor.status(0.)
        status['state']['q'][0] = 99.
        status['output']['kd'][0] = 99.
        self.assertEqual(supervisor.latest_robot_observation['q'][0], 0.)
        self.assertEqual(supervisor.last_selected_command['kd'][0], 1.)


if __name__ == '__main__':
    unittest.main()
