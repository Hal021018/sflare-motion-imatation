"""Offline SDK/clock stubs only. Never imports a native SDK or moves hardware."""
import ast
import os
import threading
import unittest
from dataclasses import replace
from pathlib import Path
from unittest import mock

from action_pipeline import ActionRepository
from movej_coordinator import (MoveJCoordinator, GAME_ACTIONS, profile_for,
                               NORMAL_PROFILE, SMOOTH_PROFILE, DANCE_PROFILE)
from motion_backend import coordinator_class
from emi_coordinator import EmiCoordinator


ROOT = Path(__file__).parent


class Robot:
    def __init__(self):
        self.pos = [0.]*7
        self.speed = [0.]*7
        self.power = 0
        self.moves = []
        self.profile = None
        self.stuck = False
        self.hook = None
        self.abort_calls = self.attach_calls = 0

    def get_state(self, **kwargs):return [self.power]
    def get_joint_pos(self, **kwargs):return self.pos[:]
    def get_joint_speed(self, **kwargs):return self.speed[:]
    def get_axis_number(self, **kwargs):return 7
    def get_limit(self, axis, **kwargs):return [200., -180.]
    def attach(self, **kwargs):self.attach_calls += 1;return 0
    def get_attached_id(self, **kwargs):return 1
    def set_speed(self, speed, **kwargs):return 0
    def set_profile(self, profile, **kwargs):self.profile=profile;return 0
    def abort(self, **kwargs):self.abort_calls += 1;return 0
    def move_j(self, pose, **kwargs):
        self.moves.append(list(pose))
        if not self.stuck:self.pos=list(pose)
        if self.hook:self.hook()
        return 0
    def emi_start(self, *args, **kwargs):raise AssertionError('EMI forbidden')
    def servo_j(self, *args, **kwargs):raise AssertionError('servo_j forbidden')


class Controller:
    def __init__(self):
        self.robot = Robot()
        self.power_calls = []
        self.disconnected = False
        self.off_fails = False
        self.status = [0]
    def connect(self, *args):return 0
    def set_data_unit(self, unit):return 0
    def add_robot(self, rid):assert rid == 1;return self.robot
    def get_status(self):return self.status[:]
    def set_power_enable(self, enabled, *args, **kwargs):
        self.power_calls.append(enabled)
        if not enabled and self.off_fails:return -1
        self.robot.power=int(enabled);return 0
    def com_close(self, name):return 0
    def check_error(self):return []
    def disconnect(self):self.disconnected=True;return 0


class SDK:
    Profile = staticmethod(lambda *values: values)
    LocationJ = staticmethod(lambda *values: values)
    def __init__(self, controller):self.controller=controller
    def Controller(self):return self.controller


class FakeCoordinator(MoveJCoordinator):
    def __init__(self):
        self.controller=Controller()
        super().__init__(SDK(self.controller), logger=lambda text: None)
        self.now=0.
        self.waits=[]
        self.hands=[]
        self.events=[]
        self.event_callback=lambda kind,state:self.events.append((kind,state))
    def _wait(self, seconds, cancel, powered):
        self._check(cancel,powered)
        self.waits.append(seconds)
        self.now += seconds
        self._check(cancel,powered)
    def _init_hands(self, events, wait=None):
        owner=self
        class Hand:
            def __getattr__(self, name):return lambda:owner.hands.append(name)
        self.right_hand=Hand()
        self.right_hand.open()
    def play(self, doc, cancel=None):
        with mock.patch('movej_coordinator.time.monotonic', lambda:self.now):
            return self.play_document(doc,cancel)


class MoveJTests(unittest.TestCase):
    def setUp(self):self.repo=ActionRepository(ROOT/'actions')

    def test_six_actions_preserve_all_points_gestures_hold_and_cleanup(self):
        for name in GAME_ACTIONS:
            with self.subTest(name=name):
                doc=self.repo.get(name);co=FakeCoordinator()
                with mock.patch('emi_coordinator.plan_action_taiji_style',side_effect=AssertionError('no interpolation')):
                    self.assertTrue(co.play(doc),co.last_error)
                expected=[list(doc.keyframes[s['frame']].right) for s in doc.program if s['op']=='move']
                self.assertEqual(co.controller.robot.moves,expected)
                self.assertEqual(co.hands,['open']+[s['action'] for s in doc.program if s['op']=='hand'])
                self.assertEqual(co.waits.count(5.),1)
                self.assertEqual(co.events.count(('phase','still')),1)
                self.assertEqual(co.controller.robot.profile,profile_for(name))
                self.assertEqual(co.controller.robot.power,0)
                self.assertTrue(co.controller.disconnected)
                self.assertTrue(co.safe_to_reuse)
                self.assertIn(('power','on'),co.events)
                self.assertIn(('power','off'),co.events)
                self.assertTrue(co.diagnostics_complete)

    def test_profiles_match_original_source_without_importing_it(self):
        source=ROOT.parents[2]/'action service'/'01_源码'/'当前版本'/'dual_arm_motion_actions.py'
        if not source.exists():self.skipTest('Original source not present on this host')
        expected={}
        for item in ast.parse(source.read_text(encoding='utf-8-sig')).body:
            if isinstance(item,ast.Assign) and isinstance(item.value,ast.Call):
                for target in item.targets:
                    if isinstance(target,ast.Name) and target.id in {'NORMAL_PROFILE','SMOOTH_PROFILE','DANCE_PROFILE'}:
                        expected[target.id]=tuple(ast.literal_eval(arg) for arg in item.value.args)
        self.assertEqual(expected,dict(NORMAL_PROFILE=NORMAL_PROFILE,SMOOTH_PROFILE=SMOOTH_PROFILE,DANCE_PROFILE=DANCE_PROFILE))

    def test_cancel_aborts_no_next_point(self):
        co=FakeCoordinator();cancel=threading.Event()
        co.controller.robot.hook=cancel.set
        self.assertFalse(co.play(self.repo.get('handshake'),cancel))
        self.assertEqual(len(co.controller.robot.moves),1)
        self.assertEqual(co.controller.robot.abort_calls,1)
        self.assertEqual(co.controller.robot.power,0)
        self.assertTrue(co.controller.disconnected)

    def test_pre_cancel_does_not_connect(self):
        co=FakeCoordinator();cancel=threading.Event();cancel.set()
        self.assertFalse(co.play(self.repo.get('like'),cancel))
        self.assertEqual(co.controller.power_calls,[])

    def test_off_target_timeout_aborts(self):
        co=FakeCoordinator();co.controller.robot.stuck=True
        self.assertFalse(co.play(self.repo.get('like')))
        self.assertIn('到位超时',co.last_error)
        self.assertEqual(len(co.controller.robot.moves),1)
        self.assertEqual(co.controller.robot.abort_calls,1)

    def test_arrival_requires_speed_not_only_position(self):
        co=FakeCoordinator();co.controller.robot.speed=[1.]*7
        self.assertFalse(co.play(self.repo.get('like')))
        self.assertIn('到位超时',co.last_error)

    def test_invalid_feedback_stops(self):
        co=FakeCoordinator();co.controller.robot.speed=[float('nan')]*7
        self.assertFalse(co.play(self.repo.get('like')))
        self.assertIn('速度反馈无效',co.last_error)

    def test_all_targets_checked_before_power(self):
        co=FakeCoordinator();doc=self.repo.get('like')
        frame=replace(doc.keyframes[0],right=(999.,0.,0.,0.,0.,0.,0.))
        doc=replace(doc,keyframes=(frame,doc.keyframes[-1]))
        self.assertFalse(co.play(doc))
        self.assertNotIn(True,co.controller.power_calls)
        self.assertEqual(co.controller.robot.moves,[])

    def test_no_disconnect_if_power_off_unconfirmed(self):
        co=FakeCoordinator();co.controller.off_fails=True
        self.assertFalse(co.play(self.repo.get('like')))
        self.assertFalse(co.safe_to_reuse)
        self.assertFalse(co.controller.disconnected)

    def test_never_take_over_preexisting_power(self):
        co=FakeCoordinator();co.controller.robot.power=1
        self.assertFalse(co.play(self.repo.get('like')))
        self.assertEqual(co.controller.power_calls,[])
        self.assertEqual(co.controller.robot.attach_calls,0)
        self.assertTrue(co.controller.disconnected)

    def test_backend_scope_and_explicit_rollback(self):
        with mock.patch.dict(os.environ,{},clear=True):
            for name in GAME_ACTIONS:self.assertIs(coordinator_class(name,'right'),MoveJCoordinator)
            self.assertIs(coordinator_class('gozero','right'),MoveJCoordinator)
            self.assertIs(coordinator_class('wave','right'),EmiCoordinator)
            self.assertIs(coordinator_class('gozero','dual'),EmiCoordinator)
        with mock.patch.dict(os.environ,{'PALLAS_GAME_MOTION_MODE':'emi'}):
            self.assertIs(coordinator_class('like','right'),EmiCoordinator)

    def test_worker_priority_zero_uses_movej_after_verified_cleanup(self):
        import motion_backend
        from test_action_session import SessionTests
        fixture=SessionTests()
        fixture.setUp()
        coordinators=[]
        def factory(**kwargs):
            if coordinators:
                self.assertTrue(coordinators[-1].controller.disconnected)
                self.assertEqual(coordinators[-1].controller.robot.power,0)
            co=FakeCoordinator()
            if not coordinators:
                co.controller.robot.hook=lambda:fixture.command('gozero')
            coordinators.append(co)
            return co
        try:
            with mock.patch.dict(os.environ,{'PALLAS_GAME_MOTION_MODE':'movej'}), \
                    mock.patch.object(motion_backend,'MoveJCoordinator',factory):
                fixture.command('like')
                fixture.node._run(fixture.node._session)
            self.assertEqual(len(coordinators),2)
            self.assertEqual(len(coordinators[0].controller.robot.moves),1)
            self.assertEqual(coordinators[1].controller.robot.moves,[[0.]*7])
            self.assertEqual(fixture.states[-1],'ready')
        finally:
            fixture.doCleanups()


if __name__=='__main__':unittest.main()
