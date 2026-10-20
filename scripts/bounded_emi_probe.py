"""Bounded SDK comparison: J7 +0.5 degrees and back, always confirm power-off.

Run only on the supervised test machine; no alarm clearing or limit changes.
"""
import math
import time
import PallasSDK as sdk


def ok(value, name):
    if value != 0:
        raise RuntimeError('%s returned %r' % (name, value))


def main():
    controller = sdk.Controller()
    connected = False
    robot = None
    started = False
    try:
        ok(controller.connect('192.168.3.100', 2094), 'connect')
        connected = True
        ok(controller.set_data_unit(1), 'degrees')
        robot = controller.add_robot(1)
        print('SDK', sdk.__file__, 'status', controller.get_status(), flush=True)
        if controller.get_status()[0] != 0 or robot.get_state()[0] != 0:
            raise RuntimeError('Protection active or unexpected pre-existing power')
        if robot.get_axis_number() != 7:
            raise RuntimeError('Expected seven axes')
        initial = list(robot.get_joint_pos())[:7]
        if not all(math.isfinite(x) for x in initial):
            raise RuntimeError('Invalid initial feedback')
        for axis, value in enumerate(initial, 1):
            lo, hi = sorted(robot.get_limit(axis)[:2])
            if not lo <= value <= hi:
                raise RuntimeError('Initial joint outside limits')
            if axis == 7 and value + .5 >= hi:
                raise RuntimeError('No J7 probe clearance')
        errors_before = controller.check_error()
        print('initial', initial, 'errors_before', errors_before, flush=True)
        ok(robot.attach(), 'attach')
        ok(robot.set_speed(100), 'speed')
        ok(robot.set_profile(sdk.Profile(100, 100, 100, 100, .1, .1, -1)), 'profile')
        ok(controller.idn_write('1203.150', '1.0.0', '4'), 'emi parameter')
        if controller.get_status()[0] != 0:
            raise RuntimeError('Protection active')
        ok(controller.set_power_enable(True, 1, timeout=5000), 'power on')
        if robot.get_state()[0] != 1:
            raise RuntimeError('Power on not confirmed')
        ok(robot.abort(), 'abort')
        actual = list(robot.get_joint_pos())[:7]
        if max(abs(a-b) for a,b in zip(actual, initial)) > .5:
            raise RuntimeError('Post-power position changed')
        ok(robot.emi_start(0x8800, 1, .02, 0, 0), 'emi start')
        started = True
        peak = None
        for index in range(151):
            if index % 5 == 0:
                if controller.get_status()[0] != 0:
                    raise RuntimeError('Protection active during probe')
                actual = list(robot.get_joint_pos())[:7]
                if max(abs(a-b) for a,b in zip(actual, initial)) > 1.5:
                    raise RuntimeError('Probe displacement envelope exceeded')
                errors = controller.check_error()
                if any('-5314' in str(item) and item not in errors_before for item in errors):
                    raise RuntimeError('Invalid robot ID during streaming: %r' % errors)
            offset = .5 * max(0, min(1, (index-25)/50, (150-index)/50))
            command = initial + [0.] * 8
            command[6] += offset
            ok(robot.servo_j(command, 0., .04, t=.02, lookahead_time=.03, gain=900.), 'servo_j')
            if index == 100:
                peak = list(robot.get_joint_pos())[6]
            time.sleep(.02)
        print('RESULT initial_J7', initial[6], 'peak_J7', peak,
              'final', list(robot.get_joint_pos())[:7],
              'errors_after', controller.check_error(), flush=True)
    finally:
        if connected:
            if started:
                try:
                    ok(robot.emi_end(timeout=5000), 'emi end')
                except Exception as exc:
                    print('EMI_END_ERROR', repr(exc), flush=True)
            try:
                ok(controller.set_power_enable(False, 1, timeout=5000), 'power off')
            except Exception:
                robot.abort(timeout=5000)
                robot.emi_stop(timeout=5000)
                robot.emi_end(timeout=5000)
                ok(controller.set_power_enable(False, 1, timeout=5000), 'retry power off')
            if robot is not None and robot.get_state()[0] != 0:
                raise RuntimeError('Power-off NOT confirmed; retain connection')
            print('POWER_OFF_CONFIRMED', flush=True)
            ok(controller.disconnect(), 'disconnect')


if __name__ == '__main__':
    main()
