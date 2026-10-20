"""One bounded ROS wave diagnostic run using the unchanged action JSON."""

import os
import sys
import time

import rclpy
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
from dual_arm_fist_stretch import EmiActionNode


def main():
    if os.environ.get("PALLAS_ARM_MODE") != "single":
        raise RuntimeError("PALLAS_ARM_MODE must be single")
    rclpy.init()
    node = EmiActionNode()
    try:
        deadline = time.monotonic() + 10.0
        while node._e_stop_pressed is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        print("preflight_e_stop", node._e_stop_pressed, flush=True)
        if node._e_stop_pressed is not False:
            raise RuntimeError("No confirmed released e-stop; wave not started")
        command = String()
        command.data = "wave"
        node._on_command(command)
        if node._worker is None:
            raise RuntimeError("Wave worker did not start")
        deadline = time.monotonic() + 45.0
        while node._worker is not None:
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() >= deadline:
                print("wave_timeout_requesting_cancel", flush=True)
                if node._cancel is not None:
                    node._cancel.set()
                break
        cleanup_deadline = time.monotonic() + 12.0
        while node._worker is not None and time.monotonic() < cleanup_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        print("final_status", node._status_text,
              "e_stop", node._e_stop_pressed,
              "worker_active", node._worker is not None, flush=True)
        if node._worker is not None:
            raise RuntimeError("Cleanup did not finish before deadline")
    finally:
        if not node.shutdown():
            raise RuntimeError("Wave worker still active during shutdown")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
