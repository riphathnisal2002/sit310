#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped, FSMState

class BackAndForthMotion:
    def __init__(self):
        rospy.init_node('back_and_forth_motion_node', anonymous=True)
        rospy.loginfo("Node initialized")

        self.pub = rospy.Publisher('/birdie/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/birdie/fsm_node/mode', FSMState, self.fsm_callback)

        # Speed-duration pairs: (speed, duration in seconds)
        self.tasks = [
            {'v': 0.2, 'duration': 2.0},   # Forward slow
            {'v': -0.2, 'duration': 2.0},  # Backward slow
            {'v': 0.5, 'duration': 1.5},   # Forward fast
            {'v': -0.5, 'duration': 1.5},  # Backward fast
        ]

        self.current_task = None
        self.task_start_time = None
        self.is_running = False
        self.break_time = rospy.Duration(1.0)
        self.break_start = None

        rospy.Timer(rospy.Duration(0.01), self.timer_callback)

    def fsm_callback(self, msg):
        if msg.state == "LANE_FOLLOWING":
            rospy.loginfo("FSM: Starting back-and-forth motion")
            self.is_running = True
            self.current_task = None
            self.task_start_time = None
            self.break_start = None
            self.reset_tasks()
        else:
            rospy.loginfo("FSM: Stopping motion")
            self.is_running = False
            self.stop_robot()

    def reset_tasks(self):
        self.task_queue = list(self.tasks)

    def stop_robot(self):
        cmd = Twist2DStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.v = 0.0
        cmd.omega = 0.0
        self.pub.publish(cmd)

    def timer_callback(self, event):
        if not self.is_running:
            return

        now = rospy.Time.now()

        # Handle pause between tasks
        if self.break_start is not None:
            if (now - self.break_start) < self.break_time:
                return
            self.break_start = None

        # Load next task
        if self.current_task is None:
            if not self.task_queue:
                rospy.loginfo("Motion sequence completed")
                self.is_running = False
                self.stop_robot()
                return

            self.current_task = self.task_queue.pop(0)
            self.task_start_time = now
            rospy.loginfo(f"Starting motion: v={self.current_task['v']} for {self.current_task['duration']}s")

        # Execute current task
        elapsed = (now - self.task_start_time).to_sec()
        if elapsed >= self.current_task['duration']:
            rospy.loginfo("Motion task complete")
            self.current_task = None
            self.stop_robot()
            if self.task_queue:
                self.break_start = now
        else:
            cmd = Twist2DStamped()
            cmd.header.stamp = now
            cmd.v = self.current_task['v']
            cmd.omega = 0.0
            self.pub.publish(cmd)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        motion_node = BackAndForthMotion()
        motion_node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Unhandled error: {str(e)}")

