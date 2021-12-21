#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState


class ArmStatePublisher(object):
    def __init__(self, dof='6dof'):
        self.rpub = rospy.Publisher('/movo/right_arm/state', JointState, queue_size=10)
        self.lpub = rospy.Publisher('/movo/left_arm/state', JointState, queue_size=10)
        rospy.Subscriber('/movo/right_arm_controller/state', JointTrajectoryControllerState, self.right_callback)
        rospy.Subscriber('/movo/left_arm_controller/state', JointTrajectoryControllerState, self.left_callback)

        rospy.init_node('arm_state_publisher')
        self.right_positions = [0.0]*6
        self.right_velocities = [0.0]*6
        self.left_positions = [0.0]*6
        self.left_velocities = [0.0]*6
        self.seq = 0


        self.joint_names = ['shoulder_pan_joint',
                        'right_shoulder_lift_joint',
                        'right_elbow_joint',
                        'right_wrist_1_joint',
                        'right_wrist_2_joint',
                        'right_wrist_3_joint']

    def right_callback(self, data):
        self.right_positions = data.actual.positions
        self.right_velocities = data.actual.velocities

    def left_callback(self,data):
        self.left_positions = data.actual.positions
        self.left_velocities = data.actual.velocities

    def publishStates(self, laterality):
        msg = JointState()
        msg.header.seq = self.seq
        msg.header.frame_id = ''
        msg.header.stamp = rospy.get_rostime()
        msg.name  = ['%s_' % laterality +jn for jn in self.joint_names]
        msg.position =  self.right_positions if laterality == "right" else self.left_positions
        msg.velocity = self.right_velocities if laterality == "right" else self.left_velocities

        self.seq += 1
        self.rpub.publish(msg) if laterality == "right" else self.lpub.publish(msg)


def main():

    pub = ArmStatePublisher()
    rate = rospy.Rate(20)

    print("hellow")


    while not rospy.is_shutdown():
        pub.publishStates("right")
        pub.publishStates("left")
        rate.sleep()

    rospy.spin()
    


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass