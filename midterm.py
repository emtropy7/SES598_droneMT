"""
testing offboard positon control with a simple takeoff script
"""

import rospy
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import math
import numpy

from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler


class OffbPosCtl:

    curr_drone_pose = PoseStamped()
    waypointIndex = 0
    distThreshold = 1
    sim_ctr = 1


    def __init__(self):
        rospy.init_node('offboard_test', anonymous=True)
        self.des_pose = PoseStamped()
        self.isReadyToFly = False
        # location
        self.locations = numpy.matrix([[0, 0, 3, 0, 0, 0, 1],  # starting elevation
                                       # [55, -8, 20, orientation[0], orientation[1], orientation[2], orientation[3]], #rock
                                       # [40.93, 3.31, 20, orientation[0], orientation[1], orientation[2], orientation[3]], #probe                              # [40.93, 3.31, 12.28, orientation[0], orientation[1], orientation[2], orientation[3]],
                                       # [13.15, -65.5, 0.05, orientation[0], orientation[1], orientation[2], orientation[3]], #probe lowered
                                       #
                                       # [13.15, -65.5, 5, orientation[0], orientation[1], orientation[2], orientation[3]], #rover
                                       # [0, 0, 5, orientation[0],orientation[1],orientation[2],orientation[3]],
                                       ])
        self.rock_center = quaternion_from_euler(55,-8,20)

        self.pose_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        drone_pose_subscriber = rospy.Subscriber('/mavros/local_position/pose', PoseStamped,
                                                 callback=self.drone_pose_cb)

        state_sub = rospy.Subscriber('/mavros/state', State, callback=self.drone_state_cb)
        attach = rospy.Publisher('/attach', String, queue_size=10)
        rate = rospy.Rate(10)  # Hz
        rate.sleep()
        self.des_pose = self.copy_pose(self.curr_drone_pose)


        while not rospy.is_shutdown():
            shape = self.locations.shape
            #print(self.sim_ctr, shape[0], self.waypointIndex)

            if self.waypointIndex is shape[0]:
                self.waypointIndex = 0
                x = self.curr_drone_pose.pose.position.x
                y = self.curr_drone_pose.pose.position.y
                self.locations = self.roundabout(x,y)
                self.sim_ctr += 1




            if self.isReadyToFly:
                azimuth = math.atan2(self.des_pose.pose.position.y - self.curr_drone_pose.pose.position.y,
                                     self.des_pose.pose.position.x - self.curr_drone_pose.pose.position.x)
                az_quat = quaternion_from_euler(0, 0, azimuth)
                des = self.set_desired_pose(az_quat).position
                curr = self.curr_drone_pose.pose.position
                dist = math.sqrt(
                    (curr.x - des.x) * (curr.x - des.x) + (curr.y - des.y) * (curr.y - des.y) + (curr.z - des.z) * (
                            curr.z - des.z))
                if dist < self.distThreshold:
                    self.waypointIndex += 1

            self.pose_pub.publish(self.des_pose)
            rate.sleep()

#first attempt at making roundabout function
    def roundabout(self, x, y):
        N = 20
        r = 7
        dtheta = 2 * math.pi / N
        xc = x - r
        yc = y
        theta = 0
        circle = []
        for n in range(N):
            theta += dtheta
            poly_x = r * math.cos(theta) + xc
            poly_y = r * math.sin(theta) + yc
            circle.append([poly_x, poly_y, 10, 1])
            print("circle", poly_x, poly_y)
        self.waypointIndex = 0
        return numpy.array(circle,dtype=float)

    def set_desired_pose(self, _att_quat):
        self.des_pose.pose.position.x = self.locations[self.waypointIndex, 0]
        self.des_pose.pose.position.y = self.locations[self.waypointIndex, 1]
        self.des_pose.pose.position.z = self.locations[self.waypointIndex, 2]
        orientation = _att_quat
        self.des_pose.pose.orientation = Quaternion(orientation[0], orientation[1], orientation[2], orientation[3])
        return self.des_pose.pose

    def copy_pose(self, pose):
        pt = pose.pose.position
        quat = pose.pose.orientation
        copied_pose = PoseStamped()
        copied_pose.header.frame_id = pose.header.frame_id
        copied_pose.pose.position = Point(pt.x, pt.y, pt.z)
        copied_pose.pose.orientation = Quaternion(quat.x, quat.y, quat.z, quat.w)
        return copied_pose

    def drone_pose_cb(self, msg):
        self.curr_drone_pose = msg

    def rover_pose_cb(self, msg):
        self.curr_rover_pose = msg

    def drone_state_cb(self, msg):
        #print msg.mode
        if (msg.mode == 'OFFBOARD'):
            self.isReadyToFly = True
            #print "readyToFly"


if __name__ == "__main__":
    OffbPosCtl()
