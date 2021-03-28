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
    in_round = False
    round_size = 20


    def __init__(self):
        rospy.init_node('offboard_test', anonymous=True)
        self.des_pose = PoseStamped()
        self.isReadyToFly = False
        # location
        self.locations = numpy.matrix([[0, 0, 3, 0, 0, 0, 1],  # starting elevation
                                       [67, -20, 22, 0,0,0,1], #rock
                                       # [40.93, 3.31, 20, 0,0,0,1], #probe
                                       # [40.93, 3.31, 12.28, 0,0,0,1],#probe lowered
                                       # [13.15, -65.5, 5, 0,0,0,1], #rover
                                       # [13.15, -65.5, 0.05, 0,0,0,1], #rover lowered
                                       [0, 0, 5, 0, 0, 0, 1],
                                       ])
        self.rock_center = [55,-8,20]

        self.pose_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        drone_pose_subscriber = rospy.Subscriber('/mavros/local_position/pose', PoseStamped,
                                                 callback=self.drone_pose_cb)

        state_sub = rospy.Subscriber('/mavros/state', State, callback=self.drone_state_cb)
        attach = rospy.Publisher('/attach', String, queue_size=10)
        rate = rospy.Rate(10)  # Hz
        rate.sleep()
        self.des_pose = self.copy_pose(self.curr_drone_pose)
        self.course = numpy.copy(self.locations)
        self.course_size = self.course.shape[0]

        while not rospy.is_shutdown():
            n_rou = 1; #index to do rounadabout

            if self.isReadyToFly:
                if in_round:
                    azimuth = math.atan2(self.rock_center[1] - self.curr_drone_pose.pose.position.y,
                                     self.rock_center[0] - self.curr_drone_pose.pose.position.x)
                else: 
                    azimuth = math.atan2(self.des_pose.pose.position.y - self.curr_drone_pose.pose.position.y,
                                     self.des_pose.pose.position.x - self.curr_drone_pose.pose.position.x)
                az_quat = quaternion_from_euler(0, 0, azimuth)
                des = self.set_desired_pose(az_quat).position
                curr = self.curr_drone_pose.pose.position
                dist = math.sqrt(
                    (curr.x - des.x) * (curr.x - des.x) + (curr.y - des.y) * (curr.y - des.y) + (curr.z - des.z) * (curr.z - des.z))
                if dist < self.distThreshold:
                    if in_round and self.waypointIndex == self.round_size -1: #edit for length of circle array later
                        in_round = False
                        self.locations = self.course
                        self.waypointIndex = n_rou+1
                    elif not in_round and self.waypointIndex is self.course_size-1: 
                        self.waypointIndex = 0
                    elif not in_round and self.waypointIndex is n_rou:
                        x = self.curr_drone_pose.pose.position.x
                        y = self.curr_drone_pose.pose.position.y
                        z = self.curr_drone_pose.pose.position.z
                        self.locations = self.roundabout(x,y,z)
                        print(self.locations,self.des_pose.pose)
                    else: 
                        self.waypointIndex += 1

            self.pose_pub.publish(self.des_pose)
            rate.sleep()

    def roundabout(self, x, y, z):
        in_round = True
        r = 3  # calculated 7 to be optimal radius
        dtheta = 2 * math.pi / N
        xc = x - r
        yc = y
        theta = 0
        circle = []
        for n in range(self.round_size):
            theta += dtheta
            poly_x = r * math.cos(theta) + xc
            poly_y = r * math.sin(theta) + yc
            poly_z = z - 2
            circle.append([poly_x, poly_y, poly_z, 1])
            # print("circle", poly_x, poly_y, poly_z)
        self.waypointIndex = 0
        return numpy.array(circle, dtype=float)

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
