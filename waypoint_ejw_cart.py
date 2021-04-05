#!/usr/bin/env python
# This script tests if the self-created inverted pendulum can be controlled by joint velocity controller

from __future__ import print_function

import rospy
import math
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState


class Testbed(object):
    """ regulate card """
    pos_cart = 0
    vel_cart = 0
    P_gain = 200
    I_gain = 0
    D_gain = 100

    def __init__(self):
        self._sub_invpend_states = rospy.Subscriber('/invpend/joint_states', JointState, self.jointStateCallback)
        self._pub_vel_cmd = rospy.Publisher('/invpend/joint1_velocity_controller/command', Float64, queue_size=1)
        self._pub_pos_cmd_pend = rospy.Publisher('/invpend/joint2_position_controller/command',Float64,queue_size=1)

    def jointStateCallback(self, data):
        self.pos_cart = data.position[1]
        self.vel_cart = data.velocity[1]


    def return_home(self):
        g = 0 #goal position home
        e_prev = g - self.pos_cart
        e_sum = 0 #initialize integral value
        while not rospy.is_shutdown():  # this part is super important apparently to get the cart to keep moving
            rate = rospy.Rate(5)
            self._pub_pos_cmd_pend.publish(0)
            e = g - self.pos_cart #error from home and position
            P = e * self.P_gain #proportional
            e_sum = (e_sum + e * self.vel_cart) * self.I_gain  #intetgral
            if self.pos_cart != 0:
                dedt = (0-self.vel_cart) * self.D_gain  # derivative
            else:
                dedt = 1
            self.des_force = P + e_sum + dedt
            print(self.pos_cart, self.vel_cart, self.des_force)
            self._pub_vel_cmd.publish(self.des_force)
            e_prev = e

            if (self.des_force > 0.001 or self.des_force < -0.001):
                continue
            else:
                break




    def cart_pole_move(self,x,z):
        theta = math.acos(z/2)
        # theta = 2
        if x > 0:
            g = x - math.sin(theta)
        else:
            g = x + math.sin(theta)
        e_prev = g - self.pos_cart
        e_sum = 0  # initialize integral value
        while not rospy.is_shutdown():
            rate = rospy.Rate(5)
            self._pub_pos_cmd_pend.publish(theta)

            e = g - self.pos_cart #error from home and position
            P = e * self.P_gain #proportional
            e_sum = (e_sum + e * self.vel_cart) * self.I_gain  #integral
            if self.pos_cart != 0:
                dedt = (e - self.vel_cart) * self.D_gain # derivative
            else:
                dedt = 1
            self.des_force = P + e_sum + dedt
            print(self.pos_cart, self.vel_cart, self.des_force)
            self._pub_vel_cmd.publish(self.des_force)
            e_prev = e
            rospy.sleep(1. / 50)

            if (self.des_force > 0.01 or self.des_force < -0.01):
                continue
            else:
                break



def main():
    """ Perform testing actions provided by Testbed class
    """
    rospy.init_node('cart_wobble')
    cart = Testbed()
    cart.return_home()
    cart.cart_pole_move(-1,2)
    cart.cart_pole_move(1,-2)
    cart.return_home()


if __name__ == '__main__':
    main()
