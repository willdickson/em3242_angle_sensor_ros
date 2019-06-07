#!/usr/bin/env python
from __future__ import print_function
import threading
import rospy
import std_msgs.msg
import numpy as np

from em3242_angle_sensor import EM3242_AngleSensor
from em3242_angle_sensor_ros.msg import EM3242_AngleSensorData 


class EM3242_AngleSensorNode(object):

    def __init__(self):
        self.is_first = True
        self.last_angle = 0.0
        self.last_t = 0.0
        self.cumulative_angle = 0.0

        rospy.init_node('em3242_angle_sensor')
        self.loop_rate = rospy.Rate(10.0)
        self.data_publisher = rospy.Publisher('em3242_angle_sensor_data', EM3242_AngleSensorData, queue_size=10) 

        self.port = rospy.get_param('/em3242_angle_sensor/port', '/dev/ttyACM0')
        self.max_speed = rospy.get_param('/em3242_angle_sensor/max_speed', 1000.0)
        self.em3242 = EM3242_AngleSensor(self.port,sigint_handler=False)
        self.em3242.start(callback=self.on_em3242_data)


    def update_cumulative_angle(self,t,angle):
        if self.is_first:
            self.is_first = False
            self.last_angle = angle
            self.last_t = t
            self.cumulative_angle = angle
        else:
            step = get_cumulative_step(angle, self.last_angle)
            dt = t - self.last_t
            speed = abs(step/dt)
            if speed  < self.max_speed:
                self.last_angle = angle
                self.last_t = t
            else:
                step = 0.0
            self.cumulative_angle += step


    def on_em3242_data(self,angle):
        t_ros = rospy.Time.now()
        t_sec = t_ros.to_sec()
        self.update_cumulative_angle(t_sec,angle)
        msg_data = EM3242_AngleSensorData()
        msg_data.header.stamp = t_ros
        msg_data.angle = angle
        msg_data.cumulative_angle = self.cumulative_angle
        self.data_publisher.publish(msg_data)


    def run(self):
        while not rospy.is_shutdown():
            self.loop_rate.sleep()
        self.em3242.stop()


# Utility functions
# --------------------------------------------------------------------------------------

def get_cumulative_step(angle_curr, angle_last):
    inner_dist = abs(angle_curr - angle_last)
    outer_dist = 360.0 - inner_dist
    step_sign = np.sign(outer_dist - inner_dist)*np.sign(angle_curr - angle_last)
    return step_sign*min(inner_dist, outer_dist)


# ---------------------------------------------------------------------------------------
if __name__ == '__main__':

    node = EM3242_AngleSensorNode()
    node.run()


