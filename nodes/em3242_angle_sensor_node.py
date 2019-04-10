#!/usr/bin/env python
from __future__ import print_function
import threading
import rospy
import std_msgs.msg

from em3242_angle_sensor import EM3242_AngleSensor
from em3242_angle_sensor_ros.msg import EM3242_AngleSensorData 


class EM3242_AngleSensorNode(object):

    def __init__(self):
        self.is_first = True
        self.last_angle = 0.0
        self.cumulative_angle = 0.0

        rospy.init_node('em3242_angle_sensor')
        self.loop_rate = rospy.Rate(10.0)
        self.data_publisher = rospy.Publisher('em3242_angle_sensor_data', EM3242_AngleSensorData, queue_size=10) 

        self.port = rospy.get_param('/em3242_angle_sensor/port', '/dev/ttyACM0')
        self.em3242 = EM3242_AngleSensor(self.port,sigint_handler=False)
        self.em3242.start(callback=self.on_em3242_data)


    def update_cumulative_angle(self,angle):
        if self.is_first:
            self.is_first = False
            self.cumulative_angle = angle
        else:
            diff_angle = angle - self.last_angle
            if diff_angle > 180.0:
                self.cumulative_angle += (360 - angle) - self.last_angle
            elif diff_angle < -180.0:
                self.cumulative_angle += angle + (360.0 - self.last_angle)
            else:
                self.cumulative_angle += diff_angle
        self.last_angle = angle


    def on_em3242_data(self,angle):
        self.update_cumulative_angle(angle)
        msg_data = EM3242_AngleSensorData()
        msg_data.header.stamp = rospy.Time.now()
        msg_data.angle = angle
        msg_data.cumulative_angle = self.cumulative_angle
        self.data_publisher.publish(msg_data)


    def run(self):
        while not rospy.is_shutdown():
            self.loop_rate.sleep()
        self.em3242.stop()

# ---------------------------------------------------------------------------------------
if __name__ == '__main__':

    node = EM3242_AngleSensorNode()
    node.run()


