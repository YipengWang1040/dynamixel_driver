#!/usr/bin/python3
import rospy
from dynamixel_driver.msg import MotorPosition
from dynamixel_driver.srv import SetPositionTarget
import std_msgs.msg

import yaml

import motor

class dynamixel_driver:
    def __init__(self) -> None:
        rospy.init_node("dynamixel_driver")
        self.config_path=rospy.get_param("~config_file")
        print(self.config_path)

        
        with open(self.config_path) as config_file:
            config=yaml.load(config_file, Loader=yaml.BaseLoader)
            self.port=int(config["port"])
            self.velocity=float(config["velocity"])
            self.zero_pos=float(config["zero_pos"])
        
        self.device=motor.initialize(self.port)
        if(not self.device):
            return
        
        self.pub=rospy.Publisher("motor_position",MotorPosition)
        self.rate=rospy.Rate(200)
        try:
            # set up subscribers and publishers
            s=rospy.Service("set_position",SetPositionTarget,self.set_position_callback)
            self.main_loop()
        finally:
            motor.finalize(self.device)

    def set_position_callback(self, theta):
        motor.set_goal_rad(self.device,theta-self.zero_pos)
        return True

    def main_loop(self):
        while(not rospy.is_shutdown()):
            value=motor.read_dynamixel(self.device, motor.ADDR_TABLE["Present Position"])
            msg=MotorPosition()
            msg.header=std_msgs.msg.Header()
            msg.header.stamp=rospy.Time.now()
            msg.theta=value
            msg.theta_base=self.zero_pos
            self.pub.publish(msg)
            self.rate.sleep()
            
if __name__ == "__main__":
    _=dynamixel_driver()