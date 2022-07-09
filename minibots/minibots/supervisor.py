import rclpy
import os
import sys
from geometry_msgs.msg import Twist
import webots_ros2_driver_webots
sys.path.insert(1, os.path.dirname(webots_ros2_driver_webots.__file__))
#from controller import Supervisor as sv


class Supervisor():
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        rclpy.init(args=None)
        self.__node = rclpy.create_node('Supervisor')
        self.__target_pose = Twist()
        self.__node.create_subscription(Twist, 'robotatt', self.__cmd_pose_callback, 10)
        self.__myrobot_node=0
        self.__translational_field=0

        
        

    def __cmd_pose_callback(self, pose):        
        self.__target_pose = pose
       

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0.00)
        x = self.__target_pose.linear.x/1000
        y = self.__target_pose.linear.y/1000
        z = self.__target_pose.linear.z+0.04
        rob_id=self.__target_pose.angular.x
        print(rob_id)
        print(x,y,z)
        if rob_id==1:
            self.__myrobot_node = self.__robot.getFromDef('ROB1')
            self.__translation_field = self.__myrobot_node.getField('translation')
            inst_loc=[x,y,z]
            self.__translation_field.setSFVec3f(inst_loc)
        elif rob_id==2:
            self.__myrobot_node = self.__robot.getFromDef('ROB2')
            self.__translation_field = self.__myrobot_node.getField('translation')
            inst_loc=[x,y,z]
            self.__translation_field.setSFVec3f(inst_loc)

        
        
       



def main():
    print('Hi from minibots.')
    


if __name__ == '__main__':
    main()
