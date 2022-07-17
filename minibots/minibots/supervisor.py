import rclpy
import os
import sys

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image
from visualization_msgs.msg import Marker, MarkerArray

import webots_ros2_driver_webots
sys.path.insert(1, os.path.dirname(webots_ros2_driver_webots.__file__))



class Supervisor():
    def init(self, webots_node, properties):
        timer_period=0.001
        # create a webots object
        self.__robot = webots_node.robot
        rclpy.init(args=None)
        #create ros node
        self.__node = rclpy.create_node('Supervisor')
        #create sensor list
        self.__sensors=[]
        #create sensor name list
        self.__sensorNames = ['l1','c1','l2','c2','l3','c3','l4','c4','l5','c5','l6','c6','l7','c7','l8','c8']
        #initialize target pose msg
        self.__target_pose = Twist()
        #initialize left sensor msg
        self.__lidar_sensor_value = LaserScan()
        #initialize right sensor msg
        self.__camera_sensor_value = Image()
        #initialize obstacles 
        self.__box=0
        self.__translationalBox=0
        self.__sphere=0
        self.__translationalSphere=0
        self.__cylinder=0
        self.__translationalCylinder=0
        self.__physics=0
        #subscribe robotnode and obtain robot pose and id
        self.__node.create_subscription(Twist, 'robotatt', self.__cmd_pose_callback, 10)
        #subscribe lidar sensor
        self.__node.create_subscription(LaserScan, 'lidar_sensor1', self.__lidar_sensor_callback, 10)
        #subscribe camera sensor
        self.__node.create_subscription(Image, 'camera_sensor1', self.__camera_sensor_callback, 10)
        #subscribemap from gamenode
        self.__node.create_subscription(MarkerArray, 'world', self.__createWorld_callback, 10)
        #initialize require fields for webots twin
        self.__myrobot_node=0
        self.__translational_field=0
        self.__rotational_field=0
        # publish  pov to users
        self.publish_pov = self.__node.create_publisher(Image, 'cameraData', 10)
        self.userView_timer = self.__node.create_timer(timer_period, self.pov_callback)
        # publish lidar data to user
        self.publish_lidar = self.__node.create_publisher(LaserScan, 'lidarData', 10)
        self.lidar_timer = self.__node.create_timer(timer_period, self.__distance_callback)
        
        
    def __cmd_pose_callback(self, pose):        
        self.__target_pose = pose
        
    def __createWorld_callback(self, obs):
        rclpy.spin_once(self.__node, timeout_sec=0.00)
        self.__obstacles=obs
        numofBox=0
        numofSphere=0
        numofCylinder=0
        for i in range(len(self.__obstacles.markers)):
            obs_type=self.__obstacles.markers[i].type
            obs_x= self.__obstacles.markers[i].pose.position.x
            obs_y= self.__obstacles.markers[i].pose.position.y
            try:
            #box
                if obs_type==1:
                    numofBox=numofBox+1
                    obj_def= 'BOX'+ str(numofBox)
                    self.__box = self.__robot.getFromDef(obj_def)
                    self.__translationalBox = self.__box.getField('translation')
                    #self.__physics = self.__box.getField('physics')
                    loc=[obs_x,obs_y,0.08]
                    self.__translationalBox.setSFVec3f(loc)
                    #self.__physics.importSFNodeFromString('Physics {}')
                #sphere    
                elif obs_type==2:
                    numofSphere=numofSphere+1
                    obj_def= 'SPHERE'+ str(numofSphere)
                    self.__sphere = self.__robot.getFromDef(obj_def)
                    self.__translationalSphere = self.__sphere.getField('translation')
                    #self.__physics = self.__sphere.getField('physics')
                    loc=[obs_x,obs_y,0.08]
                    self.__translationalSphere.setSFVec3f(loc)
                    #self.__physics.importSFNodeFromString('Physics {}')
                #cylinder    
                elif obs_type==3:
                    numofCylinder=numofCylinder+1
                    obj_def= 'CYLINDER'+ str(numofCylinder)
                    self.__cylinder = self.__robot.getFromDef(obj_def)
                    self.__translationalCylinder = self.__cylinder.getField('translation')
                    #self.__physics = self.__cylinder.getField('physics')
                    loc=[obs_x,obs_y,0.04]
                    self.__translationalCylinder.setSFVec3f(loc)
                    #self.__physics.importSFNodeFromString('Physics {}')
            except:
                print('banana')
                
        
    def __lidar_sensor_callback(self, lidar):
        self.__lidar_sensor_value = lidar

    def __camera_sensor_callback(self, message):
        self.__camera_sensor_value = message
        
    def pov_callback(self):
        #rclpy.spin_once(self.__node, timeout_sec=0.00)
        cam=Image()        
        cam.data=self.__camera_sensor_value.data   #number of robots
        # Publish the info and logger            
        self.publish_pov.publish(cam)
        #self.__node.get_logger().info('Publishing: "%s"' % cam) 
        
    def __distance_callback(self):
        #rclpy.spin_once(self.__node, timeout_sec=0.00)
        dist=LaserScan()        
        dist=self.__lidar_sensor_value   #number of robots
        # Publish the info and logger            
        self.publish_lidar.publish(dist)
        #self.__node.get_logger().info('Publishing: "%s"' % dist)
    
    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0.00)
        # Robots
        x = self.__target_pose.linear.x/250
        y = self.__target_pose.linear.y/250
        z = self.__target_pose.linear.z
        rot= self.__target_pose.angular.z*(3.14/180)
        rob_id=self.__target_pose.angular.x
        
        # duplicate robots
        if rob_id==1:
            self.__myrobot_node = self.__robot.getFromDef('ROB1')
            self.__translation_field = self.__myrobot_node.getField('translation')
            self.__rotational_field = self.__myrobot_node.getField('rotation')
            inst_loc=[x,y,z]
            inst_rot=[0, 0, 1, rot]
            self.__translation_field.setSFVec3f(inst_loc)
            self.__rotational_field.setSFRotation(inst_rot)
        elif rob_id==2:
            self.__myrobot_node = self.__robot.getFromDef('ROB2')
            self.__translation_field = self.__myrobot_node.getField('translation')
            self.__rotational_field = self.__myrobot_node.getField('rotation')
            inst_loc=[x,y,z]
            inst_rot=[0, 0, 1, rot]
            self.__translation_field.setSFVec3f(inst_loc)
            self.__rotational_field.setSFRotation(inst_rot)
        elif rob_id==3:
            self.__myrobot_node = self.__robot.getFromDef('ROB3')
            self.__translation_field = self.__myrobot_node.getField('translation')
            self.__rotational_field = self.__myrobot_node.getField('rotation')
            inst_loc=[x,y,z]
            inst_rot=[0, 0, 1, rot]
            self.__translation_field.setSFVec3f(inst_loc)
            self.__rotational_field.setSFRotation(inst_rot)
        elif rob_id==4:
            self.__myrobot_node = self.__robot.getFromDef('ROB4')
            self.__translation_field = self.__myrobot_node.getField('translation')
            self.__rotational_field = self.__myrobot_node.getField('rotation')
            inst_loc=[x,y,z]
            inst_rot=[0, 0, 1, rot]
            self.__translation_field.setSFVec3f(inst_loc)
            self.__rotational_field.setSFRotation(inst_rot)
        elif rob_id==5:
            self.__myrobot_node = self.__robot.getFromDef('ROB5')
            self.__translation_field = self.__myrobot_node.getField('translation')
            self.__rotational_field = self.__myrobot_node.getField('rotation')
            inst_loc=[x,y,z]
            inst_rot=[0, 0, 1, rot]
            self.__translation_field.setSFVec3f(inst_loc)
            self.__rotational_field.setSFRotation(inst_rot)
        elif rob_id==6:
            self.__myrobot_node = self.__robot.getFromDef('ROB6')
            self.__translation_field = self.__myrobot_node.getField('translation')
            self.__rotational_field = self.__myrobot_node.getField('rotation')
            inst_loc=[x,y,z]
            inst_rot=[0, 0, 1, rot]
            self.__translation_field.setSFVec3f(inst_loc)
            self.__rotational_field.setSFRotation(inst_rot)
        elif rob_id==7:
            self.__myrobot_node = self.__robot.getFromDef('ROB7')
            self.__translation_field = self.__myrobot_node.getField('translation')
            self.__rotational_field = self.__myrobot_node.getField('rotation')
            inst_loc=[x,y,z]
            inst_rot=[0, 0, 1, rot]
            self.__translation_field.setSFVec3f(inst_loc)
            self.__rotational_field.setSFRotation(inst_rot)
        elif rob_id==8:
            self.__myrobot_node = self.__robot.getFromDef('ROB8')
            self.__translation_field = self.__myrobot_node.getField('translation')
            self.__rotational_field = self.__myrobot_node.getField('rotation')
            inst_loc=[x,y,z]
            inst_rot=[0, 0, 1, rot]
            self.__translation_field.setSFVec3f(inst_loc)
            self.__rotational_field.setSFRotation(inst_rot)


def main():
    print('Hi from minibots.')
    


if __name__ == '__main__':
    main()
