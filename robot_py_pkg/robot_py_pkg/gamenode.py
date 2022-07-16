import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import yaml
from yaml.loader import SafeLoader
import math
#import required msgs
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist, Vector3
from diagnostic_msgs.msg import KeyValue
from sensor_msgs.msg import Image, LaserScan 
from visualization_msgs.msg import Marker, MarkerArray


#Disclaimer ROBOT ID has been used in several type of msg

class GameNode(Node):
    
    def __init__(self):
        #initialize super class
        super().__init__('gamemaster')
        timer_period = 0.001  # publishing period(sec)
        #Game Variables
        self.__playerList=[]
        self.__zoneTypes=[]
        # Global Variables
        self.__robot= Twist()
        self.__wheelvel=Vector3()
        self.__grid=MarkerArray()
        self.__snapshot=Image()
        #Subscribe Map
        self.create_subscription(Image,'/image',self.cutMap,10)
        #subscribe robotnode and obtain robot pose and id
        self.create_subscription(Twist, 'robotatt', self.__pose_callback, 10)
        # subscribe user
        self.create_subscription(Vector3, 'wheel_vel', self.__speedCalc_callback, 10)
        # subscribe terrain colors
        self.create_subscription(MarkerArray, 'grid', self.__grid_callback, 10)
        #subscribe charge info        
        self.create_subscription(KeyValue, 'charge', self.__charge_callback, 10)
        #Publish Number of Robots
        self.publish_robotnum = self.create_publisher(Int16, 'rob_num', 10)
        self.robotnum_timer = self.create_timer(timer_period, self.robotNumber_callback)
        #Publish Run Info
        self.publish_runinfo = self.create_publisher(Vector3, 'runInfo', 10)
        self.runInfo_timer = self.create_timer(timer_period, self.runInfo_callback)
        #Publish Run Info
        self.publish_lidarProp = self.create_publisher(LaserScan, 'lidarProp', 10)
        self.lidarProp_timer = self.create_timer(timer_period, self.lidarProp_callback)
        #Publish Player Type
        self.publish_playerType = self.create_publisher(KeyValue, 'playerType', 10)
        self.playerType_timer = self.create_timer(timer_period, self.playerType_callback)
        #Publish Player Type
        self.publish_worldProp = self.create_publisher(MarkerArray, 'world', 10)
        self.worldProp_timer = self.create_timer(timer_period, self.worldProp_callback)
        #Publish Cut Map
        self.publish_userView = self.create_publisher(Image, 'view', 10)
        self.userView_timer = self.create_timer(timer_period, self.userView_callback)
        #Internal Functions
        self.robotZone_timer = self.create_timer(timer_period, self.getRobotZone)
            
    def cutMap(self, msg):
        img = np.array(msg.data).reshape((240, 320,3))
        # transpose to image data
        self.__snapshot.data=img.reshape(-1).tolist()        
    # get robot pose
    def __pose_callback(self, pose):        
        self.__robot=pose
        self.__robot.linear.x=self.__robot.linear.x/250
        self.__robot.linear.y=self.__robot.linear.y/250
    # get wheel speed from user
    def __speedCalc_callback(self, vel):        
        self.__wheelvel=vel
        return self.__wheelvel
    # get grid fields
    def __grid_callback(self,grid):
        self.__grid = grid        
    # get charge info
    def __charge_callback(self, charge):
        rob_id=charge.key
        state=charge.value
        return rob_id, state
    # set number of robots
    def robotNumber_callback(self):
        number_msg=Int16()        
        number_msg.data   #number of robots
        # Publish the info and logger            
        self.publish_robotnum.publish(number_msg)
        #self.get_logger().info('Publishing: "%s"' % number_msg)
    # set wheel velocity    
    def runInfo_callback(self):
        run_msg=Vector3()        
        run_msg.x   #Left Wheel Velocity
        run_msg.y   #Right Wheel Velocity
        run_msg.z   #Robot ID
        # Publish the info and logger            
        self.publish_runinfo.publish(run_msg)
        #self.get_logger().info('Publishing: "%s"' % run_msg)
    # set player type    
    def playerType_callback(self):
        player_msg=KeyValue()        
        player_msg.key      #PLAYER ID AS STRING
        player_msg.value    #PLAYER TYPE
        # Publish the info and logger            
        self.publish_playerType.publish(player_msg)
        #self.get_logger().info('Publishing: "%s"' % player_msg)
        
    # broadcast view of robots
    def userView_callback(self):  
        self.publish_userView.publish(self.__snapshot)
        #self.get_logger().info('Publishing: "%s"' % self.__snapshot)
        
                
    def lidarProp_callback(self):
        lidar_msg = LaserScan()
        # Robot ID
        lidar_msg.header.frame_id
        # Properties of Lidar on the Robot
        lidar_msg.angle_min
        lidar_msg.angle_max
        lidar_msg.angle_increment
        # Publish the info and logger            
        self.publish_lidarProp.publish(lidar_msg)
        #self.get_logger().info('Publishing: "%s"' % lidar_msg)
    
    def worldProp_callback(self):
        obstacle_msg=Marker()
        world_msg=MarkerArray()
        obstacle_msg.id             #Obtacle ID
        obstacle_msg.type           #type of the object BOX=1, SPHERE=2, CYLINDER=3
        obstacle_msg.action
        obstacle_msg.pose.position.x
        obstacle_msg.pose.position.y 
        obstacle_msg.pose.position.z   #obstacle orientation in rad
        obstacle_msg.color.r
        obstacle_msg.color.g
        obstacle_msg.color.b
        world_msg.markers.append(obstacle_msg)
        self.publish_worldProp.publish(world_msg)
        #self.get_logger().info('Publishing: "%s"' % world_msg)
        
    def parse_config(self):
        #if the game designer needs a whole new class of config variables this function should also be changed
        with open("config_version2.yaml") as f:
            data=yaml.load(f,SafeLoader)    
        self.__playerList=data["players"]
        self.__zoneTypes=data["zones"]
        
    #returns closest zone and rob_id
    def getRobotZone(self):
        #robot pose
        x=self.__robot.linear.x
        y=self.__robot.linear.y
        print(x,y)
        rob_id=self.__robot.angular.x
        zone_centers=[]
        for i in range(len(self.__grid.markers)):
            c_x=self.__grid.markers[i].pose.position.x
            c_y=self.__grid.markers[i].pose.position.y 
            center=[c_x,c_y]
            zone_centers.append(center)
        dist_to_nodes=[]
        for j in zone_centers:
            d=math.sqrt((j[0]-x)**2+(j[1]-y)**2)       
            dist_to_nodes.append(d)
        try:    
            dist=min(dist_to_nodes)
            zone_index=dist_to_nodes.index(dist)
            zone=zone_centers[zone_index]
            print(rob_id,dist,zone)
        except:
            dist=-1
            zone=[-1,-1]
        return [rob_id,zone]
                  

def main(args=None):
    rclpy.init(args=args)
    node = GameNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
