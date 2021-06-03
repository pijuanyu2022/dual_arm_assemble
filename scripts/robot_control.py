#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Range, JointState
import tf2_ros
from tf2_msgs.msg import TFMessage
import geometry_msgs.msg
import time
import numpy as np
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
import math


class RobotControl(object):
    def __init__(self, robot_type='UR5'):
        self.robot_type = robot_type
        if self.robot_type == 'UR5':
            rospy.init_node('UR5_control', anonymous=True)
            
        print('INIT NODE')
        
        self.initialize_publishers()
        self.initialize_subscribers()
        self.sim_time = 0.0
        
        self.robot_pose = Pose()
        self.Right_RG2_front1_sensor_val = Float32()
        self.Right_RG2_front2_sensor_val = Float32()
        self.Right_RG2_touch_sensor_val = Float32()
        self.Right_Conveyor_belt_sensor_val = Float32()

        self.Left_RG2_front1_sensor_val = Float32()
        self.Left_RG2_front2_sensor_val = Float32()
        self.Left_RG2_touch_sensor_val = Float32()
        self.Left_Conveyor_belt_sensor_val = Float32()

        self.joint_states = JointState()
        self.SENSOR_TYPES = ['Right_RG2_front1', 'Right_RG2_front2','Right_RG2_touch', 'Right_Conveyor_belt', \
            'Left_RG2_front1', 'Left_RG2_front2','Left_RG2_touch', 'Left_Conveyor_belt']
        # self.is_go_down = False

        self.initialize_publisher_msgs()

        
        #if self.robot_type == 'hexapod':
        # self.paradigm_reconfigure_srv = DynamicReconfigureServer(HexapodJointControlConfig, self.reconfig_paradigm_cb)
        #else:
        #    self.paradigm_reconfigure_srv = DynamicReconfigureServer(RWJointControlConfig, self.reconfig_paradigm_cb)

    def degToRad(self, deg):
        return deg*math.pi/180.0
        
    def initialize_publisher_msgs(self):
        if self.robot_type == 'UR5':
            self.Right_UR5_j1 = Float32()
            self.Right_UR5_j2 = Float32()
            self.Right_UR5_j3 = Float32()

            self.Right_UR5_j4 = Float32()
            self.Right_UR5_j5 = Float32()
            self.Right_UR5_j6 = Float32()
            self.Right_UR5_j7 = Float32()

            self.Left_UR5_j1 = Float32()
            self.Left_UR5_j2 = Float32()
            self.Left_UR5_j3 = Float32()

            self.Left_UR5_j4 = Float32()
            self.Left_UR5_j5 = Float32()
            self.Left_UR5_j6 = Float32()
            self.Left_UR5_j7 = Float32()

    def initialize_publishers(self):
        # initialized latched publishers
        if self.robot_type == 'UR5':
            self.Right_UR5_j1_pub = rospy.Publisher('/Right_UR5_j1', Float32, queue_size=1, latch=True)            
            self.Right_UR5_j2_pub = rospy.Publisher('/Right_UR5_j2', Float32, queue_size=1, latch=True)
            self.Right_UR5_j3_pub = rospy.Publisher('/Right_UR5_j3', Float32, queue_size=1, latch=True)

            self.Right_UR5_j4_pub = rospy.Publisher('/Right_UR5_j4', Float32, queue_size=1, latch=True)
            self.Right_UR5_j5_pub = rospy.Publisher('/Right_UR5_j5', Float32, queue_size=1, latch=True)
            self.Right_UR5_j6_pub = rospy.Publisher('/Right_UR5_j6', Float32, queue_size=1, latch=True)
            self.Right_UR5_j7_pub = rospy.Publisher('/Right_UR5_j7', Float32, queue_size=1, latch=True)

            self.Left_UR5_j1_pub = rospy.Publisher('/Left_UR5_j1', Float32, queue_size=1, latch=True)            
            self.Left_UR5_j2_pub = rospy.Publisher('/Left_UR5_j2', Float32, queue_size=1, latch=True)
            self.Left_UR5_j3_pub = rospy.Publisher('/Left_UR5_j3', Float32, queue_size=1, latch=True)

            self.Left_UR5_j4_pub = rospy.Publisher('/Left_UR5_j4', Float32, queue_size=1, latch=True)
            self.Left_UR5_j5_pub = rospy.Publisher('/Left_UR5_j5', Float32, queue_size=1, latch=True)
            self.Left_UR5_j6_pub = rospy.Publisher('/Left_UR5_j6', Float32, queue_size=1, latch=True)
            self.Left_UR5_j7_pub = rospy.Publisher('/Left_UR5_j7', Float32, queue_size=1, latch=True)

    
    def initialize_subscribers(self):
        rospy.Subscriber('/simTime', Float32, self.simTime_cb)
        rospy.Subscriber('/tf', TFMessage, self.tf_cb)

        rospy.Subscriber('/Right_RG2_front1SensorDistance', Range, self.distance_Right_RG2_front1_cb)
        rospy.Subscriber('/Right_RG2_front2SensorDistance', Range, self.distance_Right_RG2_front2_cb)
        rospy.Subscriber('/Right_RG2_touchSensorDistance',  Range, self.distance_Right_RG2_touch_cb)
        rospy.Subscriber('/Right_Conveyor_beltSensorDistance', Range, self.distance_Right_Conveyor_belt_cb)

        rospy.Subscriber('/Left_RG2_front1SensorDistance', Range, self.distance_Left_RG2_front1_cb)
        rospy.Subscriber('/Left_RG2_front2SensorDistance', Range, self.distance_Left_RG2_front2_cb)
        rospy.Subscriber('/Left_RG2_touchSensorDistance',  Range, self.distance_Left_RG2_touch_cb)
        rospy.Subscriber('/Left_Conveyor_beltSensorDistance', Range, self.distance_Left_Conveyor_belt_cb)

        if self.robot_type == 'UR5':
            rospy.Subscriber('/UR5/joint_states', JointState, self.joint_states_cb)
    
    # subscriber callbacks
    def simTime_cb(self, msg):
        self.sim_time = msg.data
    
    def tf_cb(self, msg):
        self.robot_pose.position.x = msg.transforms[0].transform.translation.x
        self.robot_pose.position.y = msg.transforms[0].transform.translation.y
        self.robot_pose.position.z = msg.transforms[0].transform.translation.z
        self.robot_pose.orientation.x = msg.transforms[0].transform.rotation.x
        self.robot_pose.orientation.y = msg.transforms[0].transform.rotation.y
        self.robot_pose.orientation.z = msg.transforms[0].transform.rotation.z
        self.robot_pose.orientation.w = msg.transforms[0].transform.rotation.w

    #Sensor callbacks Same for both robots. 
    def distance_Right_RG2_front1_cb(self, msg):
        self.Right_RG2_front1_sensor_val.data = msg.range

    def distance_Right_RG2_front2_cb(self, msg):
        self.Right_RG2_front2_sensor_val.data = msg.range
    
    def distance_Right_RG2_touch_cb(self, msg):
        self.Right_RG2_touch_sensor_val.data = msg.range    

    def distance_Right_Conveyor_belt_cb(self, msg):
        self.Right_Conveyor_belt_sensor_val.data = msg.range
        

    def distance_Left_RG2_front1_cb(self, msg):
        self.Left_RG2_front1_sensor_val.data = msg.range

    def distance_Left_RG2_front2_cb(self, msg):
        self.Left_RG2_front2_sensor_val.data = msg.range
    
    def distance_Left_RG2_touch_cb(self, msg):
        self.Left_RG2_touch_sensor_val.data = msg.range    

    def distance_Left_Conveyor_belt_cb(self, msg):
        self.Left_Conveyor_belt_sensor_val.data = msg.range

    def joint_states_cb(self, msg):
        self.joint_states = msg

    def publish_joint_values(self): 
        if self.robot_type == 'UR5':
            self.Right_UR5_j1_pub.publish(self.Right_UR5_j1)
            self.Right_UR5_j2_pub.publish(self.Right_UR5_j2)          
            self.Right_UR5_j3_pub.publish(self.Right_UR5_j3)

            self.Right_UR5_j4_pub.publish(self.Right_UR5_j4)
            self.Right_UR5_j5_pub.publish(self.Right_UR5_j5)
            self.Right_UR5_j6_pub.publish(self.Right_UR5_j6)
            self.Right_UR5_j7_pub.publish(self.Right_UR5_j7)

            self.Left_UR5_j1_pub.publish(self.Left_UR5_j1)
            self.Left_UR5_j2_pub.publish(self.Left_UR5_j2)          
            self.Left_UR5_j3_pub.publish(self.Left_UR5_j3)

            self.Left_UR5_j4_pub.publish(self.Left_UR5_j4)
            self.Left_UR5_j5_pub.publish(self.Left_UR5_j5)
            self.Left_UR5_j6_pub.publish(self.Left_UR5_j6)
            self.Left_UR5_j7_pub.publish(self.Left_UR5_j7)


    
    def reconfig_paradigm_cb(self, config, level):        
        if self.robot_type == 'UR5':
            self.Right_UR5_j1.data = config.Right_UR5_j1
            self.Right_UR5_j2.data = config.Right_UR5_j2
            self.Right_UR5_j3.data = config.Right_UR5_j3

            self.Right_UR5_j4.data = config.Right_UR5_j4
            self.Right_UR5_j5.data = config.Right_UR5_j5
            self.Right_UR5_j6.data = config.Right_UR5_j6
            self.Right_UR5_j7.data = config.Right_UR5_j7

            self.Left_UR5_j1.data = config.Left_UR5_j1
            self.Left_UR5_j2.data = config.Left_UR5_j2
            self.Left_UR5_j3.data = config.Left_UR5_j3

            self.Left_UR5_j4.data = config.Left_UR5_j4
            self.Left_UR5_j5.data = config.Left_UR5_j5
            self.Left_UR5_j6.data = config.Left_UR5_j6
            self.Left_UR5_j7.data = config.Left_UR5_j7


        self.publish_joint_values()

        return config
    
    #getters 
    def getSensorValue(self, sensor_type='Right_RG2_front1'):
        assert sensor_type in self.SENSOR_TYPES
        if sensor_type=='Right_RG2_front1':
            return self.Right_RG2_front1_sensor_val.data
        elif sensor_type=='Right_RG2_front2':
            return self.Right_RG2_front2_sensor_val.data
        elif sensor_type == 'Right_RG2_touch':
            return self.Right_RG2_touch_sensor_val.data
        elif sensor_type == 'Right_Conveyor_belt':
            return self.Right_Conveyor_belt_sensor_val.data
        elif sensor_type=='Left_RG2_front1':
            return self.Left_RG2_front1_sensor_val.data
        elif sensor_type=='Left_RG2_front2':
            return self.Left_RG2_front2_sensor_val.data
        elif sensor_type == 'Left_RG2_touch':
            return self.Left_RG2_touch_sensor_val.data
        elif sensor_type == 'Left_Conveyor_belt':
            return self.Left_Conveyor_belt_sensor_val.data
    
    def getMotorCurrentJointPosition(self, motor_id_string='Right_UR5_j1'):
        assert motor_id_string in self.joint_states.name
        motor_id = self.joint_states.name.index(motor_id_string)
        return self.joint_states.position[motor_id]
    
    def getRobotWorldLocation(self):
        return self.robot_pose.position, self.robot_pose.orientation
    
    def getCurrentSimTime(self):
        return self.sim_time
    
    #setters
    
    def setMotorTargetJointPosition(self, motor_id_string='Right_UR5_j1', target_joint_angle=0.0):
        if motor_id_string == 'Right_UR5_j1':
            self.Right_UR5_j1.data = target_joint_angle
        elif motor_id_string == 'Right_UR5_j2':
            self.Right_UR5_j2.data = target_joint_angle
        elif motor_id_string == 'Right_UR5_j3':
            self.Right_UR5_j3.data = target_joint_angle
        elif motor_id_string == 'Right_UR5_j4':
            self.Right_UR5_j4.data = target_joint_angle
        elif motor_id_string == 'Right_UR5_j5':
            self.Right_UR5_j5.data = target_joint_angle
        elif motor_id_string == 'Right_UR5_j6':
            self.Right_UR5_j6.data = target_joint_angle
        elif motor_id_string == 'Right_UR5_j7': 
            self.Right_UR5_j7.data = target_joint_angle
        elif motor_id_string == 'Left_UR5_j1':
            self.Left_UR5_j1.data = target_joint_angle
        elif motor_id_string == 'Left_UR5_j2':
            self.Left_UR5_j2.data = target_joint_angle
        elif motor_id_string == 'Left_UR5_j3':
            self.Left_UR5_j3.data = target_joint_angle
        elif motor_id_string == 'Left_UR5_j4':
            self.Left_UR5_j4.data = target_joint_angle
        elif motor_id_string == 'Left_UR5_j5':
            self.Left_UR5_j5.data = target_joint_angle
        elif motor_id_string == 'Left_UR5_j6':
            self.Left_UR5_j6.data = target_joint_angle
        elif motor_id_string == 'Left_UR5_j7': 
            self.Left_UR5_j7.data = target_joint_angle
        
        self.publish_joint_values()
    