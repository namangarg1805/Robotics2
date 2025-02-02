#!/usr/bin/env python3
#team-id-SS_1675
import numpy as np
import cv2 as cv
import cv2.aruco as aruco
from cv_bridge import CvBridge, CvBridgeError
import rospy
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from std_msgs.msg import *
from sensor_msgs.msg import *
from gazebo_ros_link_attacher.srv import Gripper

class offboard_control:
    
    def __init__(self):
        #initializing the RosNode
        rospy.init_node('pick_n_place_aruco', anonymous=True)
   
    def setArm(self,action=True):
        #Arming The Drone
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
            armService(action)
        except rospy.ServiceException as e:
            print ("Service arming call failed: %s"%e)

    def setGripper(self,action=False):
        #Box Gripper Service
        rospy.wait_for_service('/activate_gripper')
        try:
            gripperService = rospy.ServiceProxy('/activate_gripper', Gripper)
            gripperService(action)
        except rospy.ServiceException as e:
            print ("Service gripper call failed: %s"%e)

    def set_mode(self,mode):
        #Drone Mode Changer
        rospy.wait_for_service('mavros/set_mode')
        try:
            setMode = rospy.ServiceProxy('mavros/set_mode', SetMode)
            setMode(0,mode)#0 for custom mode
        except rospy.ServiceException as e:
            print("Service setmode call fail:%s"%e )
    
    def set_param(self,param_id,param_value):
        #For Keeping Drone in Offboard Mode
        rospy.wait_for_service('mavros/param/set')
        try:
            set_param_srv = rospy.ServiceProxy('mavros/param/set', ParamSet)
            set_param_srv(param_id, param_value)
        except rospy.ServiceException as e:
            print ("Service param call failed: %s"%e)

   
   
class stateMoniter:
    
    def __init__(self):
        self.state  = State()
        self.pos    = PoseStamped()
        self.data   = String()
        self.img    = np.empty([])
        self.bridge = CvBridge()
        
    def stateCb(self, msg):
        self.state = msg
    
    def local_position_callback(self, msg):
        self.pos = msg  
        
    def gripper_callback(self,msg):
        self.data=msg
    
    def image_callback(self,image_data):
        self.img = self.bridge.imgmsg_to_cv2(image_data, "bgr8")#converting image to OpenCv Format
        gray = cv.cvtColor(self.img, cv.COLOR_BGR2GRAY)#converting to Gray Color
        self.img=gray
        
    def is_at_position(self, x, y, z):
        #Check for the correct position of drone
        #Return True if the Drone is at given coordinates
        offset=1
        """offset: meters"""
        rospy.logdebug(
            "current position | x:{0:.2f}, y:{1:.2f}, z:{2:.2f}".format(
                self.pos.pose.position.x, self.pos.pose.
                position.y, self.pos.pose.position.z))

        desired = np.array((x, y, z))
        pos = np.array((self.pos.pose.position.x,
                        self.pos.pose.position.y,
                        self.pos.pose.position.z))
        return np.linalg.norm(desired - pos) < offset
    
    def detect_ArUco(self,img):
        #Detect the ArUco marker and Identifies its ArUco ID
        Detected_ArUco_markers = {}
        dictionary = cv.aruco.Dictionary_get(cv.aruco.DICT_5X5_250)
        parameters = aruco.DetectorParameters_create()
        corners, ids, rejected= aruco.detectMarkers(img, dictionary, parameters = parameters)
        if ids:
            if len(ids)>0:
                for i in range(len(ids)):
                    Detected_ArUco_markers[ids[i][0]]=corners[i][0]
        return Detected_ArUco_markers
    
    def Calculate_Center_Coordinates(self,Detected_ArUco_markers):
        #Calculate the Matrix Center Pixel Coordinates of ArUco Marker with respect to AruCo marker Dimensions
        for i,c in Detected_ArUco_markers.items():
            topLeft, topRight, bottomRight, bottomLeft = c
            center=(topLeft+bottomRight)/2
        return center


def main():
    ofb_ctl = offboard_control()
    stateMt = stateMoniter()
    #Initialising Publishers
    local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
    local_vel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
    rate = rospy.Rate(20.0)#Operating Frequency of RosCore

    #Setpoints List
    setpoints = [(0,0,3),(9,0,3),(9,0,3),(9,0,-1.6),(9,0,3),(0,0,3),(0,0,0)]
    
    our=PoseStamped()
    our.pose.position.x = 0.0
    our.pose.position.y = 0.0
    our.pose.position.z = 0.0
    
    vel = TwistStamped()
    vel.twist.linear.x = 5
    vel.twist.linear.y = 5
    vel.twist.linear.z = 5
    
    #Initialising Subscribers
    rospy.Subscriber("/mavros/state", State, stateMt.stateCb )
    rospy.Subscriber("/mavros/local_position/pose" , PoseStamped, stateMt.local_position_callback )
    rospy.Subscriber("/mavros/gripper_check", String, stateMt.gripper_callback)
    rospy.Subscriber("/iris/camera/image_raw", Image, stateMt.image_callback)
    
    #Sending sample Coordinates For OFFBOARD Mode 
    for i in range(100):
        local_pos_pub.publish(our)
        rate.sleep()
    
    rcl_except = mavros_msgs.msg.ParamValue(1<<2,0.0)
    ofb_ctl.set_param("COM_RCL_EXCEPT",rcl_except)
    
    ofb_ctl.set_mode("OFFBOARD")
    print("OFFBOARD mode Activated")
    
    #Arming The Drone by calling Function setArm
    for i in range(30):
        ofb_ctl.setArm()
        rate.sleep()

    print("Drone Armed")
    
    #Complete Algorithm
    i=0
    k=False
    while not rospy.is_shutdown() and i!=8:
        our.pose.position.x=setpoints[i][0]
        our.pose.position.y=setpoints[i][1]
        our.pose.position.z=setpoints[i][2]
        #Providing Setpoints
        for check1 in range(0,20):
            local_pos_pub.publish(our)
           # local_vel_pub.publish(vel)
            rate.sleep()
        if i==1:
            #For Detecting Box With ArUco Marker on the way
            waycheck=False
            add_diff_x=0
            while waycheck!=True:
                p=stateMt.detect_ArUco(stateMt.img)#Detecting ArUco Marker if Any and Its ID
                if p:
                    #Giving the Coordinates at which the Drone was, when the Box was Detected as it  may get ahead of it while moving
                    our.pose.position.x=stateMt.pos.pose.position.x+add_diff_x
                    our.pose.position.y=stateMt.pos.pose.position.y
                    our.pose.position.z =stateMt.pos.pose.position.z
                    for check1 in range(0,20):
                        local_pos_pub.publish(our)
                        #local_vel_pub.publish(vel)
                        rate.sleep()
                    #detecting again for exact location
                    p=stateMt.detect_ArUco(stateMt.img)
                    if p:
                        q=stateMt.Calculate_Center_Coordinates(p)
                        #Trying to match Center Of ROS Camera Frame with Box being at center
                        if q[0]<210 and q[0]>190:
                            print("Approaching Box")
                            waycheck=True
                        else:
                            #Moving the Drone According to the If condition 
                            diffcol=q[0]-200 #200 because RoSCamera capturing image size is 400*400 so center is 200,200  
                            add_diff_x=diffcol/100
            #Adding Final Box Location 
            setpoints.insert(2,(our.pose.position.x,our.pose.position.y,-1.7)) 
            
        #checking if the drone has reached correct position or not
        for check1 in range(0,20):
            k=stateMt.is_at_position(our.pose.position.x,our.pose.position.y,our.pose.position.z)
            rate.sleep()
            
        if k==True and i==4:
            #Releasing The Box
            print("Gripper Deactivate")
            for xl in range(20):
                ofb_ctl.setGripper(False)
                rate.sleep()
        
        if k==True and i==2:
            #Picking the Box
            print("Gripper Activated")
            for xl in range(30):
                ofb_ctl.setGripper(True)
                rate.sleep()
                
        if k==True:
            if i!=1:
                setpoint_reached=(setpoints[i][0],setpoints[i][1],setpoints[i][2])
                print(f"setpoint {setpoint_reached} reached")
            i+=1
        else:
            continue
        rate.sleep()
    #Disarming the Drone
    ofb_ctl.setArm(False)
    print("Drone Disarmed")
    print("Mission Completed")
            
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass