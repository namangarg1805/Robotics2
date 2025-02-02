#!/usr/bin/env python3
#team-id-SS_1675
import threading
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
        rospy.init_node('strawberry_stacker', anonymous=True)
   
    def setArmD1(self,action=True):
        #Arming The Drone
        rospy.wait_for_service('/edrone0/mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('/edrone0/mavros/cmd/arming', CommandBool)
            armService(action)
        except rospy.ServiceException as e:
            print ("Service arming call failed: %s"%e)
            
    def setArmD2(self,action=True):
        #Arming The Drone
        rospy.wait_for_service('/edrone1/mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('/edrone1/mavros/cmd/arming', CommandBool)
            armService(action)
        except rospy.ServiceException as e:
            print ("Service arming call failed: %s"%e)

    def setGripperD1(self,action=False):
        #Box Gripper Service
        rospy.wait_for_service('/edrone0/activate_gripper')
        try:
            gripperService = rospy.ServiceProxy('/edrone0/activate_gripper', Gripper)
            gripperService(action)
        except rospy.ServiceException as e:
            print ("Service gripper call failed: %s"%e)
            
    def setGripperD2(self,action=False):
        #Box Gripper Service
        rospy.wait_for_service('/edrone1/activate_gripper')
        try:
            gripperService = rospy.ServiceProxy('/edrone1/activate_gripper', Gripper)
            gripperService(action)
        except rospy.ServiceException as e:
            print ("Service gripper call failed: %s"%e)

    def set_modeD1(self,mode):
        #Drone Mode Changer
        rospy.wait_for_service('/edrone0/mavros/set_mode')
        try:
            setMode = rospy.ServiceProxy('/edrone0/mavros/set_mode', SetMode)
            setMode(0,mode)#0 for custom mode
        except rospy.ServiceException as e:
            print("Service setmode call fail:%s"%e )
            
    def set_modeD2(self,mode):
        #Drone Mode Changer
        rospy.wait_for_service('/edrone1/mavros/set_mode')
        try:
            setMode = rospy.ServiceProxy('/edrone1/mavros/set_mode', SetMode)
            setMode(0,mode)#0 for custom mode
        except rospy.ServiceException as e:
            print("Service setmode call fail:%s"%e )    
            
    def set_paramD1(self,param_id,param_value):
        #For Keeping Drone in Offboard Mode
        rospy.wait_for_service('/edrone0/mavros/param/set')
        try:
            set_param_srv = rospy.ServiceProxy('/edrone0/mavros/param/set', ParamSet)
            set_param_srv(param_id, param_value)
        except rospy.ServiceException as e:
            print ("Service param call failed: %s"%e)
            
    def set_paramD2(self,param_id,param_value):
        #For Keeping Drone in Offboard Mode
        rospy.wait_for_service('/edrone1/mavros/param/set')
        try:
            set_param_srv = rospy.ServiceProxy('/edrone1/mavros/param/set', ParamSet)
            set_param_srv(param_id, param_value)
        except rospy.ServiceException as e:
            print ("Service param call failed: %s"%e)
            
#Global Variables
alter=0
d1prev=-1
d2prev=-2
rowsD1=[]
rowsD2=[]
previous_xD1=0
previous_yD1=0
previous_xD2=0
previous_yD2=0
setpointsD1=[]
setpointsD2=[]
assigned_blue=[]
assigned_red=[]

class stateMoniter:
    
    def __init__(self):
        self.stateD1     = State()
        self.stateD2     = State()
        self.gripcheckD1 = String()
        self.gripcheckD2 = String()
        self.spawn_row   = UInt8()
        
        self.posD1       = PoseStamped()
        self.posD2       = PoseStamped()
        
        self.imgD1       = np.empty([])
        self.imgD2       = np.empty([])
        self.bridge      = CvBridge()
    
      
    def calculate_D1_setpoints(self,rowno):
        #calculating setpoints of row
        row_y_cord=rowno*4-4
        rowcoordinate=(-3,row_y_cord,5)
        return rowcoordinate 
 
    def calculate_D2_setpoints(self,rowno):
        #calculating setpoints of row
        if rowno==15:
            row_y_cord=4
            rowcoordinate=(0,-row_y_cord,3)
            return rowcoordinate
        else:
            row_y_cord=(15-rowno)*4+4
            rowcoordinate=(-3,-row_y_cord,3)
            return rowcoordinate 
        
    #callback functions
    def spawn_callback(self,data):
        #Taking spawn information and alternatively assigning rowno to each drone
        self.spawn_row=data
        copy_spawn=self.spawn_row.data
        global rowsD1
        global rowsD1
        global setpointsD1
        global setpointsD2
        global alter
        global d2prev
        global d1prev
            
        if alter%2==0:
            if copy_spawn==d2prev and alter>0:
                d2prev=copy_spawn
                rowsD2.append(copy_spawn)
                cord=self.calculate_D2_setpoints(copy_spawn)
                setpointsD2.append(cord)
            else:
                d1prev=copy_spawn
                rowsD1.append(copy_spawn)
                cord=self.calculate_D1_setpoints(copy_spawn)
                setpointsD1.append(cord)
        else:
            if copy_spawn==d1prev:
                d1prev=copy_spawn
                rowsD1.append(copy_spawn)
                cord=self.calculate_D1_setpoints(copy_spawn)
                setpointsD1.append(cord)
            else:
                d2prev=copy_spawn
                rowsD2.append(copy_spawn)
                cord=self.calculate_D2_setpoints(copy_spawn)
                setpointsD2.append(cord)
        alter+=1
        
    def state_callbackD1(self, data):
        self.stateD1 = data
    
    def state_callbackD2(self, data):
        self.stateD2 = data
    
    def local_position_callbackD1(self, data):
        self.posD1 = data
    
    def local_position_callbackD2(self, data):
        self.posD2 = data
        
    def gripper_callbackD1(self, data):
        self.gripcheckD1 = data
    
    def gripper_callbackD2(self, data):
        self.gripcheckD2 = data
    
    def image_callbackD1(self, image_data):
        self.imgD1 = self.bridge.imgmsg_to_cv2(image_data, "bgr8")#converting image to OpenCv Format
        gray       = cv.cvtColor(self.imgD1, cv.COLOR_BGR2GRAY)   #converting to Gray Color
        self.imgD1 = gray
    
    def image_callbackD2(self, image_data):
        self.imgD2 = self.bridge.imgmsg_to_cv2(image_data, "bgr8")#converting image to OpenCv Format
        gray       = cv.cvtColor(self.imgD2, cv.COLOR_BGR2GRAY)   #converting to Gray Color
        self.imgD2 = gray
        
    def is_at_positionD1(self, x, y, z):
        #Check for the correct position of drone
        #Return True if the Drone is at given coordinates
        offset=1
        """offset: meters"""
        rospy.logdebug(
            "current position | x:{0:.2f}, y:{1:.2f}, z:{2:.2f}".format(
                self.posD1.pose.position.x, self.posD1.pose.
                position.y, self.posD1.pose.position.z))

        desired = np.array((x, y, z))
        posi = np.array((self.posD1.pose.position.x,
                        self.posD1.pose.position.y,
                        self.posD1.pose.position.z))
        return np.linalg.norm(desired - posi) < offset
    
    def is_at_positionD2(self, x, y, z):
        #Check for the correct position of drone
        #Return True if the Drone is at given coordinates
        offset=1
        """offset: meters"""
        rospy.logdebug(
            "current position | x:{0:.2f}, y:{1:.2f}, z:{2:.2f}".format(
                self.posD2.pose.position.x, self.posD2.pose.
                position.y, self.posD2.pose.position.z))

        desired = np.array((x, y, z))
        posi = np.array((self.posD2.pose.position.x,
                        self.posD2.pose.position.y,
                        self.posD2.pose.position.z))
        return np.linalg.norm(desired - posi) < offset
    
    

    def detect_ArUcoD1(self, imgD1):
        #Detect the ArUco marker and Identifies its ArUco ID
        Detected_ArUco_markers = {}
        dictionary = cv.aruco.Dictionary_get(cv.aruco.DICT_5X5_250)
        parameters = aruco.DetectorParameters_create()
        corners, ids, rejected= aruco.detectMarkers(imgD1, dictionary, parameters = parameters)
        try:
            if ids:
                if len(ids)>0:
                    for i in range(0, len(ids)):
                        Detected_ArUco_markers[ids[i][0]]=corners[i][0]
        except ValueError:
            return
        else:
            return Detected_ArUco_markers
    


    def detect_ArUcoD2(self, imgD2):
        #Detect the ArUco marker and Identifies its ArUco ID
        Detected_ArUco_markers = {}
        dictionary = cv.aruco.Dictionary_get(cv.aruco.DICT_5X5_250)
        parameters = aruco.DetectorParameters_create()
        corners, ids, rejected= aruco.detectMarkers(imgD2, dictionary, parameters = parameters)
        try:
            if ids:
                if len(ids)>0:
                    for i in range(0, len(ids)):
                        Detected_ArUco_markers[ids[i][0]]=corners[i][0]
        except ValueError:
            return
        else:
            return Detected_ArUco_markers
 
def main():
    ofb_ctl = offboard_control()
    stateMt = stateMoniter()
    
    #Initialising Publishers
    local_pos_pubD1 = rospy.Publisher('edrone0/mavros/setpoint_position/local',   PoseStamped,  queue_size=10)
    local_pos_pubD2 = rospy.Publisher('edrone1/mavros/setpoint_position/local',   PoseStamped,  queue_size=10)
    local_vel_pubD1 = rospy.Publisher('edrone0/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
    local_vel_pubD2 = rospy.Publisher('edrone1/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
    
    rate = rospy.Rate(20.0)
  
    def publish_setpointD1(setD1,fast=-1):
        #publishing the setpoints till it reaches there
        #fast is for custom power
        global previous_yD1,previous_xD1
        k=False
        #Distance formulae for calculating loop value so that according to distance it publishes for certain period of time
        #Basically Done for improving stability after reaching the position
        loop=abs((((setD1.pose.position.x - previous_xD1)**2) + ((setD1.pose.position.y - previous_yD1)**2) )**0.5)*3.3
        previous_yD1=abs(setD1.pose.position.y)
        previous_xD1=abs(setD1.pose.position.x)
        loop=int(loop)
        if loop<5:
            loop=5
        if fast==180:
            loop=180
        if fast==15:
            loop=15
        if fast==70:
            loop=70
        if fast==5:
            loop=5
        print("drone 1 power:",loop)
        while k!=True:
            for _ in range(0,loop):
                local_pos_pubD1.publish(setD1)
                rate.sleep()
            for _ in range(0,10):
                k = stateMt.is_at_positionD1(setD1.pose.position.x,setD1.pose.position.y,setD1.pose.position.z)
                rate.sleep()
            rate.sleep()
        return k
    
    def publish_setpointD2(setD2,fast=-1):
        #publishing the setpoints till it reaches there
        #fast is for custom power
        k=False
        global previous_yD2,previous_xD2
        #Distance formulae for calculating loop value so that according to distance it publishes for certain period of time
        #Basically Done for improving stability after reaching the position
        loop=abs((((abs(setD2.pose.position.x) - previous_xD2)**2) + ((abs(setD2.pose.position.y) - previous_yD2)**2) )**0.5)*3.3
        previous_yD2=abs(setD2.pose.position.y)
        previous_xD2=abs(setD2.pose.position.x)
        loop=int(loop)
        if loop<5:
            loop=5
        if fast==180:
            loop=180
        if fast==15:
            loop=15
        if fast==70:
            loop=70
        if fast==5:
            loop=5
        print("drone 2 power:",loop)
        while k!=True:
            for _ in range(0,loop):
                local_pos_pubD2.publish(setD2)
                rate.sleep()
            for _ in range(0,10):
                k = stateMt.is_at_positionD2(setD2.pose.position.x,setD2.pose.position.y,setD2.pose.position.z)
                rate.sleep()
            rate.sleep()
        return k
  
    def constant_speedD1():
        #this function makes the drone move in x-axis direction with average speed 
        if stateMt.posD1.pose.position.z>2.5 and stateMt.posD1.pose.position.z<=3.1:
            velD1.twist.linear.z=0
            for _ in range(0,3):
                local_vel_pubD1.publish(velD1)
                rate.sleep()
                
        if stateMt.posD1.pose.position.z>3.1:
            velD1.twist.linear.z = -0.5
            for _ in range(0,3):
                local_vel_pubD1.publish(velD1)
                rate.sleep()
        
        if stateMt.posD1.pose.position.z<=2.5:
            velD1.twist.linear.z = 2
            for _ in range(0,3):
                local_vel_pubD1.publish(velD1)
                rate.sleep()
    
    
    def constant_speedD2():
        #this function makes the drone move in x-axis direction with average speed 
        if stateMt.posD2.pose.position.z>2.5 and stateMt.posD2.pose.position.z<=3.1:
            velD2.twist.linear.z=0
            for _ in range(0,3):
                local_vel_pubD2.publish(velD2)
                rate.sleep()
                
        if stateMt.posD2.pose.position.z>3.1:
            velD2.twist.linear.z = -0.5
            for _ in range(0,3):
                local_vel_pubD2.publish(velD2)
                rate.sleep()
        
        if stateMt.posD2.pose.position.z<=2.5:
            velD2.twist.linear.z = 2
            for _ in range(0,3):
                local_vel_pubD2.publish(velD2)
                rate.sleep()
    
    def D1boxpickup(box_loc):
        #Function to pickup box when the Box is ready to pick
        k=False
        tri=0 #take some tries for repositioning
        while k==False:
            if stateMt.gripcheckD1.data=='True':
                k=True
                for _ in range(0,2):
                    ofb_ctl.setGripperD1(True)
            else:
                tri+=1
                if tri>=5:
                    return False
                publish_setpointD1(box_loc,15)
            rate.sleep()
        return True  

    def D2boxpickup(box_loc):
        #Function to pickup box when the Box is ready to pick
        k=False
        tri=0 #take some tries for repositioning
        while k==False:
            if stateMt.gripcheckD2.data=='True':
                k=True
                for _ in range(0,2):
                    ofb_ctl.setGripperD2(True)
            else:
                tri+=1
                if tri>=5:
                    return False
                publish_setpointD2(box_loc,15)
            rate.sleep()
        return True             

    def box_searchD1(setD1):
        #Function to search for box after entering in a particular row where box is spawned
        waycheck=False
        add_diff_x=0
        add_diff_y=0
        no_vel=0
        boxfind=0
        print("Drone1 box find started")
        while waycheck!=True:
            if stateMt.posD1.pose.position.x>=60 and boxfind==0:#if the drone skips the box then it again scan the row from starting
                setD1.pose.position.x=-3
                publish_setpointD1(setD1)
                    
            if no_vel==0:
                constant_speedD1()
                
            for _ in range(0,2):
                b=stateMt.detect_ArUcoD1(stateMt.imgD1)
            b=stateMt.detect_ArUcoD1(stateMt.imgD1)
            if b:
                #After first detect it publishes that setpoint so as to reach near the box
                setD1.pose.position.x = stateMt.posD1.pose.position.x + add_diff_x
                setD1.pose.position.y = stateMt.posD1.pose.position.y - add_diff_y
                setD1.pose.position.z = 2.8
                publish_setpointD1(setD1)
                #Again Detects box for accurately positioning of Drone so as to land on Box
                b=stateMt.detect_ArUcoD1(stateMt.imgD1)
                if b:
                    idbox=list(b)[0]
                    boxfind=1
                    no_vel=1
                    #Trying to match Center Of ROS Camera Frame with Box being at center
                    if b[idbox][0][0]<210 and b[idbox][0][0]>200 and b[idbox][0][1]<220 and b[idbox][0][1]>210:
                        print("Drone1 Approaching Box")
                        setD1.pose.position.z=-0.3
                        setD1.pose.position.x+=0.24
                        publish_setpointD1(setD1,15)
                        #Call GripCheck function
                        con=D1boxpickup(setD1)
                        if con==True:
                            waycheck=True
                            print("Drone1 Box Ready to pick")
                        else:
                            #Again start scanning by taking 1m back
                            setD1.pose.position.z=2.8
                            setD1.pose.position.x-=1
                            publish_setpointD1(setD1,15)
                            continue
                    else:
                        #Moving the Drone According to the If condition 
                        diffcolx=b[idbox][0][0]-205 #because RoSCamera capturing image size is 400*400 so center is 200,200  
                        add_diff_x=diffcolx/100
                        diffcoly=b[idbox][0][1]-215 #because RoSCamera capturing image size is 400*400 so center is 200,200 
                        add_diff_y=diffcoly/100
            else:
                continue
            rate.sleep()
        return [setD1,idbox]


    def box_searchD2(setD2):
        #Function to search for box after entering in a particular row where box is spawned
        waycheck=False
        add_diff_x=0
        add_diff_y=0
        no_vel=0
        boxfind=0
        print("Drone2 Box Find Started")
        while waycheck!=True:
            if stateMt.posD2.pose.position.x>=60 and boxfind==0: #if the drone skips the box then it again scan the row from starting
                setD2.pose.position.x=-3
                publish_setpointD2(setD2)
                    
            if no_vel==0:
                constant_speedD2()
            
            for _ in range(0,2):
                b=stateMt.detect_ArUcoD2(stateMt.imgD2)
            b=stateMt.detect_ArUcoD2(stateMt.imgD2)
            if b:
                #After first detect it publishes that setpoint so as to reach near the box
                setD2.pose.position.x = stateMt.posD2.pose.position.x + add_diff_x
                setD2.pose.position.y = stateMt.posD2.pose.position.y - add_diff_y
                setD2.pose.position.z = 2.8
                publish_setpointD2(setD2)
                #Again Detects box for accurately positioning of Drone so as to land on Box
                b=stateMt.detect_ArUcoD2(stateMt.imgD2)
                if b:
                    idbox=list(b)[0]
                    boxfind=1
                    no_vel=1
                    #Trying to match Center Of ROS Camera Frame with Box being at center
                    if b[idbox][0][0]<210 and b[idbox][0][0]>200 and b[idbox][0][1]<220 and b[idbox][0][1]>210:
                        print("Drone2 Approaching Box")
                        setD2.pose.position.z=-0.3
                        setD2.pose.position.x+=0.24
                        publish_setpointD2(setD2,15)
                        #Call GripCheck function
                        con=D2boxpickup(setD2)
                        if con==True:
                            waycheck=True
                            print("Drone2 box Ready to pick")
                        else:
                            #Again start scanning by taking 1m back
                            setD2.pose.position.z=2.8
                            setD2.pose.position.x-=1
                            publish_setpointD2(setD2,15)
                            continue
                    else:
                        diffcolx=b[idbox][0][0]-205#because RoSCamera capturing image size is 400*400 so center is 200,200  
                        add_diff_x=diffcolx/100
                        diffcoly=b[idbox][0][1]-215 #because RoSCamera capturing image size is 400*400 so center is 200,200 
                        add_diff_y=diffcoly/100
            else:
                continue
                    
            rate.sleep()
        return [setD2,idbox]
        
    def calculate_blue_blockD1(cellx,celly):
        #Calculating location of blue cells according to Drone1
        if cellx==0:
            boxdrop_loc  = (15.2, -8.8+celly*1.23, 7)
        else:
            boxdrop_loc  = (14.85+cellx*0.9, -8.8+celly*1.23, 7)
        return boxdrop_loc
    
    def calculate_red_blockD1(cellx,celly):
        #Calculating location of Red cells according to Drone1
        if cellx==0:
            boxdrop_loc  = (58, 63.8+celly*1.23, 7)
        else:
            boxdrop_loc  = (57.5+cellx*1, 63.8+celly*1.23, 7)
        return boxdrop_loc   
    
    def calculate_blue_blockD2(cellx,celly):
        #Calculating location of blue cells according to Drone2
        if cellx==0:
            boxdrop_loc  = (15.2, -68.8+celly*1.23, 6)
        else:
            boxdrop_loc  = (14.85+cellx*0.9, -68.8+celly*1.23, 6)
        return boxdrop_loc
    
    def calculate_red_blockD2(cellx,celly):
        #Calculating location of Red cells according to Drone2
        if cellx==0:
            boxdrop_loc  = (58, 3.8+celly*1.23, 6)
        
        else:
            boxdrop_loc  = (57.5+cellx*1, 3.8+celly*1.23, 6)
        return boxdrop_loc    
    
    ourD1 = PoseStamped()
    ourD1.pose.position.x = 0.0
    ourD1.pose.position.y = 0.0
    ourD1.pose.position.z = 5.0
    
    ourD1.pose.orientation.x=10
    ourD1.pose.orientation.y=0
    ourD1.pose.orientation.z=0
    ourD1.pose.orientation.w=0
    
    
    ourD2 = PoseStamped()
    ourD2.pose.position.x = 0.0
    ourD2.pose.position.y = 0.0
    ourD2.pose.position.z = 3.0
    
    ourD2.pose.orientation.x=10
    ourD2.pose.orientation.y=0
    ourD2.pose.orientation.z=0
    ourD2.pose.orientation.w=0
    
    velD1 = TwistStamped()
    velD1.twist.linear.x = 1.5
    velD1.twist.linear.y = 0
    velD1.twist.linear.z = 0
    
    velD2 = TwistStamped()
    velD2.twist.linear.x = 1.5
    velD2.twist.linear.y = 0
    velD2.twist.linear.z = 0
    
    
    #Initialising Subscribers
    rospy.Subscriber("/edrone0/mavros/state", State, stateMt.state_callbackD1)
    rospy.Subscriber("/edrone1/mavros/state", State, stateMt.state_callbackD2 )
    rospy.Subscriber("/edrone0/mavros/local_position/pose" , PoseStamped, stateMt.local_position_callbackD1 )
    rospy.Subscriber("/edrone1/mavros/local_position/pose" , PoseStamped, stateMt.local_position_callbackD2 )
    rospy.Subscriber("/edrone0/gripper_check", String, stateMt.gripper_callbackD1)
    rospy.Subscriber("/edrone1/gripper_check", String, stateMt.gripper_callbackD2)
    rospy.Subscriber("/edrone0/camera/image_raw", Image, stateMt.image_callbackD1)
    rospy.Subscriber("/edrone1/camera/image_raw", Image, stateMt.image_callbackD2)
    rospy.Subscriber("/spawn_info",UInt8, stateMt.spawn_callback)

    
    #Sending sample Coordinates For OFFBOARD Mode 
    for i in range(0,100):
        local_pos_pubD1.publish(ourD1)
        local_pos_pubD2.publish(ourD2)
        rate.sleep()
        
    rcl_except = mavros_msgs.msg.ParamValue(1<<2,0.0)

    ofb_ctl.set_paramD1("COM_RCL_EXCEPT",rcl_except)
    ofb_ctl.set_paramD2("COM_RCL_EXCEPT",rcl_except)
    rate.sleep()
        

    ofb_ctl.set_modeD1("OFFBOARD")
    ofb_ctl.set_modeD2("OFFBOARD")
    rate.sleep()
    print("OFFBOARD mode Activated")
    
    #Arming The Drone by calling Function setArm
    for i in range(0,30):
        ofb_ctl.setArmD1()
        ofb_ctl.setArmD2()
        rate.sleep()
    print("Drone 1 and 2 Armed")
    
#WE ARE USING QUEUE DATA STRUCUTURE CONCEPT, WE ARE POPING THE FIRST INDEX OF SETPOINT LIST AND TRAVERSING IT 
#AS MORE SETPOINT ARE ADDED AT THE END OF LIST WHILE RUNNING
    
    def drone1():
        global setpointsD1,assigned_blue,assigned_red,rowsD1
        publish_setpointD1(ourD1)
        print("Drone1 Started")
        i=0
        spacefound=False
        while setpointsD1:
            ourD1.pose.position.x=setpointsD1[0][0]
            ourD1.pose.position.y=setpointsD1[0][1]
            ourD1.pose.position.z=setpointsD1[0][2]
            ourD1.pose.orientation.x=10
            if i==0:
                publish_setpointD1(ourD1,180)
            else:
                publish_setpointD1(ourD1)
            print(f"Drone1 Reaching Setpoint {i}  : ",ourD1.pose.position)
            print("Drone1 Row Approached",rowsD1[i])
#            ourD1.pose.position.z=3
#            publish_setpointD1(ourD1,15)
            box=box_searchD1(ourD1)
            box_loc=box[0]
            box_id=box[1]
            box_loc.pose.position.z=7
            print("Drone1 Going Up")
            publish_setpointD1(box_loc,5)
            #Assigning only vacant cells location to Drone1 
            if box_id==1:
                while spacefound!=True:
                    for sp1 in range(0,4):
                        for sp2 in range(0,3):
                            spacecheck=(sp1,sp2)
                            if spacecheck not in assigned_red:
                                assigned_red.append(spacecheck)
                                spacefound=True
                                break
                        if spacefound==True:
                            break
                    if spacefound==False and sp1==3:
                        assigned_red=[]
                        continue
                spacefound=False
                drop_loc=calculate_red_blockD1(spacecheck[0],spacecheck[1])
                ourD1.pose.position.x=drop_loc[0]
                ourD1.pose.position.y=drop_loc[1]
                ourD1.pose.position.z=drop_loc[2]
                print("Drone1 Going to Box Loc")
                publish_setpointD1(ourD1)
                ourD1.pose.position.z=1.6
                print("Drone1 Going Down")
                publish_setpointD1(ourD1,5)
                #for _ in range(0,2):
                ofb_ctl.setGripperD1(False)
                print("Drone1 Gripper Deactivated")
#                if spacecheck[1]==2:
##                    ourD1.pose.position.z=2
##                    publish_setpointD1(ourD1,5)
#                    ourD1.pose.position.y=68.8
#                    publish_setpointD1(ourD1,5)
#                    
#                elif spacecheck[1]==0:
##                    ourD1.pose.position.z=2
##                    publish_setpointD1(ourD1,5)
#                    ourD1.pose.position.y=60.8
#                    publish_setpointD1(ourD1,5)
#                else:
##                    ourD1.pose.position.z=2
##                    publish_setpointD1(ourD1,5)
#                    ourD1.pose.position.y=64
#                    publish_setpointD1(ourD1,5)
                ourD1.pose.position.z=7
                publish_setpointD1(ourD1,5)
            #Assigning only vacant cells location to Drone1        
            elif box_id==2:
                while spacefound!=True:
                    for sp1 in range(0,4):
                        for sp2 in range(0,3):
                            spacecheck=(sp1,sp2)
                            if spacecheck not in assigned_blue:
                                assigned_blue.append(spacecheck)
                                spacefound=True
                                break
                        if spacefound==True:
                            break
                    if spacefound==False and sp1==3:
                        assigned_blue=[]
                        continue
                spacefound=False
                drop_loc=calculate_blue_blockD1(spacecheck[0],spacecheck[1])
                ourD1.pose.position.x=drop_loc[0]
                ourD1.pose.position.y=drop_loc[1]
                ourD1.pose.position.z=drop_loc[2]
                print("D1 Going to box loc")
                publish_setpointD1(ourD1)
                ourD1.pose.position.z=1.6
                print("D1 Going down")
                publish_setpointD1(ourD1,5)
                #for _ in range(0,2):
                ofb_ctl.setGripperD1(False)
                print("Drone1 Gripper Deactivated")
##                if spacecheck[1]==2:
###                    ourD1.pose.position.z=2
###                    publish_setpointD1(ourD1,5)
##                    ourD1.pose.position.y=-3.8
##                    publish_setpointD1(ourD1,5)
##                elif spacecheck[1]==0:
###                    ourD1.pose.position.z=2
###                    publish_setpointD1(ourD1,5)
##                    ourD1.pose.position.y=-10.8
##                    publish_setpointD1(ourD1,5)
##                else:
###                    ourD1.pose.position.z=2
###                    publish_setpointD1(ourD1,5)
#                    ourD1.pose.position.y=-8.6
#                    publish_setpointD1(ourD1,5)
                ourD1.pose.position.z=7
                publish_setpointD1(ourD1,5)
            print("Red Cells Used  : ", assigned_red)
            print("Blue Cells Used : ", assigned_blue)
            setpointsD1.pop(0)
            i+=1
            rate.sleep()


    def drone2():
        global setpointsD2,assigned_blue,assigned_red,rowsD2
        publish_setpointD2(ourD2)
        print("Drone2 Started")
        i=0
        spacefound=False
        while setpointsD2:
            ourD2.pose.position.x=setpointsD2[0][0]
            ourD2.pose.position.y=setpointsD2[0][1]
            ourD2.pose.position.z=setpointsD2[0][2]
            ourD2.pose.orientation.x=10
            if i==0:
                publish_setpointD2(ourD2,180)
            else:
                publish_setpointD2(ourD2)                
            print("Drone2 Row Approached : ",rowsD2[i])
            box=box_searchD2(ourD2)
            box_loc=box[0]
            box_id=box[1]
            box_loc.pose.position.z=6
            print("Drone2 Going Up")
            publish_setpointD2(box_loc,5)
            #Assigning only vacant cells location to Drone2
            if box_id==1:
                while spacefound!=True:
                    for i1 in range(0,4):
                        for j1 in range(0,3):
                            spacecheck=(i1,j1)
                            if spacecheck not in assigned_red:
                                assigned_red.append(spacecheck)
                                spacefound=True
                                break
                        if spacefound==True:
                            break
                    if spacefound==False and i1==3:
                        assigned_red=[]
                        continue
                spacefound=False
                drop_loc=calculate_red_blockD2(spacecheck[0],spacecheck[1])
                ourD2.pose.position.x=60
                ourD2.pose.position.y=0.2
                ourD2.pose.position.z=6
                publish_setpointD2(ourD2)
                ourD2.pose.position.x=drop_loc[0]
                ourD2.pose.position.y=drop_loc[1]
                ourD2.pose.position.z=drop_loc[2]
                print("D2 Going to box loc")
                publish_setpointD2(ourD2)
                ourD2.pose.position.z=1.6
                print("D2 Going Down")
                publish_setpointD2(ourD2,5)
                #for _ in range(0,2):
                ofb_ctl.setGripperD2(False)
                print("Drone2 Gripper Deactivated")
#                if spacecheck[1]==2:
##                    ourD2.pose.position.z=2
##                    publish_setpointD2(ourD2,5)
#                    ourD2.pose.position.y=9
#                    publish_setpointD2(ourD2,5)
#                elif spacecheck[1]==0:
##                    ourD2.pose.position.z=2
##                    publish_setpointD2(ourD2,5)
#                    ourD2.pose.position.y=0.8
#                    publish_setpointD2(ourD2,5)
#                else:
##                    ourD2.pose.position.z=2
##                    publish_setpointD2(ourD2,5)
#                    ourD2.pose.position.y=4
#                    publish_setpointD2(ourD2,5)
                ourD2.pose.position.z=6
                publish_setpointD2(ourD2,5)
                
            #Assigning only vacant cells location to Drone2       
            elif box_id==2:
                while spacefound!=True:
                    for i1 in range(0,4):
                        for j1 in range(0,3):
                            spacecheck=(i1,j1)
                            if spacecheck not in assigned_blue:
                                assigned_blue.append(spacecheck)
                                spacefound=True
                                break
                        if spacefound==True:
                            break
                    if spacefound==False and i1==3:
                        assigned_blue=[]
                        continue
                spacefound=False
                drop_loc=calculate_blue_blockD2(spacecheck[0],spacecheck[1])
                ourD2.pose.position.x=17
                ourD2.pose.position.y=-62.8
                ourD2.pose.position.z=6
                publish_setpointD2(ourD2,15)
                ourD2.pose.position.x=drop_loc[0]   
                ourD2.pose.position.y=drop_loc[1]
                ourD2.pose.position.z=drop_loc[2]
                print("Drone2 Going to Box loc")
                publish_setpointD2(ourD2)
                ourD2.pose.position.z=1.6
                print("Drone2 Going Down")
                publish_setpointD2(ourD2,5)
                #for _ in range(0,2):
                ofb_ctl.setGripperD2(False)
                print("Drone2 Gripper Deactivated")
#                if spacecheck[1]==2:
##                    ourD2.pose.position.z=2
##                    publish_setpointD2(ourD2,5)
#                    ourD2.pose.position.y=-64.8
#                    publish_setpointD2(ourD2,5)
#                elif spacecheck[1]==0:
##                    ourD2.pose.position.z=2
##                    publish_setpointD2(ourD2,5)
#                    ourD2.pose.position.y=-70.8
#                    publish_setpointD2(ourD2,5)
#                else:
##                    ourD2.pose.position.z=2
##                    publish_setpointD2(ourD2,5)
#                    ourD2.pose.position.y=-68.4
#                    publish_setpointD2(ourD2,5)            
                ourD2.pose.position.z=6
                publish_setpointD2(ourD2,5)
            setpointsD2.pop(0)
            i+=1
            rate.sleep()

            
    while not rospy.is_shutdown():
        #Threading for independent operations of drone
        t1=threading.Thread(target=drone1)
        t2=threading.Thread(target=drone2)
        
        t1.start()
        t2.start()
        
        t1.join()
        t2.join()
        
        print("Mission Over")
        print("Rows to be covered by Drone1 : ",rowsD1)
        print("Rows to be covered by Drone2 : ",rowsD2)
        break
        rate.sleep()
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass