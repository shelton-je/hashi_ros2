#!/usr/bin/env python3

import numpy as np
from typing import List, Dict, Tuple
import serial
import os
import time
from math import *
import rclpy
from rclpy.node import Node
import traceback
from threading import Lock
from std_msgs.msg import Float32MultiArray, Int32MultiArray
from hashi.msg import Teleop

from dynamixel_sdk.port_handler import PortHandler # goes into the port_handler.py file and imports the class PortHandler
from dynamixel_sdk.packet_handler import PacketHandler # does the same as the previous line but PacketHandler is a function
from dynamixel_sdk.group_sync_write import GroupSyncWrite
from dynamixel_sdk.group_sync_read import GroupSyncRead
from dynamixel_sdk.robotis_def import COMM_SUCCESS, DXL_HIBYTE, DXL_LOBYTE, DXL_HIWORD, DXL_LOWORD

from time import sleep


# Chopstick inverse kinematics calculation and function definition
# declare constants

lj = 32.5 # the length of the ball joint linkages in mm
lps = 28 # the length of the pitch servo horn in mm
lys = 36 # the length of the yaw servo horn in mm
lc = 168 # the length of the multibody chopstick assembly after the center of the pivot in mm
zOffset = 57.2 # distance from the center of the M8 ball joint to the TOP of the acrylic base plate in mm 
mmPerTick = 0.00244140625
#********* DYNAMIXEL Model definition *********
#***** (Use only one definition at a time) *****
MY_DXL = 'X_SERIES'       # X330 (5.0 V recommended), X430, X540, 2X430

# Control table address for continuous rotation dynamixel
ADDR_OPERATING_MODE         = 11        
DXL_OPERATING_MODE          = 4         # puts the dynamixel in continuous position mode
PWM_OPERATING_MODE          = 16
ADDR_GOAL_PWM               = 100       # address for the goal PWM value
LEN_GOAL_PWM                = 4
ADDR_GOAL_VELOCITY          = 104
LEN_GOAL_VELOCITY           = 4
ADDR_TORQUE_ENABLE          = 64
ADDR_GOAL_POSITION          = 116
LEN_GOAL_POSITION           = 4         # Data Byte Length
ADDR_PRESENT_POSITION       = 132
LEN_PRESENT_POSITION        = 4         # Data Byte Length
DXL_MINIMUM_POSITION_VALUE  = 1821      # 160.05 degrees
DXL_MAXIMUM_POSITION_VALUE  = 2275      # 199.95 degrees
ADDR_HOMING_OFFSET          = 20        # homing offset
LEN_HOMING_OFFSET           = 4
BAUDRATE                    = 57600
ADDR_HARDWARE_ERROR         = 70
LEN_HARDWARE_ERROR          = 1
# DYNAMIXEL Protocol Version (1.0 / 2.0)
PROTOCOL_VERSION            = 2.0

# Make sure that each DYNAMIXEL ID should have unique ID.
DXL1_ID                     = 1
DXL2_ID                     = 2
DXL3_ID                     = 3
DXL4_ID                     = 4
DXL5_ID                     = 5
DXL6_ID                     = 6

class HashiControl(Node):

    def __init__(self):
        super().__init__("hashi")
        # Initialize PortHandler instance

        # define different lists for testing purposes
        self.ALL_DXL_IDS = [DXL1_ID, DXL2_ID, DXL3_ID, DXL4_ID, DXL5_ID, DXL6_ID]

        self.POSITION_DXL_IDS = [DXL2_ID, DXL3_ID, DXL5_ID, DXL6_ID]
        self.LINEAR_DXL_IDS = [DXL1_ID, DXL4_ID]

        self.PLATFORM0_DXL_IDS = [DXL1_ID, DXL2_ID, DXL3_ID]
        self.PLATFORM1_DXL_IDS = [DXL4_ID, DXL5_ID, DXL6_ID]

        # Use the actual port assigned to the U2D2.
        # ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
        self.DEVICENAME                  = '/dev/ttyUSB0'

        self.TORQUE_ENABLE               = 1                 # Value for enabling the torque
        self.TORQUE_DISABLE              = 0                 # Value for disabling the torque
        self.DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold
        self.HOMING_OFFSET_VALUE         = 0                 # Initializes the homing offset as zero
        self.MAX_CW_PWM                  = -885
        self.MAX_CCW_PWM                 = 885

        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.portHandler = PortHandler(self.DEVICENAME)
        # Open U2D2 port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")

        # Set port baudrate
        if self.portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")

        # Initialize PacketHandler instance
        # Set the protocol version
        # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        # Initialize GroupSyncWrite instance
        self.groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)

        # Initialize GroupSyncRead instance for Present Position
        self.groupSyncRead = GroupSyncRead(self.portHandler, self.packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        self.groupSyncReadAlarm = GroupSyncRead(self.portHandler, self.packetHandler, ADDR_HARDWARE_ERROR, LEN_HARDWARE_ERROR)
        self.groupSyncWritePWM = GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_GOAL_PWM, LEN_GOAL_PWM)


        self.pub = self.create_publisher(Float32MultiArray, 'actual_motor_positions', 10)
        self.pub_srv_lock = Lock()
        
        # Create connection to the QTPy
        self.create_serial_connection()

        # Initialize the upright zeros
        self.initializeDynamixels()

        # Initialize the continuous rotation servos
        self.initializePlatformDynamixels(0)
        self.initializePlatformDynamixels(1)

        self.sub = self.create_subscription(Teleop, '/hashi/commands', self.teleop_command_process, 10)

    # Open serial port for microcontroller interface
    def create_serial_connection(self):
        self.ser = serial.Serial(port='/dev/ttyACM0', baudrate=9600, timeout=None)


    def initializeDynamixels(self):
        """Initializes all of the dynamixels. Puts z-axis dynamixels in continuous position, enables torque for all, 
        adds groupSyncRead parameter storage for all 
        """

        for dxl_id in self.LINEAR_DXL_IDS:

            # put all linear dynamixels in continuous position mode
            dxl_comm_result_operating, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_OPERATING_MODE, DXL_OPERATING_MODE)
            dxl_comm_result_torque, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
            dxl_addparam_result = self.groupSyncRead.addParam(dxl_id)

            if dxl_comm_result_operating != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result_operating))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print("Dynamixel#%d has been successfully put in continuous position mode" % dxl_id)

            # now enable torques for the linear dynamixels
            if dxl_comm_result_torque != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result_operating))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print("Dynamixel#%d has been successfully connected" % dxl_id)
            
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncRead addparam failed" % dxl_id)
                quit()

        # enable the torque for the remaining dynamixels, by default they're in position mode 

        for dxl_id in self.POSITION_DXL_IDS:

            # enable all Dynamixel Torques
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print("Dynamixel#%d has been successfully connected" % dxl_id)
            
            # Add parameter storage for Dynamixel present position value
            dxl_addparam_result = self.groupSyncRead.addParam(dxl_id)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncRead addparam failed" % dxl_id)

                quit()


    def initializePlatformDynamixels(self, platform):
        """Initializes all dynamixels for a specified platform

        Args: an integer between 0 and 1 corresponding to the desired platform
        """
        if platform == 0:
            dxls = self.PLATFORM0_DXL_IDS
        elif platform == 1: 
            dxls = self.PLATFORM1_DXL_IDS
        else:
            print("No platform specified")

        # linear dxl first
        dxl_comm_result_operating, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, dxls[0], ADDR_OPERATING_MODE, DXL_OPERATING_MODE)
        dxl_comm_result_torque, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, dxls[0], ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
        dxl_addparam_result = self.groupSyncRead.addParam(dxls[0])

        for dxl in dxls[1:]:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl, ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
            dxl_addparam_result = self.groupSyncRead.addParam(dxl)

        print("Platform %d Dynamixels successfully initialized" % platform)


    # Float Float Float Float Float Float -> void
    def teleop_command_process(self, req: Teleop):
        x_l,y_l,z_l,x_r,y_r,z_r = req.x_l,req.y_l,req.z_l,req.x_r,req.y_r,req.z_r
        try:
            self.movePlatform(0, np.array([[x_l,y_l,z_l],[x_l,y_l,z_l]]))
        except Exception as e:
            print(traceback.print_exc(e))
        try:
            self.movePlatform(1, np.array([[x_r,y_r,z_r],[x_r,y_r,z_r]]))
        except Exception as e:
            print(traceback.print_exc(e))

    
    def movePlatform(self, platform, coordPositions):
        print(coordPositions)
        """Moves specified platform through any number of specified cartesian points accepted by servoAngles()
        scripts/broadcast_platform_zero_transform.py scripts/force_torque_validation.py scripts/optitrack_validation.py scripts/zero_platform_and_record_origin_tf.py
        Args: platform integer number, np.array() of 1x3 coordinates
        """
        if platform == 0:
            dxls = self.PLATFORM0_DXL_IDS
        elif platform == 1: 
            dxls = self.PLATFORM1_DXL_IDS
        else:
            print("No platform specified")

        for i in range(len(coordPositions)):
            
            if coordPositions[i,0] < -50 or coordPositions[i,0] > 50:
                break
            elif coordPositions[i,1] < -50 or coordPositions[i,1] > 50:
                break
            elif coordPositions[i,2] < 225 or coordPositions[i,2] > 260:
                break

            [zTick, pitchTick, yawTick] = self.sphericalServoAngles(coordPositions[i])
            # [zTick,pitchTick,yawTick] = self.servoAngles(coordPositions[i])
            z_goal_position = [DXL_LOBYTE(DXL_LOWORD(zTick)), DXL_HIBYTE(DXL_LOWORD(zTick)), DXL_LOBYTE(DXL_HIWORD(zTick)), DXL_HIBYTE(DXL_HIWORD(zTick))]
            pitch_goal_position = [DXL_LOBYTE(DXL_LOWORD(pitchTick)), DXL_HIBYTE(DXL_LOWORD(pitchTick)), DXL_LOBYTE(DXL_HIWORD(pitchTick)), DXL_HIBYTE(DXL_HIWORD(pitchTick))] 
            yaw_goal_position = [DXL_LOBYTE(DXL_LOWORD(yawTick)), DXL_HIBYTE(DXL_LOWORD(yawTick)), DXL_LOBYTE(DXL_HIWORD(yawTick)), DXL_HIBYTE(DXL_HIWORD(yawTick))]

            z_addparam_result = self.groupSyncWrite.addParam(dxls[0],z_goal_position)
            if z_addparam_result != True:
                print(z_addparam_result)
                print("[ID:%03d] groupSyncWrite addparam failed for z" % dxls[0])
                quit()
            pitch_addparam_result = self.groupSyncWrite.addParam(dxls[1], pitch_goal_position)
            if pitch_addparam_result!= True:
                print(pitch_addparam_result)
                print("[ID:%03d] groupSyncWrite addparam failed for pitch" % dxls[1])
                quit()
            yaw_addparam_result = self.groupSyncWrite.addParam(dxls[2], yaw_goal_position)
            if yaw_addparam_result != True:
                print("[ID:%03d] groupSyncWrite addparam failed for yaw" % dxls[2])
                quit()
            
            # syncwrite current goal position
            dxl_comm_result = self.groupSyncWrite.txPacket()
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            time.sleep(0.01)
            # Clear syncwrite parameter storage
            self.groupSyncWrite.clearParam()
            
            # Syncread present position
            dxl_comm_result = self.groupSyncRead.txRxPacket()
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            
            # get present position values
            z_present_position = self.groupSyncRead.getData(dxls[0], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            pitch_present_position = self.groupSyncRead.getData(dxls[1], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            yaw_present_position = self.groupSyncRead.getData(dxls[2], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

            # print present position values
            print("The present positions of platform %d are z: %03d, pitch: %03d, yaw: %03d" % (platform, z_present_position, pitch_present_position, yaw_present_position))

            # clear syncread parameter storage
            self.groupSyncRead.clearParam()

        print("Platform %d is done moving" % platform)

    
    def sphericalServoAngles(self, coords: np.array) -> Tuple[int, int, int]:
        """Function to calculate the pitch, yaw, and linear servo positions based on formulation of the chopsticks in modified spherical coordinates.
            x and y coordinates are relative to an origin at the M8 ball joint. Positive X is to the right, in line with the threaded rod end of the pivot, and positive Y it towards the HASHI logo in the center
            z coordinate is relative to the pivot as well
            
        Args: a 1x3 np.array() of coordinates for (x,y,z)
        
        Returns: three motor tick values"""

        # define all functions used

        psiFunc = lambda x, y : atan2(x,y)
        phiFunc = lambda y, psi: asin(y/(lc*cos(psi)))
        phiFunc2 = lambda x, psi: asin(x/(lc*sin(psi)))

        z1 = lambda Phi : lc*cos(Phi)

        xcb = lambda phi, psi, lb : -lb*sin(phi)*sin(psi)
        ycb = lambda phi, psi, lb : -lb*sin(phi)*cos(psi)
        zcb = lambda phi, lb : -lb*cos(phi)

        # equations for calculating servo angle

        delAngle = lambda h1, v1, h2, v2 : degrees(atan2(h2-h1,v2-v1))
        delAngle2 = lambda h1, v1, h2, v2: degrees(atan((h2-h1)/(v2-v1)))

        # platform linear travel

        di = lambda Z, Phi : Z - lc*cos(Phi) - zOffset

        #############################
        # actual calculations

        # first extract coordinates

        x = coords[0]
        y = coords[1]
        z = coords[2]

        r = sqrt(x**2 + y**2) # used for calculating z with only xy-position and known chopstick length
        calcZ = sqrt(lc**2 - r**2)
        # calculate Phi and Psi based on x and y

        Psi = psiFunc(x,y) # 
        Phi = acos(calcZ/lc)

        # use these to calculate the spherical z-coordinate in the frame of the pivot (ignoring platform linear travel)
        z1 = lc*cos(Phi)

        # calculate linear travel using inputted z-coordinate and calculated phi

        delZ = di(z,Phi)
        zTick = -round(delZ/mmPerTick)

        # now pitch and yaw circular intersection calculations

        # pitch M2 mount xyz position
        xcbp = xcb(Phi, Psi, lps)
        ycbp = ycb(Phi, Psi, lps)
        zcbp = zcb(Phi, lps)
        # print("The location of the pitch M2 connection is X:%03f Y:%03f Z:%03f" % (xcbp, ycbp, zcbp))

        # yaw M2 mount xyz position
        xcby = xcb(Phi, Psi, lys)
        ycby = ycb(Phi, Psi, lys)
        zcby = zcb(Phi, lys)
        # print("The location of the yaw M2 connection is X:%03f Y:%03f Z:%03f" % (xcby, ycby, zcby))
        
        # calculate projected radius of circle with pitch or yaw plane 
        rProj = lambda r, a : sqrt(r**2 - a**2) # r is the original radius of the sphere and a is the x-coordinate of the sphere

        prProj = rProj(32.5,xcbp)
        yrProj = rProj(32.5,ycby)
        # print("pitch: %03f yaw: %03f" % (prProj,yrProj))

        # call circle intersection function
        pitch_intersections = self.circle_intersection(ycbp,zcbp,prProj,-32.5,0,lps)
        # print("Calculated pitch points: " + str(pitch_intersections))
        yaw_intersections = self.circle_intersection(xcby,zcby,yrProj,-32.5,0,lys)
        # print("Calculated yaw points: " + str(yaw_intersections))

        # reverse_pitch_intersections = circle_intersection(-32.5,0,lps,ycbp,zcbp,lj)
        # print("Reverse pitch stuff: " + str(reverse_pitch_intersections))
        # reverse_yaw_intersections = circle_intersection(-32.5,0,lys,xcby,zcby,lj)
        # print("reverse yaw stuff: " + str(reverse_yaw_intersections))

        # call different circle intersection function
        # pitch_intersections = other_circle_intersection(ycbp,zcbp,lj,-32.5,0,lps)
        # print("Calculated pitch points with the other function: " + str(pitch_intersections))
        # yaw_intersections = other_circle_intersection(xcby,zcby,lj,-32.5,0,lys)
        # print("Calculated yaw points with the other function: " + str(yaw_intersections))

        # put some function to throw out redundant solution
        # this function checks the horizontal coordinate and makes sure that it isn't out of range
        if pitch_intersections[0][0] > -(lj - lps*sin(radians(20))): 
            del pitch_intersections[0]
        else:
            del pitch_intersections[1]
        
        if yaw_intersections[0][0] > -(lj - lys*sin(radians(20))):
            del yaw_intersections[0]
        else:
            del yaw_intersections[1]
        
        print("Remaining pitch point: " + str(pitch_intersections))
        print("Remaining yaw point: " + str(yaw_intersections))

        # now calculate servo angles based on coordinates
        delPitch2 = delAngle2(-32.5,0,pitch_intersections[0][0],pitch_intersections[0][1])
        # delPitchAngle = delAngle(-32.5,-lps,-31.5,-26) + 180
        # print("delta pitch angle is: %03f" % delPitch2)
        print("del pitch angle2 is: " + str(delPitch2))
        print("delta pitch horizontal travel: " + str(28*sin(delPitch2)))
        # delYawAngle = delAngle(-32.5,0,yaw_intersections[0][0],yaw_intersections[0][1])
        # delYawAngle = delAngle(yaw_intersections[0][0],yaw_intersections[0][1],-32.5,-lys)
        delYaw2 = delAngle2(-32.5,0,yaw_intersections[0][0],yaw_intersections[0][1])
        # print("delta yaw angle is: %03f" % delYaw2)
        print("del yaw 2 is: " + str(delYaw2))
        print("delta yaw horizontal travel: " + str(36*sin(delYaw2)))
        # put those servos into motor tick values

        pitchTick = round((4096*(delPitch2+180))/360)
        print(pitchTick)
        yawTick = round((4096*(delYaw2+180))/360)
        print(yawTick)

        return zTick, pitchTick, yawTick

   
    def circle_intersection(self, a1, b1, r1, a2, b2, r2):
        # first points are location of circle slice from chopstick butt
        # second points are location of circle on specified pitch/yaw horn
        # Calculate the distance between the centers of the circles
        d = sqrt((a2 - a1)**2 + (b2 - b1)**2)

        # Calculate the intersection points
        a = (r1**2 - r2**2 + d**2) / (2 * d)
        # print("the calculated a value is: " + str(a))
        h = sqrt(r1**2 - a**2)
        # print(h)

        # Calculate the coordinates of the intersection points
        x3 = a1 + (a * (a2 - a1)) / d
        y3 = b1 + (a * (b2 - b1)) / d

        if h == 0:
            return [(x3, y3)]  # Tangent circles, one intersection point
        else:
            # Calculate the two intersection points
            x4 = round(x3 + (h * (b2 - b1)) / d, 3)
            y4 = round(y3 - (h * (a2 - a1)) / d, 3)
            x5 = round(x3 - (h * (b2 - b1)) / d, 3)
            y5 = round(y3 + (h * (a2 - a1)) / d, 3)
            return [(x4, y4), (x5, y5)]
        

    def pwmHoming(self, platform):
        """Homing routine for each platform. Moves the z-axis until a microswitch connected to the microcontroller is pressed
        
        """
        if platform == 0:
            dxls = self.PLATFORM0_DXL_IDS
        elif platform == 1: 
            dxls = self.PLATFORM1_DXL_IDS
        else:
            print("No platform specified")

        # zero pitch and yaw dynamixels
        zeroTick = 2048  # zero position in this case is dynamixel at 180 degrees
        zero_goal_position1 = [DXL_LOBYTE(DXL_LOWORD(zeroTick)), DXL_HIBYTE(DXL_LOWORD(zeroTick)), DXL_LOBYTE(DXL_HIWORD(zeroTick)), DXL_HIBYTE(DXL_HIWORD(zeroTick))] 
        zero_goal_position2 = [DXL_LOBYTE(DXL_LOWORD(zeroTick)), DXL_HIBYTE(DXL_LOWORD(zeroTick)), DXL_LOBYTE(DXL_HIWORD(zeroTick)), DXL_HIBYTE(DXL_HIWORD(zeroTick))] 
        
        pitch_addparam_result = self.groupSyncWrite.addParam(dxls[1], zero_goal_position1)
        if pitch_addparam_result != True:
            print(pitch_addparam_result)
            quit()
        yaw_addparam_result = self.groupSyncWrite.addParam(dxls[2], zero_goal_position2)
        if yaw_addparam_result != True:
            print(yaw_addparam_result)
            quit()

        dxl_comm_result = self.groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        self.groupSyncWrite.clearParam()

        sleep(1)

        byteState = self.ser.readline(1)
        print(byteState)
        # print("the bytestate for platform %d is %d" % (platform, int.from_bytes(byteState,"little")))
        buttonState = 0
        # print(buttonState)

        # # first check if torque is enabled
        # torqueState = packetHandler.read1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE)
        # # torqueState should be 1 when torque is enabled
        # if torqueState == 1:
        #     dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, dxls[0], ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
        # put dynamixel in pwm mode
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, dxls[0], ADDR_OPERATING_MODE, PWM_OPERATING_MODE)
        # enable torque again
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, dxls[0], ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)

        # print(HOMING_OFFSET_VALUE)
        # get initial dynamixel position
        dxl_comm_result = self.groupSyncRead.txRxPacket()
        dxl_initial_position = self.groupSyncRead.getData(dxls[0], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION) 
        print("The dynamixel initial position is: %d" % dxl_initial_position)
        # print(HOMING_OFFSET_VALUE)

        # CCW PWM moves the platform closer to the base
        start_pwm = 300
        stop_pwm = 0
        dxl_start_pwm = [DXL_LOBYTE(DXL_LOWORD(start_pwm)), DXL_HIBYTE(DXL_LOWORD(start_pwm)), DXL_LOBYTE(DXL_HIWORD(start_pwm)), DXL_HIBYTE(DXL_HIWORD(start_pwm))]  
        dxl_stop_pwm = [DXL_LOBYTE(DXL_LOWORD(stop_pwm)), DXL_HIBYTE(DXL_LOWORD(stop_pwm)), DXL_LOBYTE(DXL_HIWORD(stop_pwm)), DXL_HIBYTE(DXL_HIWORD(stop_pwm))]  
        addparam_result = self.groupSyncWritePWM.addParam(dxls[0], dxl_start_pwm)
        dxl_comm_result = self.groupSyncWritePWM.txPacket()  # writes the start PWM value
        print("dynamixel has started moving")
        self.groupSyncWritePWM.clearParam()  # in testing, it seemed to still work fine with this commented out

        # THERE IS A DELAY HERE 
        
        while True:
            
            byteState = self.ser.readline(1)
            print(byteState)
            buttonState = int.from_bytes(byteState,"little")

            if buttonState == 1 and platform == 0:
                addparam_result = self.groupSyncWritePWM.addParam(dxls[0], dxl_stop_pwm)
                addparam_result = self.groupSyncWritePWM.txPacket()
                break
            elif buttonState == 2 and platform == 1:  # button 2
                addparam_result = self.groupSyncWritePWM.addParam(dxls[0], dxl_stop_pwm)
                addparam_result = self.groupSyncWritePWM.txPacket()
                break
        

        addparam_result = self.groupSyncWritePWM.addParam(dxls[0], dxl_stop_pwm)
        addparam_result = self.groupSyncWritePWM.txPacket()
        # still in PWM mode at this point

        self.groupSyncWritePWM.clearParam()
        dxl_comm_result = self.groupSyncRead.txRxPacket()
        dxl_present_position = self.groupSyncRead.getData(dxls[0], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION) 
        # print("The dynamixel present position is: %d" % dxl_present_position)
        # print("Current platform is: %d" % platform)

        # disable the torque so I can change shit around
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, dxls[0], ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)


        # zeroes out the dynamixel position, CANNOT CHANGE IF TORQUE IS ENABLED
        HOMING_OFFSET_VALUE = -dxl_present_position  
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, dxls[0], ADDR_HOMING_OFFSET, HOMING_OFFSET_VALUE)
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, dxls[0], ADDR_HOMING_OFFSET, 0)

        dxl_comm_result = self.groupSyncRead.txRxPacket()
        dxl_zero_position = self.groupSyncRead.getData(dxls[0], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)  # this should be zero but its not for some reason
        # print("The dxl zero position (first one) is: %d" % dxl_zero_position)
        # print("the new dynamixel position is: %d" % dxl1_zero_position)
        homing_offset = -dxl_zero_position
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, dxls[0], ADDR_HOMING_OFFSET, homing_offset)

        dxl_comm_result = self.groupSyncRead.txRxPacket()
        dxl_zero_position = self.groupSyncRead.getData(dxls[0], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)  # this should be zero
        # print("the new dynamixel position is: %d" % dxl_zero_position)



        # change dynamixel back into continuous operation mode
        dxl_comm_result_operating, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, dxls[0], ADDR_OPERATING_MODE, DXL_OPERATING_MODE)
        # enable torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, dxls[0], ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)

        # MOVE DYNAMIXEL UP A BIT SO THE BUTTON IS NO LONGER PRESSED
        # replace with same PWM condition checking for button push in the future
        move_up_z_tick = -1000
        z_goal_position = [DXL_LOBYTE(DXL_LOWORD(move_up_z_tick)), DXL_HIBYTE(DXL_LOWORD(move_up_z_tick)), DXL_LOBYTE(DXL_HIWORD(move_up_z_tick)), DXL_HIBYTE(DXL_HIWORD(move_up_z_tick))]
        z_addparam_result = self.groupSyncWrite.addParam(dxls[0],z_goal_position)
        dxl_comm_result = self.groupSyncWrite.txPacket()
        sleep(0.5)
        self.groupSyncWrite.clearParam()


        return dxl_zero_position


    def deactivateDynamixels(self, dxls):
        """Deactivates all dynamixels in a list
        
        Args: a single dynamixel ID or a list of them
        """
        for dxl_id in dxls:
            dxl_comm_result_torque, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
            

    # For some reason, setting the homing offset at the end of the movement doesn't work, so I have to set it, zero it,
    # read the position after zeroing it, make it the inverse of that, and then set it again 



    ###################################################################


    # we need to wait for the linear axis to "catch up" with the pitch and yaw if we're going to be moving positions and ish
    # do this by continuously reading and checking present position from linear dynamixels
    # realistically it'll still be pretty snappy for small movements, and most of the work will be done by pitch and yaw


def main(args=None):
    rclpy.init(args=args)
    hashi_node = HashiControl()

    # home platforms
    hashi_node.pwmHoming(0)
    hashi_node.pwmHoming(1)
    sleep(3)
    Z_HOME = 230.0
    
    # Temporary node for centering 
    tmp_node = Node("temp_publisher_command")
    center_pub = tmp_node.create_publisher(Teleop, '/hashi/commands', 10)
    print("center chopsticks")
    center = Teleop()
    center.x_l, center.y_l, center.z_l, center.x_r, center.y_r, center.z_r = 0.0, 0.0, Z_HOME, 0.0, 0.0, Z_HOME
    center_pub.publish(center)
    print("centering done")

    # destory temp node after centering
    tmp_node.destroy_node()

    rclpy.spin(hashi_node)
    # disable all torques for all dynamixels
    hashi_node.deactivateDynamixels(hashi_node.ALL_DXL_IDS)
    

if __name__ == "__main__":
    main()
