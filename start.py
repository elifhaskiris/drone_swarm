#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
© Copyright 2015-2016, 3D Robotics.
guided_set_speed_yaw.py: (Copter Only)

This example shows how to move/direct Copter and send commands in GUIDED mode using DroneKit Python.

Example documentation: http://python.dronekit.io/examples/guided-set-speed-yaw-demo.html
"""
from __future__ import print_function

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import time
import math

udp1="udp:127.0.0.1:14551"
udp2="udp:127.0.0.1:14561"
udp3="udp:127.0.0.1:14571"
udp4="udp:127.0.0.1:14581"
udp5="udp:127.0.0.1:14591"
udp6="udp:127.0.0.1:14601"
udp7="udp:127.0.0.1:14611"
udp8="udp:127.0.0.1:14621"
udp9="udp:127.0.0.1:14631"
udp10="udp:127.0.0.1:14641"

print('Connecting to vehicles')
vehicle = connect(udp1, wait_ready=True)
vehicle2 = connect(udp2, wait_ready=True)
vehicle3 = connect(udp3, wait_ready=True)
vehicle4 = connect(udp4, wait_ready=True)
vehicle5 = connect(udp5, wait_ready=True)
vehicle6 = connect(udp6, wait_ready=True)
vehicle7 = connect(udp7, wait_ready=True)
vehicle8 = connect(udp8, wait_ready=True)
vehicle9 = connect(udp9, wait_ready=True)
vehicle10 = connect(udp10, wait_ready=True)

h=10
d=1


def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle5.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

        
    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    vehicle2.mode = VehicleMode("GUIDED")
    vehicle2.armed = True
    vehicle3.mode = VehicleMode("GUIDED")
    vehicle3.armed = True
    vehicle4.mode = VehicleMode("GUIDED")
    vehicle4.armed = True
    vehicle5.mode = VehicleMode("GUIDED")
    vehicle5.armed = True
    vehicle6.mode = VehicleMode("GUIDED")
    vehicle6.armed = True
    vehicle7.mode = VehicleMode("GUIDED")
    vehicle7.armed = True
    vehicle8.mode = VehicleMode("GUIDED")
    vehicle8.armed = True
    vehicle9.mode = VehicleMode("GUIDED")
    vehicle9.armed = True
    vehicle10.mode = VehicleMode("GUIDED")
    vehicle10.armed = True



    while not vehicle3.armed:      
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude
    vehicle2.simple_takeoff(aTargetAltitude) # Take off to target altitude
    vehicle3.simple_takeoff(aTargetAltitude) # Take off to target altitude
    vehicle4.simple_takeoff(aTargetAltitude) # Take off to target altitude
    vehicle5.simple_takeoff(aTargetAltitude) # Take off to target altitude
    vehicle6.simple_takeoff(aTargetAltitude) # Take off to target altitude
    vehicle7.simple_takeoff(aTargetAltitude) # Take off to target altitude
    vehicle8.simple_takeoff(aTargetAltitude) # Take off to target altitude
    vehicle9.simple_takeoff(aTargetAltitude) # Take off to target altitude
    vehicle10.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle5.location.global_relative_frame.alt)      
        if vehicle5.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)


#Arm and take of to altitude of 5 meters
arm_and_takeoff(h)

def set_roi(location):
    """
    Send MAV_CMD_DO_SET_ROI message to point camera gimbal at a 
    specified region of interest (LocationGlobal).
    The vehicle may also turn to face the ROI.

    For more information see: 
    http://copter.ardupilot.com/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_do_set_roi
    """
    # create the MAV_CMD_DO_SET_ROI command
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_DO_SET_ROI, #command
        0, #confirmation
        0, 0, 0, 0, #params 1-4
        location.lat,
        location.lon,
        location.alt
        )
    # send command to vehicle
    vehicle.send_mavlink(msg)

def goto_position_target_local_ned(veh, north, east, down):
    """	
    Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified 
    location in the North, East, Down frame.

    It is important to remember that in this frame, positive altitudes are entered as negative 
    "Down" values. So if down is "10", this will be 10 metres below the home altitude.

    Starting from AC3.3 the method respects the frame setting. Prior to that the frame was
    ignored. For more information see: 
    http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned

    See the above link for information on the type_mask (0=enable, 1=ignore). 
    At time of writing, acceleration and yaw bits are ignored.

    """
    msg = veh.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111111000, # type_mask (only positions enabled)
        north, east, down, # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
        0, 0, 0, # x, y, z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    # send command to vehicle
    veh.send_mavlink(msg)



print("start position")
DURATION = 20 #Set duration for each segment.

print("Point ROI at current location (home position)") 
# NOTE that this has to be called after the goto command as first "move" command of a particular type
# "resets" ROI/YAW commands
set_roi(vehicle.location.global_relative_frame)
time.sleep(1)
h=-10
print("Point1")
goto_position_target_local_ned(vehicle, 0,-4*d,h)

print("Point2")
goto_position_target_local_ned(vehicle2, 0,-3*d,h)

print("Point3")
goto_position_target_local_ned(vehicle3, 0,-2*d,h)

print("Point4")
goto_position_target_local_ned(vehicle4, 0,-1*d,h)

print("Point5")
goto_position_target_local_ned(vehicle5, 0,0,h)

print("Point6")
goto_position_target_local_ned(vehicle6, -1*d,-4*d,h)

print("Point7")
goto_position_target_local_ned(vehicle7, -1*d,-3*d,h)

print("Point8")
goto_position_target_local_ned(vehicle8, -1*d,-2*d,h)

print("Point9")
goto_position_target_local_ned(vehicle9, -1*d,-1*d,h)

print("Point10")
goto_position_target_local_ned(vehicle10, -1*d,0,h)
time.sleep(DURATION)



"""
The example is completing. LAND at current location.
"""

print("Setting LAND mode...")
vehicle.mode = VehicleMode("LAND")
vehicle2.mode = VehicleMode("LAND")
vehicle3.mode = VehicleMode("LAND")
vehicle4.mode = VehicleMode("LAND")
vehicle5.mode = VehicleMode("LAND")
vehicle6.mode = VehicleMode("LAND")
vehicle7.mode = VehicleMode("LAND")
vehicle8.mode = VehicleMode("LAND")
vehicle9.mode = VehicleMode("LAND")
vehicle10.mode = VehicleMode("LAND")


#Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()
vehicle2.close()
vehicle3.close()
vehicle4.close()
vehicle5.close()
vehicle6.close()
vehicle7.close()
vehicle8.close()
vehicle9.close()
vehicle10.close()


# Shut down simulator if it was started.

print("Completed")
