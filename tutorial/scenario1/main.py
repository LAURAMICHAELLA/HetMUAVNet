# DroneKit-Python program based on the "Simple Go To (Copter)" example.
# Time is synchronized to simulation clock through the "simtime" library.

import helper
import math
import ns3interface
import simtime
import struct
import sys
import time

# pip install --user dronekit
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

# Synchronize time.time() and time.sleep(n) with simulation clock
simtime_port = int(sys.argv[1])
simtime.connect(simtime_port)

# Parse other commandline arguments
uav_name, mavlink_sysid, mavlink_port = sys.argv[2].split(':')
mavlink_sysid = int(mavlink_sysid)
mavlink_port = int(mavlink_port)

# Connect to the ns3 network simulator
ns3interface.connect('127.0.0.1', mavlink_sysid - 1)

# Connect to the Vehicle
vehicle = connect(
    'tcp:127.0.0.1:{}'.format(mavlink_port),
    source_system=mavlink_sysid + 100)

# ArduCopter initialisation can take a really long time
vehicle.wait_ready('gps_0', 'armed', 'mode', 'attitude', timeout=100)

# Don't try to arm until autopilot is ready
while not vehicle.is_armable:
    print(" Waiting for vehicle to initialise...")
    time.sleep(5)

print("Arming motors")
# Copter should arm in GUIDED mode
vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True


# Confirm vehicle armed before attempting to take off
while not vehicle.armed:
    print(" Waiting for arming...")
    time.sleep(1)

print("Taking off!")
target_altitude = 5
ShortestDistance = 2
UavArrived1 = False
UavArrived2 = False
vehicle.simple_takeoff(target_altitude)  # Take off to target altitude
time.sleep(10)


# uavs behavior ---------

# base station

# We will store the position and heading of each UAV (except base)  as well as
# the absolute timestamp we receive each piece of information
uav_positions = dict()

if mavlink_sysid == 1:
    # We are the "reference" vehicle; we just hover a fixed point and
    # continuosly broadcast our position and a sequence number using two nodes
    seqnum = 1
    while True:
        lat = vehicle.location.global_relative_frame.lat
        lon = vehicle.location.global_relative_frame.lon
        vehicle.simple_goto(LocationGlobalRelative(lat, lon, target_altitude))
        ns3interface.sendto(struct.pack("<Idd", seqnum, lat, lon), 1) # for ns-3 1 is equals to node 2 becaise the base station is node 0
        ns3interface.sendto(struct.pack("<Idd", seqnum, lat, lon), 2) # for ns-3 2 is equals to node 3
#        ns3interface.sendto(struct.pack("<Idd", seqnum, lat, lon), 3) # for ns-3 2 is equals to node 3
        seqnum = seqnum + 1
#        print ('Ground Control Station sended seqnum={} messages'.format(seqnum))
        time.sleep(.5)

elif mavlink_sysid == 2:

    while True:

        lat = vehicle.location.global_relative_frame.lat
        lon = vehicle.location.global_relative_frame.lon
        startMissionPoint = LocationGlobalRelative(lat, lon, target_altitude)
        MissionPoint = LocationGlobalRelative(-27.604122, -48.518019,target_altitude)

        currentPosition = vehicle.location.global_relative_frame
        distanceOfBegin = helper.get_distance_metres(startMissionPoint, currentPosition)

        while (distanceOfBegin > ShortestDistance):
            currentPosition = vehicle.location.global_relative_frame
            distanceOfBegin = helper.get_distance_metres(startMissionPoint, currentPosition)
            time.sleep(.1)

        distanceOfEnd = helper.get_distance_metres(MissionPoint, currentPosition)

        while (distanceOfEnd > ShortestDistance):
            distanceOfEnd = helper.get_distance_metres(MissionPoint, currentPosition)
            currentPosition = vehicle.location.global_relative_frame
            vehicle.simple_goto(MissionPoint)
            time.sleep(.1)
        else:
            vehicle.mode = VehicleMode("LAND")
            UavArrived1 = True

        # Process incoming messages
        while ns3interface.message_available():
            payload, sender = ns3interface.recvfrom()
            seqnum, lat, lon = struct.unpack("<Idd", payload)
            distance = helper.get_distance_metres(
                LocationGlobalRelative(lat, lon),
                vehicle.location.global_relative_frame
            )
            print('The misson vehicle 1 received a message of base: seqnum={} distance={} meters'.format(seqnum, distance))
        time.sleep(.1)

elif mavlink_sysid == 3:
    #position in map in relation to base (-1, -1,5)


    while True:
        lat = vehicle.location.global_relative_frame.lat
        lon = vehicle.location.global_relative_frame.lon
        startMissionPoint = LocationGlobalRelative(lat, lon, target_altitude)
        MissionPoint = LocationGlobalRelative(-27.604034, -48.518743, target_altitude)

        currentPosition = vehicle.location.global_relative_frame
        distanceOfBegin = helper.get_distance_metres(startMissionPoint, currentPosition)

        while (distanceOfBegin > ShortestDistance):
            currentPosition = vehicle.location.global_relative_frame
            distanceOfBegin = helper.get_distance_metres(startMissionPoint, currentPosition)
            time.sleep(.1)

        distanceOfEnd = helper.get_distance_metres(MissionPoint, currentPosition)

        while (distanceOfEnd > ShortestDistance):
            distanceOfEnd = helper.get_distance_metres(MissionPoint, currentPosition)
            currentPosition = vehicle.location.global_relative_frame
            vehicle.simple_goto(MissionPoint)
            time.sleep(.1)
        else:
            vehicle.mode = VehicleMode("LAND")
            UavArrived2 = True

        # Process incoming messages
        while ns3interface.message_available():
            payload, sender = ns3interface.recvfrom()
            seqnum, lat, lon = struct.unpack("<Idd", payload)
            distance = helper.get_distance_metres(
                LocationGlobalRelative(lat, lon),
                vehicle.location.global_relative_frame
            )
            print('The mission vehicle 2 received a message of base: seqnum={} distance={} meters'.format(seqnum, distance))
            time.sleep(.1)
else:
    if (UavArrived1==True and UavArrived2==True):
        vehicle.mode = VehicleMode("LAND")
        print('SCENARIO1 ENDED')
        vehicle.close()
        sys.exit()

# This line cannot be reached in this example
#vehicle.close()
