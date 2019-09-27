# DroneKit-Python program based on the "Simple Go To (Copter)" example.
# Time is synchronized to simulation clock through the "simtime" library.

import helper
import math
import ns3interface
import os
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
ns3interface.connect(os.getenv('NS3_INTERFACE', '127.0.0.1'), mavlink_sysid - 1)

# Connect to the Vehicle
vehicle = connect(
    'tcp:127.0.0.1:{}'.format(mavlink_port),
    source_system=mavlink_sysid + 100,
    rate=11) # request 11 Hz updates from ArduCopter (default is 4 Hz)

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
vehicle.simple_takeoff(target_altitude)  # Take off to target altitude
time.sleep(10)

# We will store the position and heading of each UAV in the flock as well as
# the absolute timestamp we receive each piece of information
uav_positions = dict()

# This is our control loop (10 Hz)
while True:
    time.sleep(1 / 10.0)

    # Update this node's position in the uav_positions dictionary
    uav_positions[ns3interface.local_id()] = (
        vehicle.location.global_relative_frame.lat,
        vehicle.location.global_relative_frame.lon,
        vehicle.location.global_relative_frame.alt,
        vehicle.heading,
        time.time()
    )

    # Broadcast it to the other agents
    ns3interface.sendto(struct.pack("<dddHd",
        uav_positions[ns3interface.local_id()][0], # lat
        uav_positions[ns3interface.local_id()][1], # lon
        uav_positions[ns3interface.local_id()][2], # alt
        uav_positions[ns3interface.local_id()][3], # heading
        uav_positions[ns3interface.local_id()][4]  # timestamp
    ), ns3interface.BROADCAST)

    # Process incoming messages
    while ns3interface.message_available():
        payload, sender = ns3interface.recvfrom()
        uav_positions[sender] = struct.unpack("<dddHd", payload)

    # Delete entries that have not been updated for more than one second
    for uav_id in list(uav_positions.keys()):
        if uav_positions[uav_id][4] + 1.0 < time.time():
            del uav_positions[uav_id]

    # Print list of current uav_positions entries
    print('[{}] UAV {} currently knows about {}'.format(
        time.time(),
        ns3interface.local_id(),
        ', '.join(map(str, sorted(uav_positions)))
    ))

    # Calculate flock's average position (lat and lon only)
    lat_avg = lon_avg = 0
    for lat, lon, alt, heading, timestamp in uav_positions.values():
        lat_avg = lat_avg + lat
        lon_avg = lon_avg + lon
    lat_avg = lat_avg / len(uav_positions)
    lon_avg = lon_avg / len(uav_positions)

    # Calculate flock's avarage heading (actually the mean versor)
    ver_x = ver_y = 0
    for lat, lon, alt, heading, timestamp in uav_positions.values():
        ver_x = ver_x + math.cos(heading * math.pi / 180)
        ver_y = ver_y + math.sin(heading * math.pi / 180)
    if abs(ver_x) < .001 and abs(ver_y) < .001:
        heading_avg = vehicle.heading
    else:
        # Calculate heading using atan2 and convert it to [0, 359]
        heading_avg = int(math.atan2(ver_y, ver_x) * 180 / math.pi + 360) % 360

    # Find the nearest agent's heading
    nearest_uav_distance = None
    nearest_uav_location = None
    for uav_id, (lat, lon, alt, heading, timestamp) in uav_positions.items():
        if uav_id != ns3interface.local_id():
            # Calculate distance between us and the other UAV
            distance = helper.get_distance_metres(
                vehicle.location.global_relative_frame,
                LocationGlobalRelative(lat, lon, alt)
            )
            # Find minimum
            if (nearest_uav_distance is None) or nearest_uav_distance > distance:
                nearest_uav_distance = distance
                nearest_uav_location = LocationGlobalRelative(lat, lon, alt)

    # Apply flocking rules
    if (nearest_uav_distance is None) or nearest_uav_distance > 3:
        # Alignment and cohesion
        alignment_speed = helper.heading_diff(
            heading_avg,
            vehicle.heading
        )
        cohesion_speed = helper.heading_diff(
            helper.get_bearing( # direction towards flock center
                vehicle.location.global_relative_frame,
                LocationGlobalRelative(lat_avg, lon_avg, target_altitude)
            ),
            vehicle.heading
        )
        target_yawrate = alignment_speed * .01 + cohesion_speed * .02
    else:
        # Separation
        separation_speed = helper.heading_diff(
            helper.get_bearing( # direction towards the nearest UAV
                nearest_uav_location,
                vehicle.location.global_relative_frame
            ),
            vehicle.heading
        )
        target_yawrate = separation_speed * .025

    # A simple proportional controller for vertical speed (m/s)
    vert_speed = (vehicle.location.global_relative_frame.alt - target_altitude) * .1

    # Horizontal speed is constant (m/s)
    horizontal_speed = .3

    vehicle.send_mavlink(vehicle.message_factory.set_position_target_local_ned_encode(
        0, # time_boot_ms (ignored)
        0, 0, # target system, target component (this is filled by DroneKit)
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame
        0b0000011111000111, # type_mask (enables x, y, z speeds and yaw rate)
        0, # lat_int (ignored)
        0, # lon_int (ignored)
        0, # alt (ignored)
        horizontal_speed, # X velocity (forward, in m)
        0, # Y velocity (right, in m)
        vert_speed, # Z velocity (down, in m)
        0, 0, 0, # afx, afy, afz acceleration (ignored)
        0, # yaw (ignored)
        target_yawrate # yawrate (in rad/s)
    ))

# This line cannot be reached in this example
vehicle.close()
