import os
import subprocess
import time
from dronekit import connect, VehicleMode
from pymavlink import mavutil

os.environ["MAVLINK20"] = "0"
os.environ["MAVLINK10"] = "1"

def send_ned_velocity(vehicle, velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors and duration.
    velocity_x: Forward (+) / Backward (-) in m/s
    velocity_y: Right (+) / Left (-) in m/s
    velocity_z: Down (+) / Up (-) in m/s
    """
    print(f"Sending velocity command: X:{velocity_x}m/s, Y:{velocity_y}m/s, Z:{velocity_z}m/s for {duration} seconds")
    
    # Create the MAVLink message to set the velocity
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # Frame of reference (relative to drone's current heading)
        0b0000111111000111, # Bitmask indicating we only want to control VELOCITY
        0, 0, 0, # x, y, z positions (ignored)
        velocity_x, velocity_y, velocity_z, # x, y, z velocities in m/s
        0, 0, 0, # x, y, z acceleration (ignored)
        0, 0)    # yaw, yaw_rate (ignored)

    # We must send the velocity command every 1 second, otherwise the drone stops and hovers safely!
    for _ in range(duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)

print("Starting Virtual Drone...")
drone_process = subprocess.Popen(['.venv\\Scripts\\dronekit-sitl.exe', 'copter'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
time.sleep(3)

print("Connecting to drone...")
vehicle = connect('tcp:127.0.0.1:5760', wait_ready=True)

print("\nWaiting for GPS lock (required for GUIDED mode)...")
while not vehicle.is_armable:
    print(".", end="", flush=True)
    time.sleep(1)

print("\n\nSetting mode to GUIDED (using raw MAVLink to bypass old firmware bugs)...")
while vehicle.mode.name != 'GUIDED':
    vehicle._master.mav.set_mode_send(
        vehicle._master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        4 # MAVLink ID for GUIDED mode
    )
    time.sleep(1)

print("Arming motors...")
while not vehicle.armed:
    vehicle._master.mav.command_long_send(
        vehicle._master.target_system, vehicle._master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0 # The '1' means ARM
    )
    time.sleep(1)

# 1. TAKEOFF
target_altitude = 5.0 # meters
print(f"\n[PHASE 1] Taking off to {target_altitude} meters...")
vehicle.simple_takeoff(target_altitude)

while True:
    alt = vehicle.location.global_relative_frame.alt
    print(f" Altitude: {alt:.2f}m")
    if alt >= target_altitude * 0.95:
        print(" -> Takeoff complete!")
        break
    time.sleep(1)

# 2. FLY FORWARD
print("\n[PHASE 2] Flying FORWARD at 2 m/s for 5 seconds...")
# velocity_x = 2.0 (forward), velocity_y = 0.0, velocity_z = 0.0
send_ned_velocity(vehicle, 2.0, 0.0, 0.0, 5)

# 3. HOVER
print("\n[PHASE 3] Braking and hovering in place for 3 seconds...")
# Setting all velocities to 0 stops the drone
send_ned_velocity(vehicle, 0.0, 0.0, 0.0, 3)

# 4. FLY BACKWARD
print("\n[PHASE 4] Flying BACKWARD at 2 m/s for 5 seconds...")
# velocity_x = -2.0 (backward)
send_ned_velocity(vehicle, -2.0, 0.0, 0.0, 5)

# 5. LAND
print("\n[PHASE 5] Landing the drone...")
while vehicle.mode.name != 'LAND':
    vehicle._master.mav.set_mode_send(
        vehicle._master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        9 # MAVLink ID for LAND mode in ArduCopter
    )
    time.sleep(1)

while True:
    alt = vehicle.location.global_relative_frame.alt
    print(f" Altitude: {alt:.2f}m")
    if alt <= 0.2: # Less than 20cm means it landed
        print(" -> Drone has safely landed!")
        break
    time.sleep(1)

print("\nMission Accomplished! Disconnecting...")
vehicle.close()
drone_process.terminate()
