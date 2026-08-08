import os
import subprocess
import time
from pymavlink import mavutil
import sys

# DroneKit SITL (ArduCopter 3.3.0) requires MAVLink 1 to avoid crashing on connect
os.environ["MAVLINK20"] = "0"
os.environ["MAVLINK10"] = "1"

print("Starting Virtual Drone natively on Windows...")
drone_process = subprocess.Popen(['.venv\\Scripts\\dronekit-sitl.exe', 'copter'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
time.sleep(3)

print("Connecting to drone...")
vehicle = mavutil.mavlink_connection('tcp:127.0.0.1:5760')

print("Sending wake-up heartbeat...")
vehicle.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
vehicle.wait_heartbeat(timeout=10)
print(f"Connected successfully to Drone (System ID: {vehicle.target_system})!")

vehicle.mav.request_data_stream_send(
    vehicle.target_system, vehicle.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL, 
    2, # 2 Hz
    1  # Start sending
)

# ======= AUTONOMOUS FLIGHT COMMANDS =======
print("Setting mode to GUIDED...")
vehicle.mav.set_mode_send(
    vehicle.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4
)
time.sleep(2)

print("Arming motors...")
vehicle.mav.command_long_send(
    vehicle.target_system, vehicle.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 1, 0, 0, 0, 0, 0, 0
)
time.sleep(2)

target_altitude = 10.0 # meters
print(f"Taking off to {target_altitude} meters...")
vehicle.mav.command_long_send(
    vehicle.target_system, vehicle.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0, 0, 0, 0, 0, 0, 0, target_altitude
)

# ======= MONITOR ALTITUDE =======
print("Monitoring altitude... (Press Ctrl+C to stop)")
start_altitude = None

try:
    while True:
        vehicle.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        
        msg = vehicle.recv_match(type=['GLOBAL_POSITION_INT', 'VFR_HUD'], blocking=True, timeout=2.0)
        if msg:
            if msg.get_type() == 'GLOBAL_POSITION_INT':
                alt_meters = msg.relative_alt / 1000.0
            else:
                # If using VFR_HUD, we must subtract the starting runway altitude (~584m)
                if start_altitude is None:
                    start_altitude = msg.alt
                alt_meters = msg.alt - start_altitude
                
            print(f"Current Altitude: {alt_meters:.2f} meters")
            
            if alt_meters >= target_altitude * 0.95:
                print("Reached target altitude!")
                break
except KeyboardInterrupt:
    print("\nDisconnecting...")
    
vehicle.close()
drone_process.terminate()
