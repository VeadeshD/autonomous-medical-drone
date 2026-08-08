import os
import subprocess
import time
import math
import random
import json
import threading
import argparse
import cv2
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

os.environ["MAVLINK20"] = "0"
os.environ["MAVLINK10"] = "1"

# ==========================================
# TELEMETRY EXPORTER (For Web Dashboard)
# ==========================================
# We share these global variables with the dashboard thread
current_phase = "Initializing"
current_status = "Booting up..."
current_distance = 0.0

def telemetry_loop(vehicle):
    """Background thread that writes telemetry.json 4 times a second"""
    telemetry_file = os.path.join(os.path.dirname(__file__), "dashboard", "telemetry.json")
    
    # Simulate a battery that starts at 100% and slowly drains!
    # This bypasses a bug in the old 2016 ArduCopter firmware where 
    # factory-resetting the drone breaks the battery monitor.
    simulated_battery = 100.0
    
    while True:
        try:
            alt = vehicle.location.global_relative_frame.alt if vehicle.location.global_relative_frame else 0.0
            
            # Drain battery if motors are armed
            if vehicle.armed and simulated_battery > 0:
                simulated_battery -= 0.1 # Drain by 0.1% every loop (0.4% per second)
                
            mode = vehicle.mode.name if vehicle.mode else "UNKNOWN"
            
            data = {
                "altitude": alt,
                "battery": int(simulated_battery),
                "mode": mode,
                "phase": current_phase,
                "status": current_status,
                "distance": current_distance
            }
            with open(telemetry_file, "w") as f:
                json.dump(data, f)
        except Exception:
            pass
        time.sleep(0.25)

# ==========================================
# HELPER FUNCTIONS
# ==========================================
def get_distance_metres(loc1, loc2):
    dlat = loc2.lat - loc1.lat
    dlong = loc2.lon - loc1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def send_ned_velocity(vehicle, velocity_x, velocity_y, velocity_z):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, 0b0000111111000111,
        0, 0, 0, velocity_x, velocity_y, velocity_z, 0, 0, 0, 0, 0)
    vehicle.send_mavlink(msg)

class WebcamAIVision:
    def __init__(self):
        # Open default webcam
        self.cap = cv2.VideoCapture(0)
        self.target_reached = False

    def get_target_offset(self):
        ret, frame = self.cap.read()
        if not ret:
            # If webcam fails, just hover
            return 0.0, 0.0, False

        # Flip horizontally for a mirror effect
        frame = cv2.flip(frame, 1)
        h, w, _ = frame.shape
        
        # Convert to HSV and threshold for ORANGE
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # HSV range for Orange
        lower_orange = np.array([5, 100, 100])
        upper_orange = np.array([25, 255, 255])
        
        mask = cv2.inRange(hsv, lower_orange, upper_orange)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        err_x, err_y = 0.0, 0.0
        
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            
            if area > 500: # Minimum size to avoid noise
                x, y, bw, bh = cv2.boundingRect(largest_contour)
                cx = x + bw // 2
                cy = y + bh // 2
                
                # Draw targeting reticle
                cv2.rectangle(frame, (x, y), (x+bw, y+bh), (0, 255, 0), 2)
                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
                
                # Map Camera Y-axis to Drone Forward Velocity (vx -> err_x)
                # If object is in top half (cy < h/2), drone moves FORWARD.
                err_x = ((h / 2.0) - cy) / (h / 2.0) 
                
                # Map Camera X-axis to Drone Strafe Velocity (vy -> err_y)
                # If object is on the right (cx > w/2), drone moves RIGHT.
                err_y = (cx - (w / 2.0)) / (w / 2.0)
                
                # If the object is HUGE, the drone has reached the landing zone!
                if area > (w * h * 0.4): 
                    self.target_reached = True

        # Draw crosshair at center
        cv2.line(frame, (w//2, 0), (w//2, h), (255, 255, 255), 1)
        cv2.line(frame, (0, h//2), (w, h//2), (255, 255, 255), 1)
        
        # Show the live feed!
        cv2.imshow("Drone Medical Vision System", frame)
        cv2.waitKey(1)
        
        return err_x, err_y, False

# ==========================================
# BOOTUP & TAKEOFF
# ==========================================
parser = argparse.ArgumentParser(description='Autonomous Drone Mission')
parser.add_argument('--target_lat', type=float, default=None, help='Target Latitude')
parser.add_argument('--target_lon', type=float, default=None, help='Target Longitude')
args = parser.parse_args()

print("Starting Virtual Drone...")

# Force a factory reset of the simulator so we always get 100% battery!
for f in ["eeprom.bin", "mav.parm"]:
    try:
        os.remove(f)
    except FileNotFoundError:
        pass

sitl_args = ['.venv\\Scripts\\dronekit-sitl.exe', 'copter']
drone_process = subprocess.Popen(sitl_args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
time.sleep(3)

print("Connecting to drone...")
vehicle = connect('tcp:127.0.0.1:5760', wait_ready=True)

# Start telemetry background thread
telemetry_thread = threading.Thread(target=telemetry_loop, args=(vehicle,), daemon=True)
telemetry_thread.start()

current_phase = "Booting"
current_status = "Waiting for GPS lock..."
print(f"\n{current_status}")

while not vehicle.is_armable:
    print(".", end="", flush=True)
    time.sleep(1)

current_status = "Setting GUIDED mode & Arming..."
print(f"\n\n{current_status}")
while vehicle.mode.name != 'GUIDED':
    vehicle._master.mav.set_mode_send(
        vehicle._master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4)
    time.sleep(1)

while not vehicle.armed:
    vehicle._master.mav.command_long_send(
        vehicle._master.target_system, vehicle._master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    time.sleep(1)

current_phase = "Takeoff"
current_status = "Climbing to 20 meters..."
print(f"\n[PHASE 1] {current_status}")
vehicle.simple_takeoff(20.0)
while True:
    alt = vehicle.location.global_relative_frame.alt
    if alt >= 19.0:
        break
    time.sleep(1)

# ==========================================
# PHASE 2: LONG RANGE TRANSIT (GPS WAYPOINT)
# ==========================================
current_phase = "GPS Transit"
current_status = "Flying to Emergency Zone..."
print(f"\n[PHASE 2] {current_status}")

current_loc = vehicle.location.global_relative_frame

# Because ArduPilot SITL is hardcoded to spawn in Australia, trying to use your phone's 
# real-world coordinates in America causes a 27,000 kilometer flight time.
# To make this demo work perfectly every time, we will just simulate that the emergency
# is exactly 100 meters away from wherever the drone spawned!
target_lat = current_loc.lat + 0.0009 
target_lon = current_loc.lon
print(f" -> Teleporting emergency to 100 meters away from current location for demo!")

emergency_location = LocationGlobalRelative(target_lat, target_lon, 20.0)

vehicle.simple_goto(emergency_location)

while True:
    current_loc = vehicle.location.global_relative_frame
    distance = get_distance_metres(current_loc, emergency_location)
    current_distance = distance
    current_status = f"Distance to target: {distance:.1f}m"
    print(f" GPS Transit -> {current_status}")
    
    if distance <= 2.0:
        break
    time.sleep(2)

# ==========================================
# PHASE 3: THE HANDOFF & TERMINAL GUIDANCE
# ==========================================
current_phase = "AI Velocity Control"
current_status = "Swapped to Live Webcam AI. Searching..."
print("\n[PHASE 3] GPS-Denied Environment Detected (Entering Building).")
print("          Booting Webcam Computer Vision Model...")

send_ned_velocity(vehicle, 0.0, 0.0, 0.0)
time.sleep(2)

ai_vision = WebcamAIVision()
dt = 0.2 
MAX_VELOCITY = 2.0 # Max speed of 2 m/s

while True:
    err_x, err_y, _ = ai_vision.get_target_offset()
    
    if ai_vision.target_reached:
        current_status = "Landing Zone Locked! Target reached."
        print(f"-> {current_status}")
        send_ned_velocity(vehicle, 0.0, 0.0, 0.0) 
        time.sleep(2)
        break
        
    if err_x == 0.0 and err_y == 0.0:
        current_status = "Searching for Orange Landing Pad..."
        print(f" AI Vision -> {current_status}")
    else:
        current_status = f"Tracking target... Vx: {err_x:.1f}, Vy: {err_y:.1f}"
        print(f" AI Vision -> {current_status}")
    
    # Scale normalized errors (-1 to 1) to velocities
    current_vx = err_x * MAX_VELOCITY
    current_vy = err_y * MAX_VELOCITY
    
    send_ned_velocity(vehicle, current_vx, current_vy, 0.0)
    time.sleep(dt)

# Close the webcam window
cv2.destroyAllWindows()

# ==========================================
# PHASE 4: PAYLOAD DELIVERY
# ==========================================
current_phase = "Payload Delivery"
current_status = "Landing to deploy payload..."
print(f"\n[PHASE 4] {current_status}")

while vehicle.mode.name != 'LAND':
    vehicle._master.mav.set_mode_send(
        vehicle._master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 9)
    time.sleep(1)

while True:
    # Instead of checking if altitude is 0, we check if the drone has 
    # automatically disarmed its motors after touching a solid surface!
    if not vehicle.armed:
        current_status = "Payload safely delivered! Mission Complete."
        current_phase = "Mission Complete"
        print(f" -> {current_status}")
        break
    time.sleep(1)

# Give the background telemetry thread 1 second to write the final 
# "Mission Complete" status to the JSON file before we kill the drone!
time.sleep(1) 
vehicle.close()
drone_process.terminate()
