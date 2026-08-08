import os
import subprocess
import time
import math
import random
from dronekit import connect, VehicleMode
from pymavlink import mavutil

os.environ["MAVLINK20"] = "0"
os.environ["MAVLINK10"] = "1"

# ==========================================
# 1. SIMULATED AI VISION SYSTEM
# ==========================================
class SimulatedAIVision:
    def __init__(self, target_x, target_y):
        # Imagine the victim is 15 meters forward and 8 meters to the right
        self.target_x = target_x 
        self.target_y = target_y
        self.drone_x = 0.0
        self.drone_y = 0.0
        
    def get_target_offset(self, velocity_x, velocity_y, dt):
        # Update drone's virtual position based on its velocity and time passed
        self.drone_x += velocity_x * dt
        self.drone_y += velocity_y * dt
        
        # Calculate how far away the target is from the drone right now
        offset_x = self.target_x - self.drone_x
        offset_y = self.target_y - self.drone_y
        
        # Add a tiny bit of random noise (because real cameras are never 100% perfect!)
        offset_x += random.uniform(-0.2, 0.2)
        offset_y += random.uniform(-0.2, 0.2)
        
        return offset_x, offset_y

def send_ned_velocity(vehicle, velocity_x, velocity_y, velocity_z):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, 0b0000111111000111,
        0, 0, 0, velocity_x, velocity_y, velocity_z, 0, 0, 0, 0, 0)
    vehicle.send_mavlink(msg)

# ==========================================
# 2. BOOTUP & TAKEOFF
# ==========================================
print("Starting Virtual Drone...")
drone_process = subprocess.Popen(['.venv\\Scripts\\dronekit-sitl.exe', 'copter'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
time.sleep(3)

print("Connecting to drone...")
vehicle = connect('tcp:127.0.0.1:5760', wait_ready=True)

print("\nWaiting for GPS lock...")
while not vehicle.is_armable:
    print(".", end="", flush=True)
    time.sleep(1)

print("\n\nSetting mode to GUIDED...")
while vehicle.mode.name != 'GUIDED':
    vehicle._master.mav.set_mode_send(
        vehicle._master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4)
    time.sleep(1)

print("Arming motors...")
while not vehicle.armed:
    vehicle._master.mav.command_long_send(
        vehicle._master.target_system, vehicle._master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    time.sleep(1)

print("\n[PHASE 1] Taking off to 5.0 meters...")
vehicle.simple_takeoff(5.0)
while True:
    alt = vehicle.location.global_relative_frame.alt
    if alt >= 4.75:
        break
    time.sleep(1)

# ==========================================
# 3. CLOSED-LOOP AI TRACKING (THE BRAIN)
# ==========================================
print("\n[PHASE 2] Starting AI Vision Tracking...")
ai_vision = SimulatedAIVision(target_x=15.0, target_y=8.0)
dt = 0.5 # We will process a camera frame every 0.5 seconds
P_GAIN = 0.3 # Proportional Gain (How aggressively to fly towards the target)

current_vx, current_vy = 0.0, 0.0

while True:
    # 1. Ask the AI: "Where is the victim?"
    err_x, err_y = ai_vision.get_target_offset(current_vx, current_vy, dt)
    distance = math.sqrt(err_x**2 + err_y**2)
    
    print(f"AI Vision -> Target is {err_x:.1f}m Forward, {err_y:.1f}m Right (Total Distance: {distance:.1f}m)")
    
    # 2. Are we close enough to drop the payload?
    if distance < 1.0: # Within 1 meter of the victim!
        print("-> Target successfully reached! Hovering over victim!")
        send_ned_velocity(vehicle, 0.0, 0.0, 0.0) # Hit the brakes
        time.sleep(2)
        break
        
    # 3. Calculate new velocities using Proportional Control (P-Controller)
    # The further away the drone is, the faster it flies. As it gets closer, it slows down smoothly!
    current_vx = err_x * P_GAIN
    current_vy = err_y * P_GAIN
    
    # Cap maximum speed for safety (don't fly faster than 5 m/s indoors)
    current_vx = max(min(current_vx, 5.0), -5.0)
    current_vy = max(min(current_vy, 5.0), -5.0)
    
    # 4. Send the velocity to the drone
    send_ned_velocity(vehicle, current_vx, current_vy, 0.0)
    
    time.sleep(dt)

# ==========================================
# 4. PAYLOAD DELIVERY (LANDING)
# ==========================================
print("\n[PHASE 3] Initiating Rescue Landing...")
while vehicle.mode.name != 'LAND':
    vehicle._master.mav.set_mode_send(
        vehicle._master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 9)
    time.sleep(1)

while True:
    if vehicle.location.global_relative_frame.alt <= 0.2:
        print(" -> Drone has safely delivered the payload!")
        break
    time.sleep(1)

vehicle.close()
drone_process.terminate()
