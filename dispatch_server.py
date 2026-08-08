from flask import Flask, request, jsonify, send_from_directory
import subprocess
import os

app = Flask(__name__, static_folder='mobile')

# Global variable to keep track of the drone process
current_drone_mission = None

@app.route('/')
def serve_mobile_app():
    """Serve the Mobile App UI to the phone"""
    return send_from_directory('mobile', 'index.html')

@app.route('/dispatch', methods=['POST'])
def dispatch_drone():
    """API Endpoint that receives the SOS signal from the phone"""
    global current_drone_mission
    
    data = request.json
    lat = data.get('lat')
    lon = data.get('lon')
    
    if lat is None or lon is None:
        return jsonify({"error": "Missing GPS coordinates"}), 400
        
    print(f"\n[🚨 EMERGENCY DISPATCH 🚨]")
    print(f"Target GPS Locked: {lat}, {lon}")
    print(f"Launching Drone...\n")
    
    # If a mission is already running, we should kill it first or reject the request
    # For now, let's just launch a new one (assuming only 1 emergency at a time)
    if current_drone_mission is not None and current_drone_mission.poll() is None:
        return jsonify({"error": "A drone is already in flight!"}), 400
        
    # Spawn the hybrid mission in the background, passing the phone's GPS coordinates!
    current_drone_mission = subprocess.Popen(
        ['.venv\\Scripts\\python.exe', 'hybrid_mission.py', '--target_lat', str(lat), '--target_lon', str(lon)]
    )
    
    return jsonify({"status": "Success", "message": "Drone dispatched"}), 200

@app.route('/status', methods=['GET'])
def check_status():
    """API Endpoint for the mobile app to check if the drone has arrived"""
    global current_drone_mission
    
    if current_drone_mission is None:
        return jsonify({"status": "IDLE"}), 200
        
    # If the process has terminated (poll() is not None), the mission is complete!
    if current_drone_mission.poll() is not None:
        return jsonify({"status": "COMPLETE"}), 200
        
    return jsonify({"status": "INBOUND"}), 200

if __name__ == '__main__':
    print("="*50)
    print("DISPATCH SERVER RUNNING")
    print("Connect your phone to your laptop's IP address on port 5000!")
    print("="*50)
    # Run on 0.0.0.0 so the phone can access it over the local Wi-Fi network!
    app.run(host='0.0.0.0', port=5000)
