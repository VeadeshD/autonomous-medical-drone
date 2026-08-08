# 🚁 Autonomous Emergency Medical Drone System

Welcome to the software repository for my Autonomous Emergency Medical Drone System! I built this software infrastructure as a Proof of Concept (PoC) to demonstrate how autonomous robotics, computer vision, and mobile web technologies can be combined to revolutionize emergency medical response.

## 🚀 Project Overview

The goal of this project is to create an end-to-end system where a user can instantly summon a medical drone (carrying an EpiPen, defibrillator, or antivenom) to their physical GPS location using their phone. Once dispatched, the drone flies autonomously, and then uses an onboard camera to visually track a landing pad for a precise payload delivery in GPS-denied environments.

Here is a breakdown of the core systems I engineered:

### 1. Flight Control Systems & Physics Simulation
I wrote a robust Python mission script utilizing the `dronekit` library to interface with the MAVLink protocol. 
* **ArduCopter SITL Integration:** To test my flight code safely, I integrated a physics-accurate virtual drone environment (SITL).
* **Autonomous Navigation:** I programmed the drone to automatically take off, climb to a safe altitude, and navigate via GPS waypoints generated dynamically by the user's phone.

### 2. Advanced Computer Vision (OpenCV)
I designed a live computer vision pipeline to handle terminal guidance (the final phase of landing).
* **Live Video Feed:** The mission script captures a live camera feed.
* **Color Tracking:** I utilized the HSV color space and contour detection to lock onto a specific colored object representing the medical landing zone.
* **Closed-Loop Velocity Control:** I translated the tracked object's bounding box coordinates into X and Y velocity vectors, allowing the drone to physically strafe and follow the landing pad before initiating a terminal landing.

### 3. Mission Control Dashboard
I built a real-time command center for monitoring the drone!
* **Live Telemetry:** The dashboard actively polls a telemetry JSON file at 4Hz to provide live updates on the drone's altitude, battery drain, and flight phase.
* **Premium UI/UX:** I used advanced CSS techniques (Glassmorphism, CSS Variables, Flexbox) to create a sleek, modern, dark-mode interface.

### 4. Mobile Dispatch API
To bridge the gap between the drone and the user, I built a fully functioning web server using Flask.
* **Geolocation:** The mobile web app securely accesses the user's physical GPS hardware to acquire real-world coordinates.
* **Subprocess Management:** When the SOS button is tapped, the Flask server dynamically spins up a new instance of the drone's flight code in the background, passing the precise coordinates directly into the mission logic!
* **Live Status Polling:** The mobile app actively asks the server for the drone's flight status, automatically transitioning the UI to a success state the moment the payload is delivered.

## 🔧 Engineering Challenges Overcome
Throughout this project, I tackled several difficult engineering challenges, including:
* **Planetary Transit Calculations:** Fixing distance calculation bugs by simulating a 100-meter local offset when the virtual drone (spawned in Australia) tried to fly to my physical phone in America.
* **Firmware Bypass:** Identifying and bypassing legacy bugs in ArduCopter 3.3 by writing "hammer" loops to force MAVLink mode changes.
* **Battery Drain Simulation:** Writing a custom battery drain simulator to bypass a corrupted EEPROM bug in the firmware.

## 📈 Future Research (Hardware Phase)
With both the physical airframe designed and this software simulation pipeline 100% completed and proven, my next phase is deploying this code onto a physical Pixhawk/Raspberry Pi drone frame. This will allow me to research real-world flight latency, physical payload drop mechanics, and computer vision reliability in the field!
