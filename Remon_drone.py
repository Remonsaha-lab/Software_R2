import websockets
import asyncio
import json
import time
import re
import random
import math 
import os
import sys
import socket
from datetime import datetime

project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

from drone_simulator.logging_config import get_logger

logger = get_logger(__name__)


async def perform_pre_flight_safety_check(websocket, controller):
    """Immediate safety check to ensure we're at a safe altitude with current sensor status"""
    logger.info("Performing pre-flight safety check...")
    
   
    status_command = {
        'speed': 1,  
        'altitude': 0,  
        'movement': 'fwd' 
    }
    
    await websocket.send(json.dumps(status_command))
    status_response = await asyncio.wait_for(websocket.recv(), timeout=2.0)
    status_data = json.loads(status_response)
    controller.update_with_response(status_data)
    
    logger.info(f"Initial safety check: sensor={controller.telemetry.sensor_status}, altitude={controller.telemetry.y_position}")
    
    
    if controller.telemetry.y_position == 0:
       
        logger.info("Starting at ground level, ascending safely")
        
       
        ascend_command = {
            'speed': 1,
            'altitude': 1,  
            'movement': 'fwd'
        }
        
        await websocket.send(json.dumps(ascend_command))
        ascend_response = await asyncio.wait_for(websocket.recv(), timeout=2.0)
        ascend_data = json.loads(ascend_response)
        controller.update_with_response(ascend_data)
        
        logger.info(f"After initial ascent: altitude={controller.telemetry.y_position}")
   
    if controller.telemetry.sensor_status == "RED":
       
        max_safe_altitude = 2 
        if controller.telemetry.y_position > max_safe_altitude:
            logger.warning(f"Starting at unsafe altitude {controller.telemetry.y_position} with RED sensor. Descending to {max_safe_altitude}")
            
            emergency_command = {
                'speed': 1,
                'altitude': -1,  
                'movement': 'fwd'
            }
            
            await websocket.send(json.dumps(emergency_command))
            await asyncio.wait_for(websocket.recv(), timeout=2.0)
            
            
            controller.target_altitude = max_safe_altitude
            controller.phase = "RED_CRUISE"
    
    return controller.telemetry.sensor_status, controller.telemetry.y_position


class TelemetryParser:
    def __init__(self):  
        self.x_position = 0
        self.y_position = 0
        self.battery = 100
        self.gyroscope = [0, 0, 0]
        self.wind_speed = 0
        self.dust_level = 0
        self.sensor_status = "GREEN"
        self.status = "success"
        self.iterations = 0
        self.total_distance = 0
        self.last_update_time = time.time()
        self.history = []
        self.crash_reason = None
        self.last_ping_time = time.time()
        self.ping_interval = 5 
        self.ping_timeout = 3   

    def parse_telemetry(self, telemetry_str):
        """Parse telemetry string from the server response."""
        if not telemetry_str or not isinstance(telemetry_str, str):
            logger.error(f"Invalid telemetry data: {telemetry_str}")
            return False
        
        pattern = r"X-(-?\d+)-Y-(-?\d+)-BAT-(\d+\.?\d*)-GYR-\[(-?\d+\.?\d*),\s*(-?\d+\.?\d*),\s*(-?\d+\.?\d*)\]-WIND-(\d+\.?\d*)-DUST-(\d+\.?\d*)-SENS-(\w+)"
        match = re.search(pattern, telemetry_str)

        if match:
            prev_x = self.x_position
            prev_y = self.y_position
            prev_battery = self.battery

            self.x_position = int(match.group(1))
            self.y_position = int(match.group(2))
            self.battery = float(match.group(3))  
            self.gyroscope = [float(match.group(4)), float(match.group(5)), float(match.group(6))]
            self.wind_speed = float(match.group(7))  
            self.dust_level = float(match.group(8))  
            self.sensor_status = match.group(9)

           
            distance_moved = math.sqrt((self.x_position - prev_x)**2 + (self.y_position - prev_y)**2)
            battery_drain = prev_battery - self.battery

           
            self.check_crash_conditions()

            self.history.append({
                "x_position": self.x_position,
                "y_position": self.y_position,
                "battery_status": battery_drain,
                "battery_level": self.battery,
                "gyroscope": self.gyroscope.copy(),
                "wind_speed": self.wind_speed,
                "dust_level": self.dust_level,
                "sensor_status": self.sensor_status,
                "distance_moved": distance_moved,
                "timestamp": time.time()
            })

            if len(self.history) > 30:  
                self.history.pop(0)

            logger.debug(f"Parsed telemetry: x={self.x_position}, y={self.y_position}, bat={self.battery:.2f}%, "
                         f"gyro={self.gyroscope}, wind={self.wind_speed:.1f}, dust={self.dust_level:.1f}, "
                         f"sensor={self.sensor_status}")

            return True

        else:
            logger.error(f"Failed to parse telemetry: {telemetry_str}")
            return False
    
    def check_crash_conditions(self):
        """Check for potential crash conditions based on telemetry data"""
        tilt_angle = self.get_tilt_angle()
        
        if self.battery <= 5:  
            self.crash_reason = "Battery Depletion"
            logger.warning(f"Critical battery level: {self.battery}%")
        
        elif self.y_position < 0.5: 
            self.crash_reason = "Ground Collision Risk"
            logger.warning(f"Near ground collision: altitude = {self.y_position}")
        
        elif self.sensor_status == "RED" and self.y_position >= 2.5:  
            self.crash_reason = "Approaching Unsafe Altitude (RED sensor)"
            logger.warning(f"DANGER: Approaching unsafe altitude with RED sensor: {self.y_position}/3.0")
        
        elif self.sensor_status == "YELLOW" and self.y_position >= 950:  
            self.crash_reason = "Approaching Unsafe Altitude (YELLOW sensor)"
            logger.warning(f"DANGER: Approaching unsafe altitude with YELLOW sensor: {self.y_position}/1000")
        
        elif tilt_angle >= 40: 
            self.crash_reason = "High Tilt Angle"
            logger.warning(f"DANGER: High tilt angle: {tilt_angle}/45 degrees")
        
        else:
            self.crash_reason = None

    def update_metrices(self, metrics):
        self.iterations = metrics.get("iterations", self.iterations)
        self.total_distance = metrics.get("total_distance", self.total_distance)
        logger.info(f"Updated metrics: iterations={self.iterations}, distance={self.total_distance:.2f}")

    def get_tilt_angle(self):
        """Calculate the tilt angle from the gyroscope data"""
        x, y, z = self.gyroscope
        
        
        tilt_magnitude = math.sqrt(x**2 + y**2 + z**2)
        
        
        tilt_angle = min(45, tilt_magnitude * 45)
        return tilt_angle

    def get_battery_drain(self):
        """Calculate average battery drain rate based on recent history"""
        if len(self.history) < 2:
            return 0
        
       
        drains = [entry['battery_status'] for entry in self.history[-10:] if 'battery_status' in entry]
        return sum(drains) / len(drains) if drains else 0
    
    def get_altitude_factor(self):
        """Calculate altitude-based battery drain factor
        - Ground level (y near 0): up to 1.8x faster drain
        - High altitude: down to 0.6x slower drain
        """
        if self.y_position < 10:
           
            return 1.8 - (self.y_position / 10) * 0.3
        elif self.y_position < 100:
            
            return 1.5 - (self.y_position / 100) * 0.5
        else:
          
            return max(0.6, 1.0 - math.log10(self.y_position/100) * 0.2)
    
    def get_safe_altitude(self):
        """Determine safe altitude based on sensor status"""
        if self.sensor_status == "RED":
            return 2  
        elif self.sensor_status == "YELLOW":
            return 800  
        else:  # GREEN
            if self.battery < 30:
                return 1000  
            else:
                return 1500  

class DroneController:
    def __init__(self): 
        self.telemetry = TelemetryParser()  
        self.last_command = {'speed': 1, 'altitude': -1, 'movement': 'fwd'}
        self.stuck_count = 0
        self.phase = "TAKEOFF"
        self.target_altitude = 2  
        self.direction_changes = 0
        self.last_direction_change = 0
        self.adaptive_speed = 3
        self.last_altitude_adjustment = 0
        self.consecutive_adjustments = 0
        self.last_battery_levels = []

    def decide_next_command(self):
        """Decide the next command to send to the drone."""
        
        if self.telemetry.sensor_status == "RED":
            
            if self.telemetry.y_position > 2:
                logger.warning("CRITICAL SAFETY OVERRIDE: RED sensor at unsafe altitude, emergency descent")
                self.phase = "EMERGENCY_LANDING" 
                return {
                    'speed': 1,
                    'altitude': -1,
                    'movement': 'fwd'
                }

        self.last_battery_levels.append(self.telemetry.battery)
        if len(self.last_battery_levels) > 10:
            self.last_battery_levels.pop(0)
            
        self.determine_phase()

       
        logger.debug(f"Current phase: {self.phase}, Alt: {self.telemetry.y_position}, "
                    f"Bat: {self.telemetry.battery:.1f}%, Sensor: {self.telemetry.sensor_status}")

        command = {
            'speed': 1, 
            'altitude': 0,
            'movement': 'fwd'
        }
        
        if self.phase == "TAKEOFF":
            command = self.phase_takeoff_strategy()
        elif self.phase == "CRUISE":
            command = self.cruise_strategy()
        elif self.phase == "CONSERVE_ENERGY":
            command = self.conservation_energy_strategy()
        elif self.phase == "EMERGENCY_LANDING":
            command = self.emergency_strategy()
        elif self.phase == "RED_CRUISE":
            command = self.red_cruise_strategy()
        
        
        if 'speed' not in command or command['speed'] is None:
            command['speed'] = 1
        if 'altitude' not in command or command['altitude'] is None:
            command['altitude'] = 0
        if 'movement' not in command or command['movement'] is None:
            command['movement'] = 'fwd'
        
        
        if command['speed'] == 5 and command['altitude'] > 1:
           
            command['altitude'] = min(1, command['altitude'])
            logger.info("Safety limiter: Reduced altitude change due to high speed")
        
        
        if self.telemetry.sensor_status == "RED" and self.telemetry.y_position > 2:
           
            command['altitude'] = -1
            command['speed'] = 1
            logger.warning("SAFETY OVERRIDE: Reducing altitude due to RED sensor status")
        elif self.telemetry.sensor_status == "RED" and self.telemetry.y_position > 0:
           
            command['altitude'] = min(0, command['altitude'])
            logger.warning("SAFETY OVERRIDE: Preventing ascent due to RED sensor status")
        
        return command

    def determine_phase(self):
        """Determine the current flight phase based on telemetry data"""
        
        
        if self.telemetry.sensor_status == "RED":
            logger.warning("RED sensor status detected - switching to EMERGENCY_LANDING phase")
            self.phase = "EMERGENCY_LANDING"
            return
            
        
        if (self.telemetry.battery < 15 or 
            self.telemetry.get_tilt_angle() > 42):
            self.phase = "EMERGENCY_LANDING"
            return
            
        if self.telemetry.y_position < 100:
            self.phase = "TAKEOFF"
            return
            
        if (self.telemetry.battery < 50 or
            (self.telemetry.sensor_status == "YELLOW" and self.telemetry.wind_speed > 12) or
            (self.telemetry.dust_level > 12 and self.telemetry.wind_speed > 12)):
            self.phase = "CONSERVE_ENERGY"
            return
        
        self.phase = "CRUISE"

    def phase_takeoff_strategy(self):
        """Safely ascend to an optimal starting altitude"""
        
       
        if self.telemetry.sensor_status == "RED":
            logger.warning("RED sensor status detected during takeoff - maintaining very low altitude")
            
           
            if self.telemetry.y_position > 2:
                return {
                    "speed": 1,
                    "altitude": -1, 
                    "movement": "fwd"
                }
            
            return {
                "speed": 1,
                "altitude": 0, 
                "movement": "fwd"
            }
        
        
        elif self.telemetry.sensor_status == "YELLOW":
            target_altitude = min(100, self.target_altitude)
        else:
            target_altitude = self.target_altitude
            
       
        altitude_diff = target_altitude - self.telemetry.y_position
        
        if altitude_diff > 20:
            altitude_change = 2
        elif altitude_diff > 0:
            altitude_change = 1
        else:
            altitude_change = 0

        if altitude_diff <= 0:
            self.phase = "CRUISE"
            logger.info(f"Takeoff complete. Transitioning to {self.phase} at altitude {self.telemetry.y_position}")
            return self.cruise_strategy()
            
        return {
            "speed": 1, 
            "altitude": altitude_change,
            "movement": "fwd"
        }

    def cruise_strategy(self):
        """Optimize altitude and speed for maximum distance while monitoring conditions"""
        
        
        self.target_altitude = self.telemetry.get_safe_altitude()
        altitude_diff = self.target_altitude - self.telemetry.y_position
        
       
        optimal_speed = self.calculate_optimal_speed()
        
       
        altitude_change = 0
        if abs(altitude_diff) > 5:
            
            if self.telemetry.sensor_status == "RED":
                altitude_change = min(1, max(-1, altitude_diff))
            elif self.telemetry.sensor_status == "YELLOW":
                altitude_change = min(1, max(-1, altitude_diff))
            else:
                altitude_change = min(2, max(-2, altitude_diff))
            
           
            if self.last_altitude_adjustment != 0:
                self.consecutive_adjustments += 1
                if self.consecutive_adjustments > 5:
                    self.consecutive_adjustments = 0
                    altitude_change = 0  
            else:
                self.consecutive_adjustments = 0
        
        self.last_altitude_adjustment = altitude_change
        
        if len(self.telemetry.history) > 5:
            recent_x_positions = [h['x_position'] for h in self.telemetry.history[-5:]]
            if max(recent_x_positions) - min(recent_x_positions) < 10:
                self.stuck_count += 1
            else:
                self.stuck_count = 0
        
        
        movement = 'fwd'
        
       
        if self.stuck_count > 3:
            movement = 'rev' if self.last_command['movement'] == 'fwd' else 'fwd'
            self.stuck_count = 0
            self.direction_changes += 1
            self.last_direction_change = self.telemetry.iterations
            logger.info(f"Changing direction to {movement} after being stuck")
        
       
        if self.telemetry.iterations - self.last_direction_change < 10:
            movement = self.last_command['movement']
        
       
        if self.telemetry.wind_speed > 15 or self.telemetry.dust_level > 15:
            optimal_speed = min(optimal_speed, 3)
            

        optimal_speed = max(1, optimal_speed)
        
        return {
            'speed': optimal_speed,
            'altitude': altitude_change,
            'movement': movement
        }

    def conservation_energy_strategy(self):
        """Optimize for battery conservation while maintaining safe flight"""
        
       
        current_drain = 0
        if len(self.last_battery_levels) > 5:
            current_drain = (self.last_battery_levels[0] - self.last_battery_levels[-1]) / len(self.last_battery_levels)
        
       
        altitude_change = 0
        
       
        if current_drain > 0.5:  
            if self.telemetry.sensor_status == "GREEN":
                
                altitude_change = min(2, 1500 - self.telemetry.y_position)
                if altitude_change <= 0:
                    altitude_change = 0 
            elif self.telemetry.sensor_status == "YELLOW":
                
                altitude_change = min(1, 800 - self.telemetry.y_position)
                if altitude_change <= 0:
                    altitude_change = 0
            else:  
                altitude_change = min(0, 2 - self.telemetry.y_position)
                if altitude_change < 0:
                    altitude_change = -1  

        if len(self.telemetry.history) > 5:
            recent_x_positions = [h['x_position'] for h in self.telemetry.history[-5:]]
            if max(recent_x_positions) - min(recent_x_positions) < 5:
                self.stuck_count += 1
            else:
                self.stuck_count = 0
        
      
        movement = 'fwd'
        if self.stuck_count > 5:
            movement = 'rev' if self.last_command['movement'] == 'fwd' else 'fwd'
            self.stuck_count = 0
            logger.info(f"Conservation mode: changing direction to {movement}")
        
       
        return {
            'speed': 1,  
            'altitude': altitude_change,
            'movement': movement
        }
            
    def calculate_optimal_speed(self):
        """Calculate the optimal speed based on current conditions"""
        
       
        base_speed = 3
        
       
        if self.telemetry.wind_speed > 15:
            base_speed -= 1  
        if self.telemetry.dust_level > 15:
            base_speed = min(base_speed, 2)  
            
       
        if self.telemetry.battery < 40:
            base_speed -= 1 
            
        
        tilt_angle = self.telemetry.get_tilt_angle()
        if tilt_angle > 30:
            base_speed = 1  
        elif tilt_angle > 20:
            base_speed = min(base_speed, 2)  
       
        return max(1, min(5, base_speed))
    
    def update_with_response(self, response_data):
        """Update controller state based on server response"""
        
       
        previous_status = self.telemetry.sensor_status if hasattr(self.telemetry, 'sensor_status') else None
        
        if 'telemetry' in response_data:
            self.telemetry.parse_telemetry(response_data['telemetry'])
        
        
        if previous_status and previous_status != self.telemetry.sensor_status:
            logger.info(f"Sensor status changed: {previous_status} -> {self.telemetry.sensor_status}")
        
        if 'metrics' in response_data:
            self.telemetry.update_metrices(response_data['metrics'])
        
       
        if 'status' in response_data:
            prev_status = self.telemetry.status
            self.telemetry.status = response_data['status']
            
           
            if response_data['status'] != prev_status:
                if response_data['status'] == 'crashed':
                    logger.warning(f"Drone crashed! Reason: {self.telemetry.crash_reason}")
                    logger.warning(f"Final metrics: iterations={self.telemetry.iterations}, "
                                  f"distance={self.telemetry.total_distance:.2f}")
                else:
                    logger.info(f"Status changed: {prev_status} -> {response_data['status']}")
    
    def emergency_strategy(self):
        """Handle emergency conditions more effectively"""
        
        
        if self.telemetry.get_tilt_angle() > 35:
            logger.warning("Emergency: High tilt angle detected, stabilizing...")
            return {
                'speed': 1,  
                'altitude': 0,  
                'movement': self.last_command['movement']  
            }
        
        
        if self.telemetry.sensor_status == "RED":
            logger.warning("Emergency: RED sensor status, reducing altitude...")
            
            
            target_altitude = min(2, self.telemetry.y_position)
            altitude_change = target_altitude - self.telemetry.y_position
            
           
            if altitude_change < 0:
                altitude_change = max(-1, altitude_change)
                
            return {
                "speed": 1,  
                "altitude": altitude_change,
                "movement": "fwd"  
            }
        
       
        if self.telemetry.sensor_status == "YELLOW" and self.telemetry.y_position > 900:
            logger.warning("Emergency: YELLOW sensor status at high altitude, reducing altitude...")
            
            return {
                "speed": 1,
                "altitude": -1, 
                "movement": "fwd"
            }
        
      
        if self.telemetry.battery < 15:
            logger.warning("Emergency: Low battery, conserving energy...")
            
            
            optimal_altitude = 500 if self.telemetry.sensor_status == "GREEN" else 100
            altitude_diff = optimal_altitude - self.telemetry.y_position
            
            
            if abs(altitude_diff) > 100:
                altitude_change = 1 if altitude_diff > 0 else -1
            else:
                altitude_change = 0 
                
            return {
                'speed': 1, 
                'altitude': altitude_change,
                'movement': 'fwd'  
            }
        
        
        return {
            'speed': 1,
            'altitude': 0,
            'movement': 'fwd'
        }

    def red_cruise_strategy(self):
        """Optimize flight with RED sensor - staying below 3 units while maximizing distance"""
        
        
        current_altitude = self.telemetry.y_position
        
        if current_altitude >= 2.5:
           
            logger.warning(f"RED cruise - altitude {current_altitude} approaching danger zone, descending")
            altitude_change = -1  
        elif current_altitude > 2:
           
            altitude_change = -1
        elif current_altitude < 1:
           
            altitude_change = 1 if current_altitude == 0 else 0
        else:
            
            altitude_change = 0
        
        
        movement = 'fwd'
        
       
        speed = 2 if current_altitude > 2 else 3
        
        
        if current_altitude < 2 and self.telemetry.wind_speed < 10 and self.telemetry.dust_level < 10:
            speed = 4  
        
        return {
            'speed': speed,
            'altitude': altitude_change,
            'movement': movement
        }


def is_server_running(host, port):
    """Check if the server is running on the given host and port."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            s.settimeout(2)
            s.connect((host, port))
            return True
        except (socket.timeout, ConnectionRefusedError):
            return False

async def keep_alive_ping(websocket):
    """Send periodic pings to keep the connection alive."""
    try:
        logger.info("Starting keep-alive ping task")
        while True:
            try:
                logger.debug("Sending keep-alive ping")
                pong_waiter = await websocket.ping()
                
               
                try:
                    await asyncio.wait_for(pong_waiter, timeout=5)
                    logger.debug("Received pong response")
                except asyncio.TimeoutError:
                    logger.warning("No pong received after 5 seconds")
                    
                
                await asyncio.sleep(15)
                
            except websockets.exceptions.ConnectionClosed:
                logger.warning("Connection closed while sending ping")
                break
            except Exception as e:
                logger.error(f"Error sending keep-alive ping: {e}")
                await asyncio.sleep(5)  
                
    except asyncio.CancelledError:
        logger.debug("Keep-alive ping task cancelled")
        pass

async def send_keepalive_messages(websocket):
    """Send periodic keepalive messages that maintain the connection."""
    try:
        while True:
            try:
               
                await websocket.send(json.dumps({
                    "speed": 1,
                    "altitude": 0,
                    "movement": "fwd",
                    "type": "keepalive"
                }))
                logger.debug("Sent keepalive message with required fields")
                await asyncio.sleep(5)  
            except Exception as e:
                logger.error(f"Error sending keepalive message: {e}")
                break
    except asyncio.CancelledError:
        logger.debug("Keepalive message task cancelled")
        pass

async def main():
    """Main function to run the drone controller"""
    url = "ws://localhost:8765"
    host, port = "localhost", 8765
    controller = DroneController()
    retry_count = 0
    max_retries = 5
    
    if not is_server_running(host, port):
        logger.error(f"Server is not running at {host}:{port}. Please start the server and try again.")
        return

    while retry_count < max_retries:
        try:
            logger.info(f"connecting to drone server by this {url}")
           
            async with websockets.connect(
                url, 
                ping_interval=3,
                ping_timeout=10,
                close_timeout=10
            ) as websocket:
                logger.info("connected to the webserver")


                welcome = await websocket.recv()
                welcome_data = json.loads(welcome)
                logger.info(f"welcome message from server: {welcome_data}")

                
                try:
                    sensor_status, altitude = await perform_pre_flight_safety_check(websocket, controller)
                    logger.info(f"After safety check: sensor={sensor_status}, altitude={altitude}")

                   
                    if sensor_status == "RED":
                        initial_command = {
                            'speed': 2,  
                            'altitude': 0 if altitude <= 2 else -1,  
                            'movement': 'fwd' 
                        }
                        controller.phase = "RED_CRUISE"
                    else:
                        initial_command = {
                            'speed': 3,  
                            'altitude': 1, 
                            'movement': 'fwd'
                        }

                    controller.last_command = initial_command
                    await websocket.send(json.dumps(initial_command))
                    logger.info(f"Sent initial command after safety check: {initial_command}")
                    
                   
                    initial_response = await asyncio.wait_for(websocket.recv(), timeout=3.0)
                    initial_response_data = json.loads(initial_response)
                    controller.update_with_response(initial_response_data)
                    
                   
                    if controller.telemetry.sensor_status == "RED" and controller.telemetry.y_position > 2:
                        logger.warning("Still at unsafe altitude! Sending additional emergency descent")
                        emergency_command = {
                            'speed': 1,
                            'altitude': -2,
                            'movement': 'fwd'  
                        }
                        await websocket.send(json.dumps(emergency_command))
                        await asyncio.wait_for(websocket.recv(), timeout=3.0)
                    
                    
                    await asyncio.sleep(0.2)
                
                except Exception as e:
                    logger.error(f"Error during pre-flight safety check: {e}")
                    
                    initial_command = {
                        'speed': 1,
                        'altitude': -1,  
                        'movement': 'fwd'
                    }
                    controller.last_command = initial_command
                    await websocket.send(json.dumps(initial_command))
                
               
                if controller.telemetry.sensor_status == "RED" or controller.telemetry.sensor_status == "YELLOW":

                    logger.warning(f"Detected {controller.telemetry.sensor_status} sensor status - taking preventive action")
                    safe_command = {
                        'speed': 1,
                        'altitude': -1 if controller.telemetry.y_position > 1 else 0,
                        'movement': 'fwd'
                    }
                    await websocket.send(json.dumps(safe_command))
                    safe_response = await asyncio.wait_for(websocket.recv(), timeout=3.0)
                    controller.update_with_response(json.loads(safe_response))
                    logger.info(f"Applied preventive measure for {controller.telemetry.sensor_status} sensor, now at altitude {controller.telemetry.y_position}")
                
                
                keepalive_task = asyncio.create_task(
                    send_keepalive_messages(websocket)
                )
                
                
                while True:
                    
                    command = controller.decide_next_command()
                    controller.last_command = command
                    
                    
                    if command['speed'] < 1:
                        command['speed'] = 1
                        
                   
                    if 'movement' not in command or not command['movement']:
                        command['movement'] = 'fwd'
                    if 'altitude' not in command or command['altitude'] is None:
                        command['altitude'] = 0

                    
                    command_str = json.dumps(command)
                    logger.debug(f"Sending command: {command_str}")
                    
                    try:
                        await websocket.send(command_str)
                        
                       
                        response = await asyncio.wait_for(websocket.recv(), timeout=5.0)
                        response_data = json.loads(response)
                        logger.debug(f"Received response: {response}")

                        controller.update_with_response(response_data)

                        if controller.telemetry.status == 'crashed':
                            logger.warning("Drone crashed! Reconnecting...")
                            break
                            
                    except asyncio.TimeoutError:
                        logger.warning("Timeout waiting for server response")
                        try:
                            
                            pong_waiter = await websocket.ping()
                            await asyncio.wait_for(pong_waiter, timeout=3)
                            logger.info("Connection is still alive after timeout")
                        except Exception as e:
                            logger.error(f"Connection appears to be dead: {e}")
                            break
                            
                    except websockets.exceptions.ConnectionClosed as e:
                        logger.error(f"Connection closed: {e}")
                        break

                   
                    await asyncio.sleep(0.1)
                    
        except Exception as e:
            logger.error(f"Connection error: {e}")
        finally:
           
            if 'keepalive_task' in locals():
                keepalive_task.cancel()
                try:
                    await keepalive_task
                except asyncio.CancelledError:
                    pass
            
        
        retry_count += 1
        logger.info(f"Reconnecting... ({retry_count}/{max_retries})")
        await asyncio.sleep(1)

    logger.info("Max retries reached. Exiting.")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass