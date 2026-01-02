import cv2
import numpy as np
import time
import json
import os
import threading
from datetime import datetime
from pymavlink import mavutil

DETECTION_DIR = "detections"
SCAN_RESULTS_FILE = "scan_results.json"
os.makedirs(DETECTION_DIR, exist_ok=True)

class Scanner:
    def __init__(self, connection_string='udp:127.0.0.1:14551'):
        self.running = False
        self.cap = None
        self.master = None
        self.current_frame = None
        self.display_frame = None
        self.frame_id = 0
        self.detection_id = 1
        
        self.connection_string = connection_string
        
        self.scan_data = {
            "mission_id": "AGRI_MISSION_001",
            "scan_drone": {
                "vehicle_id": "SCAN_DRONE_01",
                "mission_time": datetime.utcnow().isoformat() + "Z",
                "detections": []
            }
        }
        self.scan_data_lock = threading.Lock()
        self.mavlink_connected = False
        self.last_heartbeat = 0
        
        # Initial GPS state to prevent AttributeError before first update
        self.latitude = 0.0
        self.longitude = 0.0
        self.altitude_rel = 0.0
        self.gps_fix_type = 0
        self.gps_satellites = 0
        
        # HSV Thresholds (Default Yellow)
        self.hsv_min = np.array([20, 80, 80])
        self.hsv_max = np.array([35, 255, 255])
        
        # Determine next ID based on existing files/json? 
        # For simplicity, we restart or append.
        # Ideally we load existing scan_results.
        if os.path.exists(SCAN_RESULTS_FILE):
             try:
                with open(SCAN_RESULTS_FILE, 'r') as f:
                    data = json.load(f)
                    self.scan_data = data
                    # Update ID to max + 1
                    dets = data.get("scan_drone", {}).get("detections", [])
                    if dets:
                        # try to parse ID 'PLANT_XXX'
                        try:
                           ids = [int(d['id'].split('_')[1]) for d in dets if 'PLANT_' in d['id']]
                           if ids:
                               self.detection_id = max(ids) + 1
                        except:
                           pass
             except:
                 pass

    def clear_data(self):
        with self.scan_data_lock:
            # 1. Archive current data
            if self.scan_data["scan_drone"]["detections"]:
                if not os.path.exists("logs"):
                    os.makedirs("logs")
                
                timestamp = datetime.utcnow().strftime("%Y%m%d_%H%M%S")
                backup_file = f"logs/scan_backup_{timestamp}.json"
                
                try:
                    with open(backup_file, "w") as f:
                        json.dump(self.scan_data, f, indent=2)
                    print(f"[INFO] Data archived to {backup_file}")
                except Exception as e:
                    print(f"[ERROR] Failed to archive data: {e}")

            # 2. Reset Data
            self.detection_id = 1
            self.scan_data = {
                "mission_id": "AGRI_MISSION_001",
                "scan_drone": {
                    "vehicle_id": "SCAN_DRONE_01",
                    "mission_time": datetime.utcnow().isoformat() + "Z",
                    "detections": []
                }
            }
            # Overwrite file immediately
            with open(SCAN_RESULTS_FILE, "w") as f:
                json.dump(self.scan_data, f, indent=2)
            print("[INFO] Scanner data cleared.")

    def set_connection_string(self, conn_str):
        self.connection_string = conn_str
        # Logic to reconnect could go here if we want dynamic switching

    def connect_mavlink(self):
        if self.mavlink_connected or getattr(self, 'connecting', False):
            return
            
        self.connecting = True
        try:
            print(f"[INFO] Scanner connecting to MAVLink at {self.connection_string}...")
            # Close existing if any
            if self.master:
                try: self.master.close()
                except: pass
                
            self.master = mavutil.mavlink_connection(self.connection_string)
            self.master.wait_heartbeat(timeout=5) 
            print("[INFO] Scanner MAVLink Connected")
            self.mavlink_connected = True
            
            # Start GPS thread if not alive
            if not hasattr(self, 'gps_thread') or self.gps_thread is None or not self.gps_thread.is_alive():
                def _gps_update_loop():
                    while True: # Keep alive as long as master exists? Or self.running? 
                        # If we want to reconnect, we might need this to stop/restart.
                        # Let's link it to master existence.
                        if not self.master: break
                        try:
                            msg = self.master.recv_match(type=['GLOBAL_POSITION_INT', 'HEARTBEAT', 'GPS_RAW_INT'], blocking=True, timeout=1)
                            if msg:
                                self.last_heartbeat = time.time()
                                self.mavlink_connected = True
                                msg_type = msg.get_type()
                                if msg_type == 'GLOBAL_POSITION_INT':
                                    self.latitude = msg.lat / 1e7
                                    self.longitude = msg.lon / 1e7
                                    self.altitude_rel = msg.relative_alt / 1000.0
                                elif msg_type == 'GPS_RAW_INT':
                                    self.gps_fix_type = msg.fix_type
                                    self.gps_satellites = msg.satellites_visible
                            else:
                                if time.time() - self.last_heartbeat > 5:
                                    self.mavlink_connected = False
                        except: 
                            self.mavlink_connected = False
                        time.sleep(0.1)
                
                self.gps_thread = threading.Thread(target=_gps_update_loop, daemon=True)
                self.gps_thread.start()

        except Exception as e:
            print(f"[WARN] Scanner MAVLink connection failed: {e}")
            self.mavlink_connected = False
        finally:
            self.connecting = False

    def get_gps(self):
        if not self.mavlink_connected or not self.master:
            return None

        return {
            "lat": self.latitude,
            "lon": self.longitude,
            "alt": self.altitude_rel,
            "fix": self.gps_fix_type,
            "sats": self.gps_satellites
        }

    def detect_yellow(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Use dynamic thresholds
        mask = cv2.inRange(hsv, self.hsv_min, self.hsv_max)
        
        yellow_pixels = cv2.countNonZero(mask)
        total_pixels = frame.shape[0] * frame.shape[1]
        confidence = yellow_pixels / total_pixels
        detected = confidence > 0.01
        return detected, confidence, mask

    def start(self, camera_index=None):
        if self.running:
            return
            
        if camera_index is not None:
            self.camera_index = camera_index
            
        self.running = True
        self.thread = threading.Thread(target=self._run_loop, daemon=True)
        self.thread.start()
        
        # Connect mavlink in bg
        mav_thread = threading.Thread(target=self.connect_mavlink, daemon=True)
        mav_thread.start()

    def stop(self):
        self.running = False
        if self.cap:
            self.cap.release()
            self.cap = None

    def _run_loop(self):
        # Allow explicit index or default
        idx = getattr(self, 'camera_index', 0)
        print(f"[INFO] Opening Camera {idx}...")
        
        self.cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW)
        
        if not self.cap.isOpened():
            # Fallback or Error? 
            # If user selected specific cam, we should probably fail or warn.
            print(f"[ERROR] Could not open camera {idx}. Trying fallback search...")
            self.cap.release()
            
            # Auto-search 0-5
            found = False
            for i in range(5):
                 if i == idx: continue
                 temp = cv2.VideoCapture(i, cv2.CAP_DSHOW)
                 if temp.isOpened():
                     self.cap = temp
                     print(f"[INFO] Found alternative camera at {i}")
                     found = True
                     break
            
            if not found:
                print("[ERROR] No camera found.")
                self.running = False
                return

        print("[INFO] Scanner Loop Started")
        
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                time.sleep(0.1)
                continue
            
            detected, confidence, mask = self.detect_yellow(frame)
            
            # Draw on frame (Visualization)
            if detected:
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if contours:
                    c = max(contours, key=cv2.contourArea)
                    x, y, w, h = cv2.boundingRect(c)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    label = f"ID: PLANT_{self.detection_id:03d}"
                    cv2.putText(frame, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # Prepare display frame for GUI (Offload from Main Thread)
            try:
                h, w = frame.shape[:2]
                disp_w, disp_h = 640, 480
                scale = min(disp_w/w, disp_h/h)
                new_w, new_h = int(w*scale), int(h*scale)
                resized = cv2.resize(frame, (new_w, new_h))
                self.display_frame = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
                self.frame_id += 1
            except Exception as e:
                print(f"[WARN] Frame conversion failed: {e}")

            self.current_frame = frame # Raw frame for capture logic
            
            # Save Logic
            if detected:
                # We need a debounce/throttle logic here or it spans detections
                # For this simplest implementation, let's just save if confidence is high?
                # Or use a time check.
                # Re-implementing the throttle from original script:
                
                # Check GPS (Simulated fallback if no GPS)
                gps = self.get_gps()
                if not gps:
                    gps = {"lat": 0.0, "lon": 0.0, "alt": 0.0} # Fallback for testing without drone

                det_id = f"PLANT_{self.detection_id:03d}"
                img_path = os.path.join(DETECTION_DIR, f"{det_id}.jpg")
                
                # Check if we recently saved this ID? 
                # Ideally we track positions. 
                # For now, simple throttle:
                
                if hasattr(self, 'last_save_time') and (time.time() - self.last_save_time < 2.0):
                    pass # Skip
                else:
                    cv2.imwrite(img_path, frame)
                    
                    detection_entry = {
                        "id": det_id,
                        "latitude": gps["lat"],
                        "longitude": gps["lon"],
                        "altitude_rel": gps["alt"],
                        "confidence": round(confidence, 3),
                        "timestamp": datetime.utcnow().isoformat() + "Z",
                        "image": img_path
                    }
                    
                    with self.scan_data_lock:
                        self.scan_data["scan_drone"]["detections"].append(detection_entry)
                        # Save JSON immediately or periodically? Immediately is safer for "Review" concurrent access.
                        with open(SCAN_RESULTS_FILE, "w") as f:
                            json.dump(self.scan_data, f, indent=2)

                    print(f"[SCANNER] Detected {det_id}")
                    self.detection_id += 1
                    self.last_save_time = time.time()
            
            time.sleep(0.03) # ~30FPS

# Scanner module for AGROS Ground Control System.
# Import and use Scanner class.
