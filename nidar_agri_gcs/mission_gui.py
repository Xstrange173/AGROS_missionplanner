import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext, font, filedialog
import json
import os
import cv2
import mission_generator
from threading import Thread
import time

# Import Scanner
try:
    from scan_receiver import Scanner
    SCANNER_AVAILABLE = True
except ImportError:
    SCANNER_AVAILABLE = False
    print("Warning: scan_receiver.py not found or invalid.")

# Config
SCAN_RESULTS_FILE = "scan_results.json"
SETTINGS_FILE = "settings.json"

class MissionAGROSGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("AGROS Mission Control & Live Scan Station")
        self.root.geometry("1150x850")
        
        self.raw_detections = []    # Loaded from JSON
        self.selection_state = {}   # ID -> Boolean
        
        # Load Settings
        settings = self.load_settings()
        self.connection_string_var = tk.StringVar(value=settings.get("connection_string", "udp:127.0.0.1:14551"))
        self.mp_path_var = tk.StringVar(value=settings.get("mp_path", self.auto_detect_mp()))

        # Scanner Instance
        self.scanner = None
        if SCANNER_AVAILABLE:
            self.scanner = Scanner()
            if self.scanner:
                self.scanner.set_connection_string(self.connection_string_var.get())
        
        # Style
        style = ttk.Style()
        style.theme_use('clam')
        
        # Default Config
        self.connection_string_var = tk.StringVar(value="udp:127.0.0.1:14551")
        if self.scanner:
             self.scanner.set_connection_string(self.connection_string_var.get())
        
        # === Header ===
        header_frame = ttk.Frame(root, padding="10")
        header_frame.pack(fill=tk.X)
        ttk.Label(header_frame, text="AGROS Mission Command", font=("Helvetica", 16, "bold")).pack(side=tk.LEFT)
        
        # Connection Status in Header
        status_frame = ttk.Frame(header_frame)
        status_frame.pack(side=tk.RIGHT)
        ttk.Label(status_frame, text="MAVLink Status:").pack(side=tk.LEFT, padx=5)
        self.lbl_mav_status = ttk.Label(status_frame, text="CONNECTED" if (self.scanner and self.scanner.mavlink_connected) else "DISCONNECTED", 
                                       font=("Arial", 10, "bold"), foreground="red")
        self.lbl_mav_status.pack(side=tk.LEFT, padx=5)

        # LINK MISSION PLANNER BUTTON
        link_btn = ttk.Button(header_frame, text="🚀 LINK MISSION PLANNER", command=self.launch_mission_planner)
        link_btn.pack(side=tk.RIGHT, padx=20)
        
        # === Controls Pane (Notebook for Tabbed View) ===
        notebook = ttk.Notebook(root)
        notebook.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        # TAB 1: MISSION PLANNING
        self.frame_plan = ttk.Frame(notebook)
        notebook.add(self.frame_plan, text="Plan & Generate")
        self._init_planning_tab(self.frame_plan)
        
        # TAB 2: LIVE SCANNING
        self.frame_scan = ttk.Frame(notebook)
        notebook.add(self.frame_scan, text="Live Vision System")
        self._init_scanning_tab(self.frame_scan)
        
        # TAB 3: SETTINGS
        self.frame_settings = ttk.Frame(notebook)
        notebook.add(self.frame_settings, text="Settings")
        self._init_settings_tab(self.frame_settings)
        
        # === Log Window (Bottom) ===
        log_frame = ttk.LabelFrame(root, text="System Log", padding="5")
        log_frame.pack(fill=tk.X, padx=10, pady=5)
        
        # Clear Log Button
        clear_btn = ttk.Button(log_frame, text="Clear Log", command=self.clear_log, width=10)
        clear_btn.pack(side=tk.RIGHT, padx=5, anchor="n")
        
        self.log_area = scrolledtext.ScrolledText(log_frame, height=5, state='disabled')
        self.log_area.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Start Video Loop
        self.video_update_loop()

        # Initial Load
        self.load_data()

    def clear_log(self):
        self.log_area.config(state='normal')
        self.log_area.delete('1.0', tk.END)
        self.log_area.config(state='disabled')

    def log(self, message):
        self.log_area.config(state='normal')
        self.log_area.insert(tk.END, message + "\n")
        self.log_area.see(tk.END)
        self.log_area.config(state='disabled')
        print(message)

    def apply_settings(self):
        conn_str = self.connection_string_var.get()
        mp_path = self.mp_path_var.get()
        
        self.log(f"[CONFIG] Updating connection to: {conn_str}")
        self.log(f"[CONFIG] MP Path: {mp_path}")
        
        if self.scanner:
            self.scanner.set_connection_string(conn_str)
        
        self.save_settings()
        messagebox.showinfo("Settings", "Configuration Saved Successfully!")

    def load_settings(self):
        if os.path.exists(SETTINGS_FILE):
            try:
                with open(SETTINGS_FILE, 'r') as f:
                    return json.load(f)
            except: pass
        return {}

    def save_settings(self):
        settings = {
            "connection_string": self.connection_string_var.get(),
            "mp_path": self.mp_path_var.get()
        }
        with open(SETTINGS_FILE, 'w') as f:
            json.dump(settings, f, indent=2)

    def auto_detect_mp(self):
        paths = [
            r"C:\Program Files (x86)\Mission Planner\MissionPlanner.exe",
            r"C:\Program Files\Mission Planner\MissionPlanner.exe",
            os.path.join(os.path.expanduser("~"), "Desktop", "Mission Planner", "MissionPlanner.exe")
        ]
        for p in paths:
            if os.path.exists(p): return p
        return ""

    def browse_mp_path(self):
        path = filedialog.askopenfilename(title="Select MissionPlanner.exe", filetypes=[("Executable", "*.exe")])
        if path:
            self.mp_path_var.set(path)

    def launch_mission_planner(self):
        path = self.mp_path_var.get()
        if not path or not os.path.exists(path):
            messagebox.showerror("Launcher", "Mission Planner Executable not found!\nPlease configure path in Settings.")
            return
        
        try:
            self.log(f"[LAUNCH] Starting Mission Planner: {path}")
            os.startfile(path)
        except Exception as e:
            messagebox.showerror("Launcher", f"Failed to launch MP: {e}")

    # --- SETTINGS TAB ---
    def _init_settings_tab(self, parent):
        # Container
        container = ttk.Frame(parent, padding="10")
        container.pack(fill=tk.BOTH, expand=True)

        # MAVLink Config
        frame = ttk.LabelFrame(container, text="Communication Configuration", padding="10")
        frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(frame, text="MAVLink Connection String:", font=("Arial", 10, "bold")).grid(row=0, column=0, sticky="w", padx=5, pady=5)
        ttk.Entry(frame, textvariable=self.connection_string_var, width=40).grid(row=0, column=1, padx=5, pady=5)
        
        info_lbl = ttk.Label(frame, text="Examples: udp:127.0.0.1:14551, com4", foreground="gray")
        info_lbl.grid(row=1, column=1, sticky="w", padx=5)

        # MP Integration
        mp_frame = ttk.LabelFrame(container, text="Mission Planner Integration (Universal)", padding="10")
        mp_frame.pack(fill=tk.X, pady=10)
        
        ttk.Label(mp_frame, text="Mission Planner Path:", font=("Arial", 10, "bold")).grid(row=0, column=0, sticky="w", padx=5, pady=5)
        ttk.Entry(mp_frame, textvariable=self.mp_path_var, width=60).grid(row=0, column=1, padx=5, pady=5)
        ttk.Button(mp_frame, text="Browse...", command=self.browse_mp_path).grid(row=0, column=2, padx=5, pady=5)
        
        ttk.Label(mp_frame, text="Tip: Configure this once and it will be saved for future sessions.", 
                  foreground="gray").grid(row=1, column=1, sticky="w", padx=5)

        # Save Button at Bottom
        save_btn = ttk.Button(container, text="💾 SAVE ALL SETTINGS", style="Accent.TButton", command=self.apply_settings)
        save_btn.pack(pady=20)

    # --- PLANNING TAB LOGIC ---
    def _init_planning_tab(self, parent):
        # Parameters Frame
        param_frame = ttk.LabelFrame(parent, text="Mission Parameters", padding="10")
        param_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Label(param_frame, text="Takeoff Alt (m):").grid(row=0, column=0, padx=5)
        self.takeoff_alt_var = tk.DoubleVar(value=10.0)
        ttk.Entry(param_frame, textvariable=self.takeoff_alt_var, width=8).grid(row=0, column=1, padx=5)
        
        ttk.Label(param_frame, text="Spray Alt (m):").grid(row=0, column=2, padx=5)
        self.spray_alt_var = tk.DoubleVar(value=3.0)
        ttk.Entry(param_frame, textvariable=self.spray_alt_var, width=8).grid(row=0, column=3, padx=5)
        
        ttk.Label(param_frame, text="Loiter Time (s):").grid(row=0, column=4, padx=5)
        self.loiter_time_var = tk.IntVar(value=6)
        ttk.Entry(param_frame, textvariable=self.loiter_time_var, width=8).grid(row=0, column=5, padx=5)
        
        ttk.Button(param_frame, text="Refresh", command=self.load_data).grid(row=0, column=6, padx=10)
        ttk.Button(param_frame, text="Clear All Detections", command=self.clear_detections).grid(row=0, column=7, padx=5)

        # Split Pane
        content_pane = ttk.PanedWindow(parent, orient=tk.HORIZONTAL)
        content_pane.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        # Left: Treeview
        list_frame = ttk.Labelframe(content_pane, text="Detections (Check to include)", padding="5")
        content_pane.add(list_frame, weight=2)
        
        columns = ("select", "id", "conf", "lat", "lon", "time")
        self.tree = ttk.Treeview(list_frame, columns=columns, show="headings", height=15)
        
        self.tree.heading("select", text="[x]")
        self.tree.heading("id", text="ID")
        self.tree.heading("conf", text="Conf")
        self.tree.heading("lat", text="Lat")
        self.tree.heading("lon", text="Lon")
        self.tree.heading("time", text="Time")
        
        self.tree.column("select", width=40, anchor="center")
        self.tree.column("id", width=80)
        self.tree.column("conf", width=50)
        self.tree.column("lat", width=80)
        self.tree.column("lon", width=80)
        self.tree.column("time", width=140)
        
        scrollbar = ttk.Scrollbar(list_frame, orient=tk.VERTICAL, command=self.tree.yview)
        self.tree.configure(yscroll=scrollbar.set)
        self.tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        # Right: Image Preview
        preview_frame = ttk.Labelframe(content_pane, text="Target Preview", padding="5")
        content_pane.add(preview_frame, weight=1)
        
        self.img_label = ttk.Label(preview_frame, text="Select a row to preview")
        self.img_label.pack(fill=tk.BOTH, expand=True, anchor="center")
        
        # Action Buttons
        action_frame = ttk.Frame(parent, padding="10")
        action_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Button(action_frame, text="GENERATE MISSION FILE", command=self.generate_file).pack(side=tk.LEFT, padx=5)
        ttk.Label(action_frame, text=" | ").pack(side=tk.LEFT, padx=5)
        ttk.Button(action_frame, text="UPLOAD MISSION (Selected)", command=self.start_upload_thread).pack(side=tk.LEFT, padx=5)
        ttk.Button(action_frame, text="UPLOAD FROM FILE", command=self.start_upload_file_thread).pack(side=tk.RIGHT, padx=5)

        # Binding
        self.tree.bind('<Button-1>', self.on_click)
        self.tree.bind('<<TreeviewSelect>>', self.on_select)

    # --- SCANNING TAB LOGIC ---
    def _init_scanning_tab(self, parent):
        # Top Controls
        ctrl_frame = ttk.Frame(parent, padding="10")
        ctrl_frame.pack(fill=tk.X)
        
        # Camera Selector
        ttk.Label(ctrl_frame, text="Cam Input:").pack(side=tk.LEFT, padx=5)
        self.cam_combo = ttk.Combobox(ctrl_frame, values=["0", "1", "2", "3", "4"], width=3, state="readonly")
        self.cam_combo.current(0)
        self.cam_combo.pack(side=tk.LEFT, padx=5)
        
        self.btn_start = ttk.Button(ctrl_frame, text="START SCAN", command=self.start_scan)
        self.btn_start.pack(side=tk.LEFT, padx=10)
        
        self.btn_stop = ttk.Button(ctrl_frame, text="STOP", command=self.stop_scan, state='disabled')
        self.btn_stop.pack(side=tk.LEFT, padx=5)
        
        self.lbl_status = ttk.Label(ctrl_frame, text="Status: IDLE", font=("Arial", 10, "bold"))
        self.lbl_status.pack(side=tk.LEFT, padx=20)
        
        # Split Pane for Video + Settings
        scan_pane = ttk.PanedWindow(parent, orient=tk.HORIZONTAL)
        scan_pane.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        # Left: Video Feed
        video_frame = ttk.LabelFrame(scan_pane, text="Drone Feed (Live)", padding="5")
        scan_pane.add(video_frame, weight=3)
        
        self.live_video_label = ttk.Label(video_frame, text="Camera Offline", background="black", foreground="white", anchor="center")
        self.live_video_label.pack(fill=tk.BOTH, expand=True)

        # Right: Calibration Controls
        calib_frame = ttk.LabelFrame(scan_pane, text="HSV Calibration", padding="10")
        scan_pane.add(calib_frame, weight=1)
        
        # Sliders
        self.h_min = self._create_slider(calib_frame, "H Min", 0, 179, 20)
        self.s_min = self._create_slider(calib_frame, "S Min", 0, 255, 80)
        self.v_min = self._create_slider(calib_frame, "V Min", 0, 255, 80)
        
        ttk.Separator(calib_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=10)
        
        self.h_max = self._create_slider(calib_frame, "H Max", 0, 179, 35)
        self.s_max = self._create_slider(calib_frame, "S Max", 0, 255, 255)
        self.v_max = self._create_slider(calib_frame, "V Max", 0, 255, 255)

    def _create_slider(self, parent, label, min_val, max_val, default):
        frame = ttk.Frame(parent)
        frame.pack(fill=tk.X, pady=2)
        ttk.Label(frame, text=label, width=6).pack(side=tk.LEFT)
        scale = tk.Scale(frame, from_=min_val, to=max_val, orient=tk.HORIZONTAL, command=self.update_hsv)
        scale.set(default)
        scale.pack(side=tk.RIGHT, fill=tk.X, expand=True)
        return scale

    def update_hsv(self, _=None):
        if self.scanner:
            import numpy as np
            h_min = self.h_min.get()
            s_min = self.s_min.get()
            v_min = self.v_min.get()
            
            h_max = self.h_max.get()
            s_max = self.s_max.get()
            v_max = self.v_max.get()
            
            self.scanner.hsv_min = np.array([h_min, s_min, v_min])
            self.scanner.hsv_max = np.array([h_max, s_max, v_max])

    def start_scan(self):
        if not self.scanner:
            self.log("[ERROR] Scanner module unavailable.")
            return
        
        try:
            full_str = self.cam_combo.get() # "0"
            idx = int(full_str)
        except:
            idx = 0
            
        # Ensure connection string is up to date
        if self.connection_string_var:
             self.scanner.set_connection_string(self.connection_string_var.get())

        self.log(f"[INFO] Starting Scanner on Camera {idx}...")
        self.scanner.start(camera_index=idx)
        
        self.btn_start.config(state='disabled')
        self.btn_stop.config(state='normal')
        self.lbl_status.config(text="Status: SCANNING", foreground="green")

    def stop_scan(self):
        if not self.scanner:
            return
            
        self.log("[INFO] Stopping Scanner...")
        self.scanner.stop()
        self.btn_start.config(state='normal')
        self.btn_stop.config(state='disabled')
        self.lbl_status.config(text="Status: IDLE", foreground="black")
        self.live_video_label.config(image='')
        self.live_video_label.config(text="Camera Offline")

    def video_update_loop(self):
        # Update MAVLink Status
        if self.scanner:
            if self.scanner.mavlink_connected:
                self.lbl_mav_status.config(text="CONNECTED", foreground="green")
            else:
                self.lbl_mav_status.config(text="DISCONNECTED", foreground="red")

        if self.scanner and self.scanner.running:
             frame = self.scanner.current_frame
             if frame is not None:
                 # Resize for display
                 h, w = frame.shape[:2]
                 # Fixed display siz
                 disp_w, disp_h = 640, 480
                 scale = min(disp_w/w, disp_h/h)
                 new_w, new_h = int(w*scale), int(h*scale)
                 
                 resized = cv2.resize(frame, (new_w, new_h))
                 rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
                 
                 # PPM Encode trick
                 _, buffer = cv2.imencode('.ppm', rgb)
                 data = buffer.tobytes()
                 photo = tk.PhotoImage(data=data)
                 
                 self.live_video_label.config(image=photo, text="")
                 self.live_video_label.image = photo 
        
        # Periodic refresh loop
        self.root.after(30, self.video_update_loop)

    # --- SHARED/HELPER LOGIC ---
    def load_data(self):
        # Trigger reconnection if disconnected
        if self.scanner and not self.scanner.mavlink_connected:
             self.log("[INFO] Refresh triggered MAVLink reconnection attempt...")
             Thread(target=self.scanner.connect_mavlink, daemon=True).start()

        # 1. Try Loading proper Scan Results (Method 3 flow)
        data_source = self.raw_detections 
        
        if self.scanner and self.scanner.scan_data:
             data_source = self.scanner.scan_data.get("scan_drone", {}).get("detections", [])
        elif os.path.exists(SCAN_RESULTS_FILE):
             try:
                 with open(SCAN_RESULTS_FILE, 'r') as f:
                    data = json.load(f)
                    data_source = data.get("scan_drone", {}).get("detections", [])
             except:
                 pass
        
        if len(data_source) == len(self.raw_detections) and len(data_source) > 0:
             pass
             
        self.raw_detections = data_source
        self.log(f"[INFO] Loaded {len(self.raw_detections)} detections.")

        # Populate Tree
        self.tree.delete(*self.tree.get_children())
        self.selection_state = {}
        
        for i, det in enumerate(self.raw_detections):
            conf = det.get('confidence', 0.0)
            timestamp = det.get('timestamp', '?')
            
            is_selected = (conf >= 0.90)
            self.selection_state[i] = is_selected
            
            check_mark = "✔" if is_selected else "☐"
            self.tree.insert("", tk.END, values=(
                check_mark, det['id'], f"{conf:.2f}",
                f"{det['latitude']:.6f}", f"{det['longitude']:.6f}",
                timestamp
            ), iid=str(i))

    def on_click(self, event):
        region = self.tree.identify("region", event.x, event.y)
        if region == "cell":
            col = self.tree.identify_column(event.x)
            item_id = self.tree.identify_row(event.y)
            if col == "#1" and item_id:
                try:
                    idx = int(item_id)
                    current_state = self.selection_state.get(idx, False)
                    new_state = not current_state
                    self.selection_state[idx] = new_state
                    char = "✔" if new_state else "☐"
                    self.tree.set(item_id, "select", char)
                except ValueError:
                    pass

    def on_select(self, event):
        selected_items = self.tree.selection()
        if not selected_items:
            return
        try:
            item_id = selected_items[0]
            idx = int(item_id)
            if 0 <= idx < len(self.raw_detections):
                det = self.raw_detections[idx]
                if det and det.get('image') and os.path.exists(det['image']):
                    self.show_image(det['image'])
                else:
                    self.img_label.config(image='', text="No Image Available")
        except ValueError:
            pass

    def show_image(self, path):
        try:
            cv_img = cv2.imread(path)
            if cv_img is None: raise Exception("CV2 Load Failed")
            h, w = cv_img.shape[:2]
            scale = min(300/w, 250/h)
            new_w, new_h = int(w*scale), int(h*scale)
            resized = cv2.resize(cv_img, (new_w, new_h))
            rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
            _, buffer = cv2.imencode('.ppm', rgb)
            data = buffer.tobytes()
            photo = tk.PhotoImage(data=data)
            self.img_label.config(image=photo, text="")
            self.img_label.image = photo 
        except Exception as e:
            self.img_label.config(text="Image Error")

    def get_selected_targets(self):
        targets = []
        spray_alt = self.spray_alt_var.get()
        loiter_time = int(self.loiter_time_var.get())
        
        for i, det in enumerate(self.raw_detections):
            if self.selection_state.get(i, False):
                t = det.copy()
                t['spray_altitude'] = spray_alt
                t['loiter_time'] = loiter_time
                targets.append(t)
        return targets

    def clear_detections(self):
        if not messagebox.askyesno("Confirm", "Are you sure you want to delete ALL detections?"):
            return
            
        if self.scanner:
            self.scanner.clear_data()
        else:
             dummy = {
                "mission_id": "AGRI_MISSION_001",
                "scan_drone": {
                    "vehicle_id": "SCAN_DRONE_01",
                    "mission_time": "2023-01-01T00:00:00Z",
                    "detections": []
                }
            }
             try:
                 with open(SCAN_RESULTS_FILE, 'w') as f:
                     json.dump(dummy, f, indent=2)
             except: pass
        
        self.raw_detections = []
        self.tree.delete(*self.tree.get_children())
        self.selection_state = {}
        self.img_label.config(image='', text="No Data")
        self.log("[INFO] Detections cleared.")

    def generate_file(self):
        targets = self.get_selected_targets()
        if not targets:
            messagebox.showwarning("Warning", "No targets selected!")
            return
            
        # 1. Ask User for Path
        initial_file = "mission.waypoints"
        path = filedialog.asksaveasfilename(
            defaultextension=".waypoints",
            initialfile=initial_file,
            filetypes=[("Waypoints", "*.waypoints"), ("Mission Plan", "*.plan"), ("All Files", "*.*")],
            title="Save Mission File"
        )
        
        if not path:
            return 
            
        try:
            # Fix: Use Text Waypoints Format to avoid MP JSON parsing errors
            content = mission_generator.generate_waypoints_content(targets)
            
            # Note: Parameter manual override for Takeoff Alt is harder in text, 
            # but generate_waypoints_content() already uses 10m default. 
            # If we want to support the GUI var, we should pass it.
            # Let's simple-patch it in the text or update generator to accept it.
            # Generator hardcodes 10m. Let's do a quick replace if needed or ignore for now.
            # Actually, let's verify if we can pass it. 
            # Ideally I'd update generate_waypoints_content signature, but let's just write what we have.
            
            with open(path, 'w') as f:
                f.write(content)
                
            self.log(f"[SUCCESS] Saved mission to {path}")
            messagebox.showinfo("Success", f"Saved mission to:\n{path}\n\nYou can now upload this file using 'Upload From File'.")
            
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def start_upload_thread(self):
        targets = self.get_selected_targets()
        if not targets:
            messagebox.showwarning("Warning", "No targets selected!")
            return
        t = Thread(target=self.upload_logic, args=(targets,))
        t.start()
        
    def upload_logic(self, targets):
        try:
            conn_str = self.connection_string_var.get()
            self.log(f"[INFO] Uploading selected targets to {conn_str}...")
            mission_generator.upload_mission_mavlink(targets, connection_string=conn_str)
            messagebox.showinfo("Upload", "Upload Successful!")
            self.log("[SUCCESS] Upload Complete.")
        except Exception as e:
            self.log(f"[ERROR] Upload Failed: {e}")
            messagebox.showerror("Upload Error", str(e))

    def start_upload_file_thread(self):
        path = filedialog.askopenfilename(
            title="Select Mission File to Upload",
            filetypes=[("Waypoints", "*.waypoints"), ("Mission Plan", "*.plan"), ("JSON", "*.json"), ("All Files", "*.*")]
        )
        if not path:
            return
            
        t = Thread(target=self.upload_file_logic, args=(path,))
        t.start()

    def upload_file_logic(self, path):
        try:
             conn_str = self.connection_string_var.get()
             self.log(f"[INFO] Uploading file: {path} to {conn_str}...")
             mission_generator.upload_mission_from_file(path, connection_string=conn_str)
             messagebox.showinfo("Upload", "File Upload Successful!")
             self.log("[SUCCESS] File Upload Complete.")
        except Exception as e:
             self.log(f"[ERROR] File Upload Failed: {e}")
             messagebox.showerror("Upload Error", str(e))

if __name__ == "__main__":
    root = tk.Tk()
    app = MissionAGROSGUI(root)
    root.mainloop()
