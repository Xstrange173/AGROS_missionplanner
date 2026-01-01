import json
import os
import sys
from pymavlink import mavutil

SELECTED_TARGETS_FILE = "selected_targets.json"
MISSION_PLAN_FILE = "mission.plan"  # File extension .plan is standard for QGC/MP JSON plans

def create_mission_item(command, params, frame=3, auto_continue=True, do_jump_id=0):
    """
    Helper to create a Mission Planner JSON item
    Frame 3 = MAV_FRAME_GLOBAL_RELATIVE_ALT
    """
    # Ensure params are floats to avoid compatibility issues
    float_params = [float(p) for p in params]
    
    return {
        "autoContinue": auto_continue,
        "command": command,
        "doJumpId": do_jump_id,
        "frame": frame,
        "params": float_params,
        "type": "SimpleItem"
    }

def generate_mp_json(targets):
    """
    Generates a Mission Planner compatible JSON structure.
    """
    items = []
    
    # 1. Takeoff (Command 22), Alt 10m
    items.append(create_mission_item(22, [0, 0, 0, 0, 0, 0, 10], do_jump_id=1))

    jump_id = 2
    for target in targets:
        lat = target['latitude']
        lon = target['longitude']
        alt = target.get('spray_altitude', 3)
        loiter_time = target.get('loiter_time', 6)

        # 2. Fly to Target (NAV_WAYPOINT = 16)
        items.append(create_mission_item(16, [0, 0, 0, 0, lat, lon, alt], do_jump_id=jump_id))
        jump_id += 1
        
        # 3. Loiter/Spray (NAV_LOITER_TIME = 19)
        items.append(create_mission_item(19, [loiter_time, 0, 0, 0, lat, lon, alt], do_jump_id=jump_id))
        jump_id += 1

    # 4. RTL (NAV_RETURN_TO_LAUNCH = 20)
    items.append(create_mission_item(20, [0, 0, 0, 0, 0, 0, 0], do_jump_id=jump_id))

    mission_plan = {
        "fileType": "Plan",
        "geoFence": {
            "polygons": [],
            "version": 2
        },
        "groundStation": "MissionPlanner",
        "mission": {
            "cruiseSpeed": 5,
            "firmwareType": 12, # 12 = ArduPilot standard in MP
            "hoverSpeed": 3,
            "items": items,
            "plannedHomePosition": [float(targets[0]['latitude']), float(targets[0]['longitude']), 0.0] if targets else [0.0, 0.0, 0.0],
            "vehicleType": 2, # Quadcopter
            "version": 2
        },
        "rallyPoints": {
            "points": [],
            "version": 2
        },
        "version": 1
    }
    return mission_plan

def upload_mission_mavlink(targets, connection_string='udp:127.0.0.1:14551'):
    """
    Uploads the mission directly to the drone via MAVLink.
    """
    print(f"[INFO] Connecting to drone at {connection_string}...")
    # Connection string - adjust if needed (e.g., com port or udp)
    master = mavutil.mavlink_connection(connection_string)
    master.wait_heartbeat()
    print("[INFO] Connected. Clearing mission...")

    master.mav.mission_clear_all_send(master.target_system, master.target_component)
    
    # Calculate total items: 1 (Home) + 1 (Takeoff) + 2*N (Fly+Spray) + 1 (RTL) = 3 + 2N
    # Wait, pymavlink Mission protocol includes a "home" (seq 0) usually?
    # Yes, Seq 0 is usually home/current location.
    
    count = 3 + 2 * len(targets)
    master.mav.mission_count_send(master.target_system, master.target_component, count)
    
    seq = 0
    
    # 0. Home (Current location / Dummy)
    master.mav.mission_item_int_send(
        master.target_system, master.target_component, seq,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0, 1, 0, 0, 0, 0, 0, 0, 0
    )
    seq += 1
    
    # 1. Takeoff
    master.mav.mission_item_int_send(
        master.target_system, master.target_component, seq,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 1, 0, 0, 0, 0, 0, 0, 10 # 10m alt
    )
    seq += 1
    
    for target in targets:
        lat = int(target['latitude'] * 1e7)
        lon = int(target['longitude'] * 1e7)
        alt = target.get('spray_altitude', 3)
        loiter_time = target.get('loiter_time', 6)
        
        # Fly to Target
        master.mav.mission_item_int_send(
            master.target_system, master.target_component, seq,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0, 1, 0, 0, 0, 0,
            lat, lon, alt
        )
        seq += 1
        
        # Spray (Loiter Time)
        master.mav.mission_item_int_send(
            master.target_system, master.target_component, seq,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME,
            0, 1, loiter_time, 0, 0, 0,
            lat, lon, alt
        )
        seq += 1
        
    # RTL
    master.mav.mission_item_int_send(
        master.target_system, master.target_component, seq,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
        0, 1, 0, 0, 0, 0, 0, 0, 0
    )
    seq += 1
    
    print("[INFO] MAVLink Mission Uploaded Successfully!")

def generate_waypoints_content(targets):
    """
    Generates QGC WPL 110 text format content.
    """
    lines = ["QGC WPL 110"]
    # Columns: Index CurrentWP CoordFrame Command P1 P2 P3 P4 Lat Lon Alt AutoContinue
    
    # 0. Home 
    # Use first target or 0,0. Frame 0 (Abs) or 3 (Rel)? 
    # Home is usually 0 (Absolute) or 3 (Relative)?
    # MP usually expects Home to be Frame 0 (Global) or matches the plan? 
    # Let's use Frame 3 (Global Rel Alt) for consistency, but Home is usually Global.
    # Actually, standard is Frame 0 for Home if possible, but 3 works.
    h_lat = float(targets[0]['latitude']) if targets else 0.0
    h_lon = float(targets[0]['longitude']) if targets else 0.0
    lines.append(f"0 1 0 16 0 0 0 0 {h_lat:.7f} {h_lon:.7f} 0.000000 1")

    seq = 1
    
    # 1. Takeoff (Cmd 22)
    # Param 7 is Alt.
    lines.append(f"{seq} 0 3 22 0 0 0 0 0 0 10.000000 1")
    seq += 1
    
    for t in targets:
        lat = float(t['latitude'])
        lon = float(t['longitude'])
        alt = float(t.get('spray_altitude', 3))
        loiter = float(t.get('loiter_time', 6))
        
        # Fly to (Cmd 16)
        lines.append(f"{seq} 0 3 16 0 0 0 0 {lat:.7f} {lon:.7f} {alt:.6f} 1")
        seq += 1
        
        # Loiter (Cmd 19) - Param 1 is time
        lines.append(f"{seq} 0 3 19 {loiter:.2f} 0 0 0 {lat:.7f} {lon:.7f} {alt:.6f} 1")
        seq += 1
        
    # RTL (Cmd 20)
    lines.append(f"{seq} 0 3 20 0 0 0 0 0 0 0 1")
    
    return "\n".join(lines)

def parse_waypoints_file(filepath):
    """
    Parses a QGC WPL 110 file into list of MAVLink items
    """
    items = []
    with open(filepath, 'r') as f:
        lines = f.readlines()
        
    if not lines or "QGC WPL 110" not in lines[0]:
        raise ValueError("Invalid Waypoints File (Header missing)")
        
    # Skip header
    for line in lines[1:]:
        parts = line.strip().split()
        if len(parts) < 12: continue
        
        # Format: Index Curr Frame Cmd P1 P2 P3 P4 X Y Z Auto
        # We need to extract: Frame, Cmd, Params(P1..P4, X,Y,Z)
        frame = int(parts[2])
        cmd = int(parts[3])
        p1 = float(parts[4])
        p2 = float(parts[5])
        p3 = float(parts[6])
        p4 = float(parts[7])
        x = float(parts[8])
        y = float(parts[9])
        z = float(parts[10])
        
        items.append({
            'command': cmd,
            'frame': frame,
            'params': [p1, p2, p3, p4, x, y, z]
        })
    return items

def upload_mission_from_file(filepath, connection_string='udp:127.0.0.1:14551'):
    """
    Uploads a Mission Planner .plan/.json OR .waypoints file.
    """
    if not os.path.exists(filepath):
        raise FileNotFoundError(f"File not found: {filepath}")

    items = []
    
    # Check if text format
    is_text = False
    with open(filepath, 'r') as f:
        first_line = f.readline()
        if "QGC WPL 110" in first_line:
            is_text = True
            
    if is_text:
        raw_items = parse_waypoints_file(filepath)
        # Convert internal dict to structure loop expects below? 
        # Actually parse_waypoints_file returns dict with 'command', 'frame', 'params'.
        # The JSON 'items' list also has 'command', 'frame', 'params'.
        # So compatible!
        items = raw_items
    else:
        # JSON fallback
        with open(filepath, 'r') as f:
            plan = json.load(f)
        items = plan.get('mission', {}).get('items', [])

    if not items:
        raise ValueError("No mission items found in file.")

    print(f"[INFO] Uploading {len(items)} items from {filepath}...")
    print(f"[INFO] Connecting to drone at {connection_string}...")
    master = mavutil.mavlink_connection(connection_string)
    master.wait_heartbeat()
    
    master.mav.mission_clear_all_send(master.target_system, master.target_component)
    
    # Count: If text file included Home (Seq 0), we use length.
    # If JSON list included Home? 
    # Usually JSON "items" starts at 1? 
    # Let's check our JSON generator: we put Home in "plannedHomePosition" but not in items.
    # Our Text generator: Line 0 IS Home.
    
    # If Text: Item 0 is Home.
    # If JSON: Item 0 is usually Takeoff (Seq 1).
    
    # We need to construct the mission sequence properly.
    if is_text:
        # Text file usually contains Seq 0.
        # So we send all items.
        # But MAVLink protocol: 
        # Seq 0 is Home.
        # mission_count should be len(items).
        pass
    else:
        # JSON usually doesn't have Seq 0 in 'items'.
        # We need to insert a dummy home or use plannedHome?
        # For this fix, let's assume we insert dummy home if JSON.
        pass
    
    count = len(items) 
    # If JSON, we need +1 for Home? 
    # Wait, my previous JSON uploader added Home manually.
    # If text has home, count is fine.
    # Let's adjust logic:
    
    final_items = []
    if is_text:
        final_items = items # contains seq 0
    else:
        # Add Home
        final_items.append({
            'command': 16, # Waypoint
            'frame': 0, # Global
            'params': [0,0,0,0,0,0,0] # Dummy
        })
        final_items.extend(items)
        
    master.mav.mission_count_send(master.target_system, master.target_component, len(final_items))
    
    seq = 0
    for item in final_items:
        cmd = item['command']
        params = item['params']
        frame = item.get('frame', mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT)
        
        master.mav.mission_item_send(
            master.target_system, master.target_component, seq,
            frame, cmd, 0, 1,
            params[0], params[1], params[2], params[3],
            params[4], params[5], params[6]
        )
        seq += 1
        
    print("[INFO] File Mission Uploaded Successfully!")

def run_mission_generation(upload=False):
    if not os.path.exists(SELECTED_TARGETS_FILE):
        print(f"[ERROR] {SELECTED_TARGETS_FILE} not found. Run Target Selector first.")
        return

    with open(SELECTED_TARGETS_FILE, 'r') as f:
        data = json.load(f)
        targets = data.get("selected_targets", [])

    if not targets:
        print("[WARN] No targets selected.")
        return

    # Method 1: Generate Mission Planner File
    mp_json = generate_mp_json(targets)
    with open(MISSION_PLAN_FILE, 'w') as f:
        json.dump(mp_json, f, indent=2)
    print(f"[SUCCESS] Mission Planner file generated: {MISSION_PLAN_FILE}")

    # Method 2: Automatic Upload
    if upload:
        upload_mission_mavlink(targets)
    else:
        print("[INFO] Skipping MAVLink upload. Use --upload CLI arg or select in Menu to upload.")

if __name__ == "__main__":
    upload_arg = "--upload" in sys.argv
    run_mission_generation(upload=upload_arg)
