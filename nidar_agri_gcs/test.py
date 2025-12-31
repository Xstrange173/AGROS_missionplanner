from mavlink_utils import connect_mavlink, get_global_position, set_mode
import time

master = connect_mavlink()

for _ in range(5):
    pos = get_global_position(master)
    if pos:
        print("Position:", pos)
    time.sleep(1)

# set_mode(master, "GUIDED")  # uncomment ONLY if safe
