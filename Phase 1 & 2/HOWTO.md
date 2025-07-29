# VOXL Offboard Path Following (CSV-based)

How to run:

1. Place CSV file on VOXL

scp path_points.csv voxl:/data/path_points.csv

2. Configure VOXL
    Edit /etc/modalai/voxl-vision-hub.conf:

    "offboard_mode": "wps",
    "coordinate_move_home": true

3. Restart Services

systemctl restart voxl-vision-hub

4. Fly
    Arm the drone
    Switch to offboard mode
    Drone will follow path in the CSV