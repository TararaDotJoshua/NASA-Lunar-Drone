# VOXL Offboard Path Following (CSV-based)

How to run:

1. Place CSV file on VOXL

scp path_points.csv voxl:/data/path_points.csv
scp tag_map.csv voxl:/data/tag_map.csv

2. Configure VOXL
    Edit /etc/modalai/voxl-vision-hub.conf:

    "offboard_mode": "wps",
    "coordinate_move_home": true,
    "en_tag_fixed_frame": true,
    "fixed_frame_filter_len": 5,
    "en_transform_mavlink_pos_setpoints_from_fixed_frame": true


3. Restart Services

systemctl restart voxl-tag-tracker
systemctl restart voxl-vision-hub


4. Fly
    Arm the drone
    Switch to offboard mode
    Drone will now:
        Relocalize to a nearby AprilTag if visible
        Follow the waypoint path defined in path_points.csv

## Notes:

- AprilTags must match IDs and poses defined in tag_map.csv

- Ensure tags are visible and mounted firmly in the environment

- Use large, high-contrast tags for better detection

- To disable relocalization:
    "en_tag_fixed_frame": false
