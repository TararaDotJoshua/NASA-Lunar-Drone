# VOXL Offboard Path Following (CSV-based)

This module (`offboard_lines.c`) enables a ModalAI VOXL 2-based drone to follow a path defined by hardcoded 3D waypoints stored in a CSV file. It uses the ModalAI offboard SDK infrastructure to generate and stream MAVLink setpoints in NED frame.

---

## Features

- Supports hardcoded or .CSV path flying
- Smooth interpolation between points
- Home-relative or abs coord support

---

## File Structure

`offboard_lines.c`
- Main offboard thread. Loads waypoints and sends position setpoints.
`/data/path_points.csv`
- CSV file containing hardcoded 3D path points (X, Y, Z) in meters.
`config_file.h` & `config_file.c`
- Configuration system used by `voxl-vision-hub`.

---

## path_points.csv Format

Comma-separated values, one line per waypoint:

x,y,z
0.0,0.0,-1.5
1.0,0.0,-1.5
1.0,1.0,-1.5
0.0,1.0,-1.5
