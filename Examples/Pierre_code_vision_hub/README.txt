
This code allows the drone to follow a path defined by coordinate points stored in a CSV file.
There are two methods, the first one (get_csv_coordinates) gets data from the csv file. 
The points’ coordinates are written in columns, one for x, one for y and one for z. 
In the offboard_coordinate.c file you will need to update the path to the correct location of your CSV file
on line 43. This method returns a list containing three different arrays (coorx, coory, coorz) 
and the number of points written in the file.
The second method (_init_path_coordinate) takes the coordinates from the CSV file and generates 
additional subpoints that the drone will follow along its path.
A third method allows the drone to follow a square shaped path. The coordinates of the four corners of 
the square are already hardcoded in the code.

Added some lines in the config_file.c (lines 423, 424, 500, 501, 671, 672) and in 
config_file.h (lines 66, 75, 76, 189, 190). 

Main.c calls the offboard_mode_init method, we had to add some lines for the program to work with the 
offboard_square.c and offboard_coordinate.c files in the offboard_mode.c (lines 55 to 62, 97 to 104 and 131, 132).

Launch the main program and select the desired mode for the drone (figure_eight, square, coordinates, …).

