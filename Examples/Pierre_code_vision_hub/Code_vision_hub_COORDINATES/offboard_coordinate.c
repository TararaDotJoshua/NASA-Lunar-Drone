#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <errno.h>
#include <math.h>

#include "config_file.h"
#include "mavlink_io.h"
#include "autopilot_monitor.h"
#include "geometry.h"
#include "macros.h"
#include "misc.h"
#include "offboard_coordinate.h"

#define FLIGHT_ALTITUDE	-1.5f
#define RATE			30	// loop rate hz
#define STEPS			25 000  // We chose arbitrarily 25 000 as the maximum number of sub-points (so the drone can approximatly fly 500m)

#define MAX_LINE 256	// Maximum length of a CSV file line.


static int running = 0;
static pthread_t offboard_coordinate_thread_id;
static int en_debug = 0;

static mavlink_set_position_target_local_ned_t path[STEPS];
static mavlink_set_position_target_local_ned_t home_position;



typedef struct {		//Definition of CoordinateList which is composed of 3 lists and an int.
    float coorx[STEPS]; //These lists will contain the x, y and z coordinates of each points entered in the CSV file.
    float coory[STEPS]; 
    float coorz[STEPS];
    int nb_pts; 		// Number of points in the CSV file.
} CoordinateList;		// CoordinateList is an object which will be useful for data collection and use.


CoordinateList get_csv_coordinates()	// This method allows the data collection from the CSV file and organizes them in a CoordinateList
{
    CoordinateList coordinates = { .nb_pts = 0 };	// Initialization of the variable coordinates of type CoordinateList, and initialization of nb_pts at 0

    FILE *file = fopen("/home/visionhub_ws/src/voxl-vision-hub/coordinates.csv", "r"); // Opening in reading mode of the CSV file
    if (!file) {
        perror("The program cannot read the file");
        return coordinates;
    }

    char line[MAX_LINE];	// creation of a table of string with 256 characters
    
    while (fgets(line, MAX_LINE, file)) {		// While there is a readable line, it reads every line and puts the data in the list of list
        line[strcspn(line, "\n")] = '\0'; 		// Erases the last character "\n" of each line and replaces it by "\0"

        char *token = strtok(line, ",");		// Splits the columns on each line by a ","
        if (token != NULL) {
            coordinates.coorx[coordinates.nb_pts] = atof(token);  		// Adds the coordinate x in the table coordinates.coorx

            token = strtok(NULL, ",");
            if (token != NULL) {
                coordinates.coory[coordinates.nb_pts] = atof(token);	// Adds the coordinate y in the table coordinates.coory
            }

            token = strtok(NULL, ",");
            if (token != NULL) {
                coordinates.coorz[coordinates.nb_pts] = atof(token);	// Adds the coordinate z in the table coordinates.coorz
            }

            coordinates.nb_pts++;				// Increments nb_pts counter for each points read
        }
    }

    fclose(file);		// close the CSV file

    // Print data of the points
    for (int i = 0; i < coordinates.nb_pts; i++) {
        printf("x : %f, y : %f, z : %f\n",
               coordinates.coorx[i],
               coordinates.coory[i],
               coordinates.coorz[i]);
    }
    return coordinates;
}


// follow a path with coordinates
static void _init_path_coordinate(void)
{
    CoordinateList list = get_csv_coordinates();		// Creates a CoordinateList with data from the CSV file

    int size = list.nb_pts;		// Gets the number of point in the CoordinateList

	if(size > 1){			// Verifies if there is at least one point

		int cpt = 0;		// Initializes cpt counter to 0 which will count the total number of sub-points
		float dist; 		// We suppose that the coordinates are in meters 
		int nb_sub_pts; 	// Creates an int nb_sub_pts which correspond to the number of sub-points between 2 points
		float v = 0.1;  	// velocity = 0.1m/s
		float a = 0.0;  	// acceleration = null

		for(int i=0; i<size-1; i++){ 		// Loop through the list of coordinates
			
			dist = sqrt(pow((list.coorx[i+1]-list.coorx[i]), 2) + 		// We calculate the distance in meter between 2 points
                        pow((list.coory[i+1]-list.coory[i]), 2) + 
                        pow((list.coorz[i+1]-list.coorz[i]), 2));
			nb_pts = floor(dist*50); 		// We choose arbitrarily to put 50 points for each meter of the path

			for(int k=0; k<nb_sub_pts; k++){		// For each sub-points between the 2 same points : 

				// Initialization of the table "path", which will contain all sub-points of the path that the drone will follow
				path[cpt+k].time_boot_ms = 0;
				path[cpt+k].coordinate_frame = MAV_FRAME_LOCAL_NED;
				path[cpt+k].type_mask = 0; 
				path[cpt+k].target_system = 0; 
				path[cpt+k].target_component = AUTOPILOT_COMPID;

				// Adds a new sub-point of coordinate x, y and z in the path
				path[cpt+k].x = (list.coorx[i+1]-list.coorx[i])*k/nb_sub_pts + list.coorx[i];	
				path[cpt+k].y = (list.coory[i+1]-list.coory[i])*k/nb_sub_pts + list.coory[i];
				path[cpt+k].z = (list.coorz[i+1]-list.coorz[i])*k/nb_sub_pts + list.coorz[i];

				// Adds the velocity of the drone for this new sub-point
				path[cpt+k].vx = v;
				path[cpt+k].vy = v;
				path[cpt+k].vz = v;

				// Adds the acceleration of the drone for this new sub-point
				path[cpt+k].ax = a;
				path[cpt+k].ay = a;
				path[cpt+k].az = a;
				
				// Adds the yaw of the drone for this new sub-point
				path[cpt+k].yaw = 0;
			}

			cpt = cpt + nb_sub_pts;		// Incrementation of the counter 
		}
	}

	else{
		printf("error : the coordinates should contains at least 2 points") ;
	}

    // now set home position
	// this will move later if figure_eight_move_home is enabled
	home_position.time_boot_ms = 0;
	home_position.coordinate_frame = path[0].coordinate_frame;
	home_position.type_mask =   POSITION_TARGET_TYPEMASK_VX_IGNORE |
								POSITION_TARGET_TYPEMASK_VY_IGNORE |
								POSITION_TARGET_TYPEMASK_VZ_IGNORE |
								POSITION_TARGET_TYPEMASK_AX_IGNORE |
								POSITION_TARGET_TYPEMASK_AY_IGNORE |
								POSITION_TARGET_TYPEMASK_AZ_IGNORE |
								POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;
	home_position.x = 0.0f;
	home_position.y = 0.0f;
	home_position.z = path[0].z;
	home_position.yaw = path[0].yaw;
	home_position.target_system = 0; // will reset later when sending
	home_position.target_component = AUTOPILOT_COMPID;

    return;
}

// From here, the code is the same as the offboard_figure_eight.c file, we only changed the name of methods to include "coordinate" 

static void _send_position_in_path(int i)
{
	if(i>=STEPS || i<0) return;

	mavlink_set_position_target_local_ned_t pos = path[i];
	if(coordinate_move_home){
		pos.x += home_position.x;
		pos.y += home_position.y;
		pos.z  = home_position.z;
	}
	mavlink_io_send_fixed_setpoint(autopilot_monitor_get_sysid(),VOXL_COMPID,pos);

	return;
}



static void _send_home_position(void)
{
	if(coordinate_move_home){
		mavlink_odometry_t odom = autopilot_monitor_get_odometry();
		home_position.x = odom.x;
		home_position.y = odom.y;
		home_position.z = odom.z;
	}
	mavlink_io_send_fixed_setpoint(autopilot_monitor_get_sysid(),VOXL_COMPID,home_position);

	return;
}



static void* _offboard_coordinate_thread_func(__attribute__((unused)) void* arg)
{
	// for loop sleep
	int64_t next_time = 0;

	int i;

	_init_path_coordinate();

	//send a few setpoints before starting
	for(i = 100; running && i > 0; --i){
		_send_home_position();
		if(my_loop_sleep(RATE, &next_time)){
			fprintf(stderr, "WARNING thread fell behind\n");
		}
	}


HOME:
	// wait for the system to be armed and in offboard mode
	while(running && !autopilot_monitor_is_armed_and_in_offboard_mode()){
		_send_home_position();
		if(my_loop_sleep(RATE, &next_time)){
			fprintf(stderr, "WARNING thread fell behind\n");
		}
		fflush(stdout);i
	}

	// give the system 2 seconds to get to home position
	i = RATE * 2;
	//while(running && i>0){
	while(running && i>0){
		// return to home position if px4 falls out of offboard mode or disarms
		if(!autopilot_monitor_is_armed_and_in_offboard_mode()) goto HOME;
		i--;
		_send_home_position();
		if(my_loop_sleep(RATE, &next_time)){
			fprintf(stderr, "WARNING thread fell behind\n");
		}
	}

	// now begin path,
	i=0;
	while(running){
		// return to home position if px4 falls out of offboard mode or disarms
		if(!autopilot_monitor_is_armed_and_in_offboard_mode()) goto HOME;
		_send_position_in_path(i);
		i++;
		if(i>=STEPS) i=0;
		if(my_loop_sleep(RATE, &next_time)){
			fprintf(stderr, "WARNING thread fell behind\n");
		}
	}

	printf("exiting offboard thread\n");
	return NULL;
}



int offboard_coordinate_init(void)
{
	running = 1;
	pipe_pthread_create(&offboard_coordinate_thread_id, _offboard_coordinate_thread_func, NULL, OFFBOARD_THREAD_PRIORITY);
	return 0;
}



int offboard_coordinate_stop(int blocking)
{
	if(running==0) return 0;
	running = 0;
	if(blocking){
		pthread_join(offboard_coordinate_thread_id, NULL);
	}
	return 0;
}



void offboard_coordinate_en_print_debug(int debug)
{
	if(debug) en_debug = 1;
}
