#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <errno.h>
#include <unistd.h>

#include "config_file.h"
#include "mavlink_io.h"
#include "autopilot_monitor.h"
#include "geometry.h"
#include "macros.h"
#include "offboard_figure_eight.h"
#include "offboard_mode.h"
#include "offboard_shomer.h"
#include "misc.h"

#define FLIGHT_ALTITUDE	-1.5f
#define RATE			30	// loop rate hz
#define RADIUS			1.0	// radius of figure 8 in meters
#define CYCLE_S			12	// time to complete one figure 8 cycle in seconds
#define STEPS			(CYCLE_S*RATE)

static int running = 0;
static pthread_t offboard_figure_eight_thread_id;
static int en_debug = 0;

static mavlink_set_position_target_local_ned_t path[STEPS];
static mavlink_set_position_target_local_ned_t home_position;
static int path_initialized = 0; // Track if the path has been initialized

#define RECTANGLE_LENGTH 0.4// Length of the rectangle
#define RECTANGLE_WIDTH 0.4   // Width of the rectangle

static void _init_path(void)
{
	int i;
	const double dt = 1.0 / RATE;
	const int side_steps = STEPS / 4; // Number of steps per side
	const double length_step = RECTANGLE_LENGTH / side_steps;
	const double width_step = RECTANGLE_WIDTH / side_steps;

	for (i = 0; i < STEPS; i++)
	{
		path[i].time_boot_ms = 0;
		path[i].coordinate_frame = MAV_FRAME_LOCAL_NED;
		path[i].type_mask = 0;
		path[i].target_system = 0;
		path[i].target_component = AUTOPILOT_COMPID;

		if (i < side_steps)
		{
			// First side (length)
			path[i].x = i * length_step;
			path[i].y = 0;
		}
		else if (i < 2 * side_steps)
		{
			// Second side (width)
			path[i].x = RECTANGLE_LENGTH;
			path[i].y = (i - side_steps) * width_step;
		}
		else if (i < 3 * side_steps)
		{
			// Third side (length)
			path[i].x = RECTANGLE_LENGTH - (i - 2 * side_steps) * length_step;
			path[i].y = RECTANGLE_WIDTH;
		}
		else
		{
			// Fourth side (width)
			path[i].x = 0;
			path[i].y = RECTANGLE_WIDTH - (i - 3 * side_steps) * width_step;
		}

		path[i].z = FLIGHT_ALTITUDE;

		// Set velocity
		if (i % side_steps == 0)
		{
			path[i].vx = 0;
			path[i].vy = 0;
		}
		else if (i < side_steps)
		{
			path[i].vx = length_step / dt;
			path[i].vy = 0;
		}
		else if (i < 2 * side_steps)
		{
			path[i].vx = 0;
			path[i].vy = width_step / dt;
		}
		else if (i < 3 * side_steps)
		{
			path[i].vx = -length_step / dt;
			path[i].vy = 0;
		}
		else
		{
			path[i].vx = 0;
			path[i].vy = -width_step / dt;
		}

		path[i].vz = 0.0f;

		// Set acceleration to zero
		path[i].afx = 0.0f;
		path[i].afy = 0.0f;
		path[i].afz = 0.0f;

		// Calculate yaw as direction of velocity
		path[i].yaw = atan2(path[i].vy, path[i].vx);
	}

	// Calculate yaw_rate by dirty differentiating yaw
	for (i = 0; i < STEPS; i++)
	{
		double next = path[(i + 1) % STEPS].yaw;
		double curr = path[i].yaw;
		if ((next - curr) < -PI) next += (TWO_PI);
		if ((next - curr) > PI) next -= (TWO_PI);
		path[i].yaw_rate = (next - curr) / dt;
	}

	if (en_debug)
	{
		printf("===========================================================\n");
		printf("   X     Y\n");
		for (i = 0; i < STEPS; i++)
		{
			printf("x:%7.3f  y:%7.3f\n", (double)path[i].x, (double)path[i].y);
		}
		printf("===========================================================\n");
		printf("   vx    dx/dt     vy    dy/dt\n");
		for (i = 0; i < STEPS; i++)
		{
			double dx = (double)(path[(i + 1) % STEPS].x - path[i].x) / dt;
			double dy = (double)(path[(i + 1) % STEPS].y - path[i].y) / dt;
			printf("vx:%7.3f dx/dt:%7.3f  vy:%7.3f dy/dt:%7.3f\n", (double)path[i].vx, dx, (double)path[i].vy, dy);
		}
		printf("===========================================================\n");
		printf("   ax    d^2x/dt     ay    d^2y/dt\n");
		for (i = 0; i < STEPS; i++)
		{
			double d2x = (double)(path[(i + 1) % STEPS].vx - path[i].vx) / dt;
			double d2y = (double)(path[(i + 1) % STEPS].vy - path[i].vy) / dt;
			printf("Ax:%7.3f d2x/dt:%7.3f  Ay:%7.3f d2y/dt:%7.3f\n", (double)path[i].afx, d2x, (double)path[i].afy, d2y);
		}
		printf("===========================================================\n");
		printf("   yaw     yaw_rate\n");
		for (i = 0; i < STEPS; i++)
		{
			printf("yaw:%7.1f deg  yaw_rate: %7.1f deg/s\n", (double)(path[i].yaw) * 180.0 / PI, (double)(path[i].yaw_rate) * 180.0 / PI);
		}
		printf("===========================================================\n");
	}

	// Set home position
	home_position.time_boot_ms = 0;
	home_position.coordinate_frame = path[0].coordinate_frame;
	home_position.type_mask = POSITION_TARGET_TYPEMASK_VX_IGNORE |
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
	home_position.target_system = 0;
	home_position.target_component = AUTOPILOT_COMPID;
	
	path_initialized = 1; // Mark the path as initialized
	return;
}

// Function to reset the path when coming back to figure eight mode
static void _reset_path(void)
{
	// Clear the path to ensure no old data remains
	memset(path, 0, sizeof(path));
	path_initialized = 0;
}

static void _send_position_in_path(int i)
{
	if(i>=STEPS || i<0) return;

	mavlink_set_position_target_local_ned_t pos = path[i];
	if(figure_eight_move_home){
		pos.x += home_position.x;
		pos.y += home_position.y;
		pos.z  = home_position.z;
	}
	mavlink_io_send_fixed_setpoint(autopilot_monitor_get_sysid(),VOXL_COMPID,pos);

	return;
}

static void _send_home_position(void)
{
	if(figure_eight_move_home){
		mavlink_odometry_t odom = autopilot_monitor_get_odometry();
		home_position.x = odom.x;
		home_position.y = odom.y;
		home_position.z = odom.z;
	}
	mavlink_io_send_fixed_setpoint(autopilot_monitor_get_sysid(),VOXL_COMPID,home_position);

	return;
}

// Function to check for shomer detection (for detecting tags)
// In a real application, this would use actual tag detection data
static int _check_for_shomer_detection(void)
{
	if (offboard_mode == FIGURE_EIGHT_WITH_SHOMER) {
		extern int detection_pending; // from shomer
		
		if (detection_pending) {
			printf("DETECTION_PENDING flag set - initiating mode switch to SHOMER\n");
			return 1;
		}
	}
	return 0;
}

static void* _offboard_figure_eight_thread_func(__attribute__((unused)) void* arg)
{
	// for loop sleep
	int64_t next_time = 0;

	int i;

	if (!path_initialized) {
		_init_path();
	}

	//send a few setpoints before starting
	for(i = 100; running && i > 0; --i){
		_send_home_position();
		if(my_loop_sleep(RATE, &next_time)){
			fprintf(stderr, "WARNING figure 8 thread fell behind\n");
		}
	}


HOME:
	// wait for the system to be armed and in offboard mode
	while(running && !autopilot_monitor_is_armed_and_in_offboard_mode()){
		_send_home_position();
		if(my_loop_sleep(RATE, &next_time)){
			fprintf(stderr, "WARNING figure 8 thread fell behind\n");
		}
		fflush(stdout);
	}

	// give the system 2 seconds to get to home position
	i = RATE * 2;
	//while(running && i>0){
	while(running && i>0){
		// return to home position if px4 falls out of offboard mode or disarms
		if(!autopilot_monitor_is_armed_and_in_offboard_mode()) goto HOME;
		
		// Check for shomer detection if in combined mode
		if (offboard_mode == FIGURE_EIGHT_WITH_SHOMER && _check_for_shomer_detection()) {
			printf("Shomer detection during figure eight - switching to shomer mode\n");
			_reset_path();
			printf("Figure eight path reset, initiating mode switch\n");
			offboard_mode_switch_figure_eight_shomer(SHOMER);
			printf("Mode switch completed, terminating figure eight thread\n");
			return NULL; // Exit thread cleanly
		}
		
		i--;
		_send_home_position();
		if(my_loop_sleep(RATE, &next_time)){
			fprintf(stderr, "WARNING figure 8 thread fell behind\n");
		}
	}

	// now begin figure 8 path,
	i=0;
	while(running){
		// return to home position if px4 falls out of offboard mode or disarms
		if(!autopilot_monitor_is_armed_and_in_offboard_mode()) goto HOME;
		
		// Check for shomer detection if in combined mode
		if (offboard_mode == FIGURE_EIGHT_WITH_SHOMER && _check_for_shomer_detection()) {
			printf("Shomer detection during figure eight - switching to shomer mode\n");
			_reset_path();
			printf("Figure eight path reset, initiating mode switch\n");
			offboard_mode_switch_figure_eight_shomer(SHOMER);
			printf("Mode switch completed, terminating figure eight thread\n");
			return NULL; // Exit thread cleanly
		}
		
		_send_position_in_path(i);
		i++;
		if(i>=STEPS) i=0;
		if(my_loop_sleep(RATE, &next_time)){
			fprintf(stderr, "WARNING figure 8 thread fell behind\n");
		}
	}

	printf("exiting offboard figure eight thread\n");
	return NULL;
}


int offboard_figure_eight_init(void)
{
	running = 1;
	pipe_pthread_create(&offboard_figure_eight_thread_id, _offboard_figure_eight_thread_func, NULL, OFFBOARD_THREAD_PRIORITY);
	return 0;
}


int offboard_figure_eight_stop(int blocking)
{
	if(running==0) return 0;
	
	printf("Stopping figure eight thread (running=%d)\n", running);
	running = 0;
	_reset_path(); // Clean up path on stop
	
	if(blocking){
		if(!pthread_equal(pthread_self(), offboard_figure_eight_thread_id)){
			printf("Waiting for figure eight thread to join...\n");
			int ret = pthread_join(offboard_figure_eight_thread_id, NULL);
			if (ret != 0) {
				fprintf(stderr, "Error joining figure eight thread: %d\n", ret);
				return -1;
			}
			printf("Figure eight thread joined successfully\n");
		}
		else{
			printf("Figure eight stop called from within thread; skipping join to avoid deadlock\n");
		}
	}
	
	return 0;
}

void offboard_figure_eight_en_print_debug(int debug)
{
	if(debug) en_debug = 1;
}
