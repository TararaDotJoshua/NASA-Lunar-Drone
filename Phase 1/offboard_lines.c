#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <math.h>

#include "config_file.h"
#include "mavlink_io.h"
#include "autopilot_monitor.h"
#include "geometry.h"
#include "macros.h"
#include "misc.h"
#include "offboard_lines.h"

#define RATE 30
#define STEPS 25000

static int running = 0;
static pthread_t thread_id;
static int en_debug = 0;

static mavlink_set_position_target_local_ned_t path[STEPS];
static mavlink_set_position_target_local_ned_t home_position;

/**
 * @brief HARDCODED WAYPOINTS for the path
 * Edit these values to change the line segment path.
 */
static const float points[][3] = {
    {0.0, 0.0, -1.5},
    {1.0, 0.0, -1.5},
    {1.0, 1.0, -1.5},
    {0.0, 1.0, -1.5},
    {0.0, 0.0, -1.5}
};
static const int num_points = sizeof(points) / sizeof(points[0]);

static void init_line_segment_path()
{
    int cpt = 0;
    float v = 0.1f;  // velocity
    float a = 0.0f;  // acceleration

    for (int i = 0; i < num_points - 1; ++i) {
        float dx = points[i+1][0] - points[i][0];
        float dy = points[i+1][1] - points[i][1];
        float dz = points[i+1][2] - points[i][2];
        float dist = sqrtf(dx*dx + dy*dy + dz*dz);
        int nb_pts = floorf(dist * 50);

        for (int k = 0; k < nb_pts; ++k) {
            path[cpt].time_boot_ms = 0;
            path[cpt].coordinate_frame = MAV_FRAME_LOCAL_NED;
            path[cpt].type_mask = 0;
            path[cpt].target_system = 0;
            path[cpt].target_component = AUTOPILOT_COMPID;

            path[cpt].x = dx * k / nb_pts + points[i][0];
            path[cpt].y = dy * k / nb_pts + points[i][1];
            path[cpt].z = dz * k / nb_pts + points[i][2];

            path[cpt].vx = v;
            path[cpt].vy = v;
            path[cpt].vz = v;

            path[cpt].ax = a;
            path[cpt].ay = a;
            path[cpt].az = a;

            path[cpt].yaw = 0;

            cpt++;
        }
    }

    home_position = path[0];
    home_position.type_mask = POSITION_TARGET_TYPEMASK_VX_IGNORE |
                               POSITION_TARGET_TYPEMASK_VY_IGNORE |
                               POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                               POSITION_TARGET_TYPEMASK_AX_IGNORE |
                               POSITION_TARGET_TYPEMASK_AY_IGNORE |
                               POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                               POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;
}

static void send_position(int i)
{
    if (i >= STEPS || i < 0) return;

    mavlink_set_position_target_local_ned_t pos = path[i];
    if (coordinate_move_home) {
        pos.x += home_position.x;
        pos.y += home_position.y;
        pos.z = home_position.z;
    }
    mavlink_io_send_fixed_setpoint(autopilot_monitor_get_sysid(), VOXL_COMPID, pos);
}

static void send_home_position()
{
    if (coordinate_move_home) {
        mavlink_odometry_t odom = autopilot_monitor_get_odometry();
        home_position.x = odom.x;
        home_position.y = odom.y;
        home_position.z = odom.z;
    }
    mavlink_io_send_fixed_setpoint(autopilot_monitor_get_sysid(), VOXL_COMPID, home_position);
}

static void* thread_func(__attribute__((unused)) void* arg)
{
    int64_t next_time = 0;
    int i;

    init_line_segment_path();

    for (i = 100; running && i > 0; --i) {
        send_home_position();
        if (my_loop_sleep(RATE, &next_time)) fprintf(stderr, "WARNING thread fell behind\n");
    }

HOME:
    while (running && !autopilot_monitor_is_armed_and_in_offboard_mode()) {
        send_home_position();
        if (my_loop_sleep(RATE, &next_time)) fprintf(stderr, "WARNING thread fell behind\n");
    }

    i = RATE * 2;
    while (running && i > 0) {
        if (!autopilot_monitor_is_armed_and_in_offboard_mode()) goto HOME;
        send_home_position();
        i--;
        if (my_loop_sleep(RATE, &next_time)) fprintf(stderr, "WARNING thread fell behind\n");
    }

    i = 0;
    while (running) {
        if (!autopilot_monitor_is_armed_and_in_offboard_mode()) goto HOME;
        send_position(i++);
        if (i >= STEPS) i = 0;
        if (my_loop_sleep(RATE, &next_time)) fprintf(stderr, "WARNING thread fell behind\n");
    }

    printf("exiting offboard_lines thread\n");
    return NULL;
}

int offboard_lines_init(void)
{
    running = 1;
    pipe_pthread_create(&thread_id, thread_func, NULL, OFFBOARD_THREAD_PRIORITY);
    return 0;
}

int offboard_lines_stop(int blocking)
{
    if (!running) return 0;
    running = 0;
    if (blocking) pthread_join(thread_id, NULL);
    return 0;
}

void offboard_lines_en_print_debug(int debug)
{
    if (debug) en_debug = 1;
}
