#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <math.h>
#include <string.h>

#include "config_file.h"
#include "mavlink_io.h"
#include "autopilot_monitor.h"
#include "geometry.h"
#include "macros.h"
#include "misc.h"
#include "offboard_lines.h"

#define RATE 30
#define STEPS 25000
#define MAX_LINE 256
#define CSV_PATH "/data/path_points.csv" //Change to .CSV location

static int running = 0;
static pthread_t thread_id;
static int en_debug = 0;

static mavlink_set_position_target_local_ned_t path[STEPS];
static mavlink_set_position_target_local_ned_t home_position;

typedef struct {
    float coorx[STEPS];
    float coory[STEPS];
    float coorz[STEPS];
    int nb_pts;
} CoordinateList;

typedef struct {
    int id;
    float x, y, z;
    float yaw_deg;
} apriltag_pose_t;

apriltag_pose_t tag_map[50];
int tag_map_count = 0;

int load_apriltag_map(const char* path) {
    FILE* fp = fopen(path, "r");
    if (!fp) {
        printf("ERROR: could not open %s\n", path);
        return -1;
    }

    tag_map_count = 0;
    char line[128];
    while (fgets(line, sizeof(line), fp) && tag_map_count < 50) {
        if (line[0] == '#' || strlen(line) < 3) continue;
        if (sscanf(line, "%d,%f,%f,%f,%f",
            &tag_map[tag_map_count].id,
            &tag_map[tag_map_count].x,
            &tag_map[tag_map_count].y,
            &tag_map[tag_map_count].z,
            &tag_map[tag_map_count].yaw_deg) == 5) {
            tag_map_count++;
        }
    }

    fclose(fp);
    printf("Loaded %d tag poses\n", tag_map_count);
    return 0;
}

static CoordinateList load_csv_coordinates()
{
    CoordinateList coordinates = { .nb_pts = 0 };
    FILE *file = fopen(CSV_PATH, "r");
    if (!file) {
        perror("Could not open CSV file");
        return coordinates;
    }

    char line[MAX_LINE];
    while (fgets(line, MAX_LINE, file)) {
        char *token = strtok(line, ",");
        if (token) {
            coordinates.coorx[coordinates.nb_pts] = atof(token);
            token = strtok(NULL, ",");
        }
        if (token) {
            coordinates.coory[coordinates.nb_pts] = atof(token);
            token = strtok(NULL, ",");
        }
        if (token) {
            coordinates.coorz[coordinates.nb_pts] = atof(token);
        }
        coordinates.nb_pts++;
    }

    fclose(file);
    return coordinates;
}

static void generate_path_from_csv()
{
    CoordinateList list = load_csv_coordinates();
    int cpt = 0;
    float v = 0.1f;
    float a = 0.0f;

    for (int i = 0; i < list.nb_pts - 1; ++i) {
        float dx = list.coorx[i+1] - list.coorx[i];
        float dy = list.coory[i+1] - list.coory[i];
        float dz = list.coorz[i+1] - list.coorz[i];
        float dist = sqrtf(dx*dx + dy*dy + dz*dz);
        int nb_pts = floorf(dist * 50);
        if (nb_pts < 1) nb_pts = 1;

        for (int k = 0; k < nb_pts && cpt < STEPS; ++k) {
            path[cpt].time_boot_ms = 0;
            path[cpt].coordinate_frame = MAV_FRAME_LOCAL_NED;
            path[cpt].type_mask = 0;
            path[cpt].target_system = 0;
            path[cpt].target_component = AUTOPILOT_COMPID;

            path[cpt].x = dx * k / nb_pts + list.coorx[i];
            path[cpt].y = dy * k / nb_pts + list.coory[i];
            path[cpt].z = dz * k / nb_pts + list.coorz[i];

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

    generate_path_from_csv();

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
    load_apriltag_map("/data/tag_map.csv");
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
