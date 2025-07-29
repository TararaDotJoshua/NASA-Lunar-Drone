#ifndef OFFBOARD_COORDINATE_H
#define OFFBOARD_COORDINATE_H


int offboard_coordinate_init(void);
int offboard_coordinate_stop(int blocking);

/**
 * @brief      enable or disable the printing of debug messages
 *
 * @param[in]  debug  0 to disable, nonzero to enable
 */
void offboard_coordinate_en_print_debug(int debug);


#endif // end #define OFFBOARD_COORDINATE_H