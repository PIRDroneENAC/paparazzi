/*
 * Copyright (C) Fabien Bonneval
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/multi/follow_human.c"
 * @author Fabien Bonneval
 * Allow to follow a human
 */
 
#define NB_HUMAN_POS 10
#include <stdio.h>
#include "multi/follow_human.h"
#include "generated/airframe.h"
#include "generated/flight_plan.h"

#include "subsystems/navigation/waypoints.h"

#include "state.h"
#include "pprzlink/messages.h"
#include "pprzlink/dl_protocol.h"
#include "firmwares/rotorcraft/navigation.h"
#include "subsystems/datalink/datalink.h"

struct humanGpsData dataHuman[NB_HUMAN_POS];
unsigned int data_pointer;

bool_t follow_human_init(void) {
  data_pointer = 0;
  int i;
  for(i=0;i<NB_HUMAN_POS;i++) {
    //dataHuman[i] = 0;
  }
  nav_set_heading_deg(75);
  return FALSE;
}

bool_t handle_new_human_pos() {
  printf("FOLLOW : new gps data received !\n");fflush(stdout);
  unsigned char *buffer = dl_buffer;
  struct humanGpsData pos_human;
  //pos_human.id = DL_HUMAN_GPS_human_id(buffer);
  pos_human.lla.lat = DL_HUMAN_GPS_lat(buffer);
  pos_human.lla.lon = DL_HUMAN_GPS_lon(buffer);
  pos_human.lla.alt = DL_HUMAN_GPS_alt(buffer);
  //pos_human.course = DL_HUMAN_GPS_course(buffer);
  /*  
  // Translate to ENU
  enu_of_ecef_point_i(&enu, &state.ned_origin_i, &new_pos);
  INT32_VECT3_SCALE_2(enu, enu, INT32_POS_OF_CM_NUM, INT32_POS_OF_CM_DEN);
  */
  //nav_set_heading_deg(135);
  // Move the waypoint
  waypoint_set_lla (pos_human.id, &(pos_human.lla));
  
  return FALSE;
}

