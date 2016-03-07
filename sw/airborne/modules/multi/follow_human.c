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

#include "multi/follow_human.h"
#include "generated/airframe.h"
#include "generated/flight_plan.h"

#include "subsystems/navigation/waypoints.h"

#include "state.h"
#include "pprzlink/messages.h"
#include "pprzlink/dl_protocol.h"
#include "firmwares/rotorcraft/navigation.h"
#include "subsystems/datalink/datalink.h"


bool_t follow_human_init(void) {
  nav_set_heading_deg(75);
  return FALSE;
}
#include <stdio.h>
bool_t handle_new_human_pos() {

printf("bla\n");fflush(stdout);
/*
 * code de la fonction follow_change_wp du module follow
 */
  /*unsigned char *buffer = dl_buffer;
  struct EcefCoor_i new_pos;
  struct EnuCoor_i enu;
  new_pos.x = DL_REMOTE_GPS_ecef_x(buffer);
  new_pos.y = DL_REMOTE_GPS_ecef_y(buffer);
  new_pos.z = DL_REMOTE_GPS_ecef_z(buffer);
  
  // Translate to ENU
  enu_of_ecef_point_i(&enu, &state.ned_origin_i, &new_pos);
  INT32_VECT3_SCALE_2(enu, enu, INT32_POS_OF_CM_NUM, INT32_POS_OF_CM_DEN);

  // TODO: Add the angle to the north

  // Update the offsets
  enu.x += POS_BFP_OF_REAL(0);
  enu.y += POS_BFP_OF_REAL(0);
  enu.z += POS_BFP_OF_REAL(0);

  // TODO: Remove the angle to the north
  */
  nav_set_heading_deg(135);
  // Move the waypoint
  //waypoint_set_enu_i(99, &enu);
  
  return FALSE;
}

