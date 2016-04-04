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

humanGpsData dataHuman[NB_HUMAN_POS];
uint8_t data_pointer;

bool_t follow_human_init(void) {
  data_pointer = 0;
  int i;
  for(i=0;i<NB_HUMAN_POS;i++) {
    dataHuman[i].lla.alt = 0;
    dataHuman[i].lla.lat = 0;
    dataHuman[i].lla.lon = 0;
  }
  
  return FALSE;
}

bool_t handle_new_human_pos() {
  printf("FOLLOW : new gps data received !\n");fflush(stdout);
  unsigned char *buffer = dl_buffer;
  
  humanGpsData pos_human;
  pos_human.lla.lat = DL_HUMAN_GPS_lat(buffer);
  pos_human.lla.lon = DL_HUMAN_GPS_lon(buffer);
  pos_human.lla.alt = DL_HUMAN_GPS_alt(buffer);
  setLastHumanPos(pos_human);
  
  return FALSE;
}

int getHumanPos(humanGpsData *data, uint8_t i) {
  if(i>NB_HUMAN_POS) {
    return -1;
  }
  data = &(dataHuman[(data_pointer + i) % NB_HUMAN_POS]);
  return 0;
}

int setLastHumanPos(humanGpsData data) {
  dataHuman[data_pointer % NB_HUMAN_POS] = data;
  return 0;
}

