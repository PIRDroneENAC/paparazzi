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
static int counter=0;

bool_t follow_human_init(void) {
  data_pointer = 0;
  int i;
  for(i=0;i<NB_HUMAN_POS;i++) {
    dataHuman[i].lla.alt = 0;
    dataHuman[i].lla.lat = 0;
    dataHuman[i].lla.lon = 0;
    dataHuman[i].speed = 0;
  }
  
  return FALSE;
}

bool_t handle_new_human_pos(unsigned char *buffer) {
  dataHuman[data_pointer % NB_HUMAN_POS].lla.lat = DL_HUMAN_GPS_lat(buffer);
  dataHuman[data_pointer % NB_HUMAN_POS].lla.lon = DL_HUMAN_GPS_lon(buffer);
  dataHuman[data_pointer % NB_HUMAN_POS].lla.alt = DL_HUMAN_GPS_alt(buffer);
  dataHuman[data_pointer % NB_HUMAN_POS].speed = DL_HUMAN_GPS_speed(buffer);
  data_pointer++;
  
  printf("FOLLOW : new gps data : lat = %d    lon = %d    alt = %d    speed = %d\n", dataHuman[data_pointer % NB_HUMAN_POS].lla.lat,dataHuman[data_pointer % NB_HUMAN_POS].lla.lon,dataHuman[data_pointer % NB_HUMAN_POS].lla.alt,dataHuman[data_pointer % NB_HUMAN_POS].speed);fflush(stdout);
  return FALSE;
}

int getHumanPos(humanGpsData *data, uint8_t i) {
  if(i>NB_HUMAN_POS) {
    return -1;
  }

  *data = dataHuman[(data_pointer + i) % NB_HUMAN_POS];
  return 0;
}
/*
int convertRad(input){
 return (Math.PI * input)/180;
}

int relative_position(gps_cible,gps_drone){
 lat_a_degre, lon_a_degre, lat_b_degre, lon_b_degre=gps_cible.lla.lat,gps_cible.lla.lon,drone.lla.lat,gps_drone.lla.lon
 R = 6378000; //Rayon de la terre en mètre
 lat_a = convertRad(lat_a_degre);
 lon_a = convertRad(lon_a_degre);
 lat_b = convertRad(lat_b_degre);
 lon_b = convertRad(lon_b_degre);
 d = R * (Math.PI/2 - Math.asin( Math.sin(lat_b) * Math.sin(lat_a) + Math.cos(lon_b - lon_a) * Math.cos(lat_b) * Math.cos(lat_a)));
 return d;
}

int optimal_pitch(gps_cible,gps_drone,d){
 lat_human, lon_human, lat_drone, lon_drone=gps_cible.lla.lat,gps_cible.lla.lon,drone.lla.lat,gps_drone.lla.lon;
 psi = cotan⁻¹(cos(lat_drone)tan(lat_human)/sin(long_human-long_drone)-sin(lat_human)/tan(long_human-long_drone));
 h=1,71 -90/100 *d*tan(psi);
 return psi,h;
}

int mission(){
 struct LlaCoor_i gps_drone;
 humanGPSData* gps_human;
 int distance,distance_target = 7,pitch,altitude;
  //Initialisation
  if (counter==0){
   //ask for the coordinate of drone and human 
   gps_drone = stateGetPositionLla_i();//coord.lla.lat coord.lon coord.alt
   getHumanPos(&gps_human,0); 
   distance = relative_position();//relative position drone-human
   set_heading_towards(gps_human.lla.lat,gps_human.lla.lon);//new heading_consigne =
   //placement du drone à distance-consigne
  }
  else{
   gps_drone = stateGetPositionLla_i();//coord.lla.lat coord.lon coord.alt
   getHumanPos(&gps_human,0); 
   distance = relative_position(gps_human,gps_drone);//relative position drone-human
   pitch,altitude = optimal_pitch(gps_human,gps_drone,distance);//calcul de pitch/altitude optimal
   if (abs(heading-heading_consigne)< = 5) {
    if (abs(distance-distance_traget) = 0) {
      //nav_roll = 0
    }
    else {
     if (abs(distance-distance_traget) < 0.5) {
     
     }
    }
   }*/
   
  /*TODO si à la bonne distance
  	 phi=0 (psi?)
  alors regarder si anticipation possible
     sinon
  si (d-dc) < 0,5 
  alors changer angle et altitude pour revenir à la bonne distance en restant dans le meme axe 
  si d-dc > 0,5  
  	alors augmenter angle et altitude pour suivre le drone
   }*/
  // recalculer le heading
/*  }
 }
 counter++;
}*/

