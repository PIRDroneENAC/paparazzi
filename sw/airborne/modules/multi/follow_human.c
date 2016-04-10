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

bool_t handle_new_human_pos(unsigned char *buffer) {
  printf("FOLLOW : new gps data received !\n");fflush(stdout);
  
  dataHuman[data_pointer % NB_HUMAN_POS].lla.lat = DL_HUMAN_GPS_lat(buffer);
  dataHuman[data_pointer % NB_HUMAN_POS].lla.lon = DL_HUMAN_GPS_lon(buffer);
  dataHuman[data_pointer % NB_HUMAN_POS].lla.alt = DL_HUMAN_GPS_alt(buffer);
  data_pointer++;
  return FALSE;
}

int getHumanPos(humanGpsData *data, uint8_t i) {
  if(i>NB_HUMAN_POS) {
    return -1;
  }

  *data = dataHuman[(data_pointer + i) % NB_HUMAN_POS];
  return 0;
}

int mission(){
  //Initialisation de la position relative du drone avec la cible
  //if (getHumanPos(*data, i))==0){
   //calcul de la position relative du drone.
   //modification du cap (pour etre dans le champ visuel) heading_consigne =
   //placement du drone à distance-consigne 
  //}
 //while(1){
  //calcul de la distance drone/cible
  //calcul de psi/h optimal
  /*if (abs(heading-heading_consigne)< = (environ) l’optimal et h pareil 
  	alors 
  si à la bonne distance
  	phi=0
  alors regarder si anticipation possible
  		sinon
  si (d-dc) < 0,5 
  alors changer angle et altitude pour revenir à la bonne distance en restant dans le meme axe 
  si d-dc > 0,5  
  	alors augmenter angle et altitude pour suivre le drone
  sinon changer angle et altitude pour redonner le visuel*/
 //}
  
  
}

