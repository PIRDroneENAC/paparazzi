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
#include <math.h>
#include "multi/follow_human.h"
#include "generated/airframe.h"
#include "generated/flight_plan.h"

#include "subsystems/navigation/waypoints.h"

#include "state.h"
#include "pprzlink/messages.h"
#include "pprzlink/dl_protocol.h"
#include "firmwares/rotorcraft/navigation.h"
#include "subsystems/datalink/datalink.h"

#define ANGLE_OK RadOfDeg(5)
#define TAILLE_HUMAN 1.8
#define HALF_APERTURE RadOfDeg(15)
#define MIN_ALT 2.
#define MIN_PITCH RadOfDeg(-10)
#define MAX_PITCH RadOfDeg(10)
#define DISTANCE_TARGET 7.
#define KP_SPEED 0.8
#define KI_SPEED 0.
#define KD_SPEED 1
#define K_SPEED_CONS 0.5
#define K_PITCH 0.295
#define DIST_TOLERANCE 2.

humanGpsData dataHuman[NB_HUMAN_POS];
uint8_t data_pointer;
bool_t newData = FALSE;
float estimated_human_speed;
float time_last_gps;

float _speedCmd;
float _speedCons;
float _pitchCmd;

bool_t follow_human_init(void) {
  data_pointer = 0;
  int i;
  for(i=0;i<NB_HUMAN_POS;i++) {
    //dataHuman[i].enu.x = 0;
    //dataHuman[i].enu.y = 0;
    //dataHuman[i].enu.z = 0;
    dataHuman[i].speed = 0;
    
    dataHuman[i].enu.x = 0;
    dataHuman[i].enu.y = 300;
    dataHuman[i].enu.z = 0;
  }
  estimated_human_speed = 0.;
  time_last_gps = 0.;
  return FALSE;
}

bool_t handle_new_human_pos(unsigned char *buffer) {
  //get coordinate from pprzlink...
  struct LlaCoor_i lla_i;
  lla_i.lat = DL_HUMAN_GPS_lat(buffer);
  lla_i.lon = DL_HUMAN_GPS_lon(buffer);
  lla_i.alt = DL_HUMAN_GPS_alt(buffer);
  //convert it in LlaCoor_f...
  struct LlaCoor_f lla_f;
  LLA_FLOAT_OF_BFP(lla_f,lla_i);
  //and convert it in enu coordinate.
  enu_of_lla_point_f(&(dataHuman[data_pointer % NB_HUMAN_POS].enu), &(state.ned_origin_f), &lla_f);
  dataHuman[data_pointer % NB_HUMAN_POS].speed = DL_HUMAN_GPS_speed(buffer)/100.0;
  data_pointer++;
  
  
  
  humanGpsData dataPred;
  struct EnuCoor_f * human_pos = &(dataHuman[data_pointer % NB_HUMAN_POS].enu);
  getHumanPos(&dataPred, 0);
  float dist = sqrt(pow(human_pos->x - dataPred.enu.x,2) + pow(human_pos->y - dataPred.enu.y,2));
  float time_now = get_sys_time_float();
  float speed_estimated = dist/(time_now - time_last_gps);
  estimated_human_speed = 0.2 * estimated_human_speed + 0.8 * speed_estimated;
  time_last_gps = time_now;
  printf("est speed: %f\t time:%f\n", speed_estimated, time_now);
  
  //printf("FOLLOW : new human gps data : x= %f    y= %f    z= %f    speed = %f\n", dataHuman[data_pointer % NB_HUMAN_POS].enu.x,
  //									    dataHuman[data_pointer % NB_HUMAN_POS].enu.y,
  //									    dataHuman[data_pointer % NB_HUMAN_POS].enu.z,
  //									    dataHuman[data_pointer % NB_HUMAN_POS].speed);fflush(stdout);
  newData = TRUE;	//used to prevent periodic function that a new data is arrived.
  //move the HUMAN waypoint for visualisation
  struct EnuCoor_i human_pos_i;
  human_pos_i.x= POS_BFP_OF_REAL(dataHuman[data_pointer % NB_HUMAN_POS].enu.x);
  human_pos_i.y= POS_BFP_OF_REAL(dataHuman[data_pointer % NB_HUMAN_POS].enu.y);
  human_pos_i.z= POS_BFP_OF_REAL(dataHuman[data_pointer % NB_HUMAN_POS].enu.z);
  waypoint_move_enu_i(WP_HUMAN, &human_pos_i);
  
  return FALSE;
}

int getHumanPos(humanGpsData *data, uint8_t i) {
  if(i>NB_HUMAN_POS) {
    return -1;
  }
  *data = dataHuman[(data_pointer + i) % NB_HUMAN_POS];
  return 0;
}

int getEstimatedHumanPos(humanGpsData *data) {
	humanGpsData prev1;
	getHumanPos(&prev1,0);	//last value
	humanGpsData prev2;
	getHumanPos(&prev2,0);	//second to last value
	float dx = prev1.enu.x - prev2.enu.x;
	float dy = prev1.enu.y - prev2.enu.y;
	float angle = atan2(dy,dx);
	float diff_time = time_last_gps - get_sys_time_float();
	float dxn = sin(angle) * estimated_human_speed * diff_time;
	float dyn = cos(angle) * estimated_human_speed * diff_time;
	float newX = prev1.enu.x + dxn;
	float newY = prev1.enu.y + dyn;
	printf("dx:%f\tdy:%f\n", dxn, dyn);

	data->enu.x = newX;
	data->enu.y = newY;

	struct EnuCoor_i human_pos_est_i;
	human_pos_est_i.x= POS_BFP_OF_REAL(newX);
	human_pos_est_i.y= POS_BFP_OF_REAL(newY);
	human_pos_est_i.z= POS_BFP_OF_REAL(prev1.enu.z);
    waypoint_move_enu_i(WP_HUMAN_ESTIMATED, &human_pos_est_i);

	return 0;
}


void calc_speed_cmd(float speedLong){
    static int count=0;
    static float errInt=0;
    static float err_old=0;
    if(!(count%5)) {
        float err = _speedCons - speedLong;
        errInt += err;
        _speedCmd = KP_SPEED*err + KI_SPEED * errInt + KD_SPEED * (err-err_old);
        err_old = err;
    }
}

void calc_speed_cons(float distance){
    static unsigned int count=0;
    if(!(count%5)) {
        _speedCons = K_SPEED_CONS * (distance - DISTANCE_TARGET);	//+ human speed
    }
    count++;
    
    //if(count%500 < 250){ _speedCons=2.5;} else {_speedCons=-2.5;};
}

void calc_pitch() {
    float pitchcmd = 0.90*_pitchCmd + 0.1*(-K_PITCH * _speedCmd);
    _pitchCmd = CLAMP(pitchcmd, MIN_PITCH, MAX_PITCH);
}

float optimal_alt(float pitch, float distance) {
    float hmin = TAILLE_HUMAN-distance*tan(pitch+HALF_APERTURE);
    float hmax = distance*tan(HALF_APERTURE-pitch);
    float h= Max(0.05*hmin+0.95*hmax,MIN_ALT);
    return h;
}

bool_t follow_human_periodic(void) {
//begin when entered in the follow block, and continue until FALSE is returned.
    ///move in initialization function
    horizontal_mode = HORIZONTAL_MODE_ATTITUDE;
    vertical_mode = VERTICAL_MODE_ALT;
    //set orientation of the drone towards the human to follow
    humanGpsData human_pos;
    //getHumanPos(&human_pos, 0);
    getEstimatedHumanPos(&human_pos);
    struct EnuCoor_f *pos_drone = stateGetPositionEnu_f();
    //struct FloatEulers * attitude_drone = stateGetNedToBodyEulers_f();
    
    nav_set_heading_towards(human_pos.enu.x, human_pos.enu.y);

    //if (ABS(attitude_drone->psi-ANGLE_FLOAT_OF_BFP(nav_heading)) <= ANGLE_OK) {

    float distance = sqrt(pow(human_pos.enu.x-pos_drone->x,2)+pow(human_pos.enu.y-pos_drone->y,2));
    //float dalt = Max(0,human_pos.enu.z) - pos_drone->z;
    float speedNorm = stateGetHorizontalSpeedNorm_f();
    float speedDir = stateGetHorizontalSpeedDir_f();
    
    float speedLat = speedNorm*sin(speedDir - ANGLE_FLOAT_OF_BFP(nav_heading));
    nav_roll = ANGLE_BFP_OF_REAL(-0.06 * speedLat);
    
    float speedLong = speedNorm*cos(speedDir - ANGLE_FLOAT_OF_BFP(nav_heading));
    calc_speed_cons(distance);
    calc_speed_cmd(speedLong);
    calc_pitch();
    
    //printf("speedCons: %f\tspeedCMD:%f\tpitchCmd : %f\tdist:%f\n",_speedCons,_speedCmd, _pitchCmd, distance);
    //cons	read	cmd
    //printf("%.2f\t%.2f\t%.2f\n",_speedCons, speedLong, _speedCmd);
    
    
    //float altitude = optimal_alt(attitude_drone->theta, distance);

    nav_pitch = ANGLE_BFP_OF_REAL(_pitchCmd);

    //nav_flight_altitude = POS_BFP_OF_REAL(altitude);


    //POS_FLOAT_OF_BFP()
    //state.ned_origin_f.hmsl : altitude du terrain
    
    //truct EnuCoor_f* stateGetSpeedEnu_f()
    

    //nav_flight_altitude = POS_BFP_OF_REAL(10);		//in meters, above the ground level
    
    
    
    
    //printf("x= %f    y=%f    z=%f\n", pos_drone->x, pos_drone->y, pos_drone->z);
    //struct FloatEulers * attitude = stateGetNedToBodyEulers_f();
    //nav_flight_altitude = POS_BFP_OF_REAL(18);
    //navigation_SetFlightAltitude(180);
    //static int count = 0;
    //printf("periodic nav_alt = %d hmsl = %f  phi = %f  psi = %f    theta = %f\n", nav_altitude, state.ned_origin_f.hmsl, attitude->phi, attitude->psi, attitude->theta);
    //count++;
    //if(count > 200){
    //    count = 0;
    //    return FALSE;
    //}
    return TRUE;
}

