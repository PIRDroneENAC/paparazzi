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
 * @file "modules/multi/follow_human.h"
 * @author Fabien Bonneval
 * Allow to follow a human
 */

#ifndef FOLLOW_HUMAN_H
#define FOLLOW_HUMAN_H
#include "math/pprz_geodetic_int.h"
#include "std.h"
#include "subsystems/datalink/datalink.h"

extern bool_t follow_human_init(void);
extern bool_t handle_new_human_pos(void);

struct humanGpsData {
  //uint8_t id;
  struct LlaCoor_i lla;
  //int32_t course;
};
/*
#define ParseHumanGps() { \
    if (DL_REMOTE_GPS_ac_id(dl_buffer) == FOLLOW_HUMAN_ID) { \
    } \
  }
*/
#endif

