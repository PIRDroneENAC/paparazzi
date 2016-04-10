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

typedef struct humanGpsData {
  //uint8_t id;
  struct LlaCoor_i lla;
  //int32_t course;
}humanGpsData;

extern bool_t follow_human_init(void);
extern bool_t handle_new_human_pos(unsigned char *);
int getHumanPos(humanGpsData *data, uint8_t i);


#define ParseHumanGps() { \
    handle_new_human_pos(dl_buffer); \
  }

#endif

