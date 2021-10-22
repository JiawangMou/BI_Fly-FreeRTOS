/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
 *
 * This file is part of paparazzi.
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/** @file firmwares/rotorcraft/stabilization/stabilization_attitude.h
 *  General attitude stabilization interface for rotorcrafts.
 *  The actual implementation is automatically included.
 */

#ifndef STABILIZATION_ATTITUDE_H
#define STABILIZATION_ATTITUDE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "std.h"
#include "airframe.h"
#include "pprz_algebra_int.h"
#include "stabilization_attitude_quat_float.h"

#include "stabilizer_types.h"

extern void stabilization_attitude_init(void);
extern void stabilization_attitude_enter(float heading);    // input heading in degrees
extern void stabilization_attitude_quit();
extern void stabilization_attitude_run(control_t *control, sensorData_t *sensor, state_t *state, setpoint_t *setpoint);
extern void paparazziUpdatePIDParams(void);

extern int32_t stabilization_cmd[COMMANDS_NB];

typedef struct {
  float roll, pitch, yaw;
  float err_angle[3];
  float err_rate[3];
  float err_rate_dd[3];
  float u[3];
} quat_control_debug_t;

extern quat_control_debug_t quat_control_debug;

#ifdef __cplusplus
}
#endif

#endif /* STABILIZATION_ATTITUDE_H */
