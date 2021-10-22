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

/** @file firmwares/rotorcraft/stabilization/stabilization_attitude_quat_float.c
 * @brief Quaternion attitude stabilization (floating point).
 */

#include "airframe.h"

#include "stabilization_attitude.h"
#include "stabilization_attitude_quat_transformations.h"

#include "std.h"
#include "pprz_algebra_float.h"
#include "pprz_algebra_int.h"

#include "config_param.h"
#include "sensfusion6.h"

struct FloatAttitudeGains stabilization_gains[STABILIZATION_ATTITUDE_GAIN_NB];

struct FloatEulers stab_att_sp_euler;
struct FloatQuat   stab_att_sp_quat;

struct FloatQuat state_quat;

struct AttRefQuatFloat att_ref_quat_f;

struct FloatQuat stabilization_att_sum_err_quat;

struct FloatRates body_rate;
struct FloatRates last_body_rate;
struct FloatRates body_rate_d;

float stabilization_att_fb_cmd[COMMANDS_NB];
float stabilization_att_ff_cmd[COMMANDS_NB];
int32_t stabilization_cmd[COMMANDS_NB];

static int gain_idx = STABILIZATION_ATTITUDE_GAIN_IDX_DEFAULT;

static bool on_flight = false;

// static const float phi_pgain[] = STABILIZATION_ATTITUDE_PHI_PGAIN;
// static const float theta_pgain[] = STABILIZATION_ATTITUDE_THETA_PGAIN;
// static const float psi_pgain[] = STABILIZATION_ATTITUDE_PSI_PGAIN;

// static const float phi_dgain[] = STABILIZATION_ATTITUDE_PHI_DGAIN;
// static const float theta_dgain[] = STABILIZATION_ATTITUDE_THETA_DGAIN;
// static const float psi_dgain[] = STABILIZATION_ATTITUDE_PSI_DGAIN;

// static const float phi_igain[] = STABILIZATION_ATTITUDE_PHI_IGAIN;
// static const float theta_igain[] = STABILIZATION_ATTITUDE_THETA_IGAIN;
// static const float psi_igain[] = STABILIZATION_ATTITUDE_PSI_IGAIN;

// static const float phi_ddgain[] = STABILIZATION_ATTITUDE_PHI_DDGAIN;
// static const float theta_ddgain[] = STABILIZATION_ATTITUDE_THETA_DDGAIN;
// static const float psi_ddgain[] = STABILIZATION_ATTITUDE_PSI_DDGAIN;

// static const float phi_dgain_d[] = STABILIZATION_ATTITUDE_PHI_DGAIN_D;
// static const float theta_dgain_d[] = STABILIZATION_ATTITUDE_THETA_DGAIN_D;
// static const float psi_dgain_d[] = STABILIZATION_ATTITUDE_PSI_DGAIN_D;

quat_control_debug_t quat_control_debug;


#define IERROR_SCALE 1024

void stabilization_attitude_init(void)
{
  /* setpoints */
  FLOAT_EULERS_ZERO(stab_att_sp_euler);
  float_quat_identity(&stab_att_sp_quat);
  /* reference */
  attitude_ref_quat_float_init(&att_ref_quat_f);
  attitude_ref_quat_float_schedule(&att_ref_quat_f, STABILIZATION_ATTITUDE_GAIN_IDX_DEFAULT);

  VECT3_ASSIGN(stabilization_gains[0].p,
               configParam.pidAngle.roll.kp*180.f/M_PI,
               configParam.pidAngle.pitch.kp*180.f/M_PI,
               configParam.pidAngle.yaw.kp*180.f/M_PI);

  VECT3_ASSIGN(stabilization_gains[0].d,
               configParam.pidAngle.roll.kd*180.f/M_PI,
               configParam.pidAngle.pitch.kd*180.f/M_PI,
               configParam.pidAngle.yaw.kd*180.f/M_PI);

  VECT3_ASSIGN(stabilization_gains[0].i,
               configParam.pidAngle.roll.ki*180.f/M_PI,
               configParam.pidAngle.pitch.ki*180.f/M_PI,
               configParam.pidAngle.yaw.ki*180.f/M_PI);

  VECT3_ASSIGN(stabilization_gains[0].dd,
               configParam.pidRate.roll.kp*180.f/M_PI,
               configParam.pidRate.pitch.kp*180.f/M_PI,
               configParam.pidRate.yaw.kp*180.f/M_PI);
  
  VECT3_ASSIGN(stabilization_gains[0].rates_d,
               configParam.pidRate.roll.kd*180.f/M_PI,
               configParam.pidRate.pitch.kd*180.f/M_PI,
               configParam.pidRate.yaw.kd*180.f/M_PI);

  for (uint8_t i = 0; i < COMMANDS_NB; i++) {
    stabilization_cmd[i] = 0;
  }

  float_quat_identity(&stabilization_att_sum_err_quat);
  FLOAT_RATES_ZERO(last_body_rate);
  FLOAT_RATES_ZERO(body_rate_d);
}

void stabilization_attitude_gain_schedule(uint8_t idx)
{
  if (gain_idx >= STABILIZATION_ATTITUDE_GAIN_NB) {
    // This could be bad -- Just say no.
    return;
  }
  gain_idx = idx;
  attitude_ref_quat_float_schedule(&att_ref_quat_f, idx);
}

void stabilization_attitude_enter(float heading)
{
  /* reset psi setpoint to current psi angle */
  stab_att_sp_euler.psi = RadOfDeg(heading);
  getStateQuanternion(&state_quat.qi, &state_quat.qx, &state_quat.qy, &state_quat.qz);

  attitude_ref_quat_float_enter(&att_ref_quat_f, &state_quat);

  float_quat_identity(&stabilization_att_sum_err_quat);

  on_flight = true;
}

void stabilization_attitude_quit(){
  on_flight = false;
}

void stabilization_attitude_set_earth_cmd_i(struct Int32Vect2 *cmd, int32_t heading)
{
  struct FloatVect2 cmd_f;
  cmd_f.x = ANGLE_FLOAT_OF_BFP(cmd->x);
  cmd_f.y = ANGLE_FLOAT_OF_BFP(cmd->y);
  float heading_f;
  heading_f = ANGLE_FLOAT_OF_BFP(heading);

  quat_from_earth_cmd_f(&stab_att_sp_quat, &cmd_f, heading_f);
}

#ifndef GAIN_PRESCALER_FF
#define GAIN_PRESCALER_FF 1
#endif
static void attitude_run_ff(float ff_commands[], struct FloatAttitudeGains *gains, struct FloatRates *ref_accel)
{
  /* Compute feedforward based on reference acceleration */

  ff_commands[COMMAND_ROLL]          = GAIN_PRESCALER_FF * gains->dd.x * ref_accel->p;
  ff_commands[COMMAND_PITCH]         = GAIN_PRESCALER_FF * gains->dd.y * ref_accel->q;
  ff_commands[COMMAND_YAW]           = GAIN_PRESCALER_FF * gains->dd.z * ref_accel->r;
}

#ifndef GAIN_PRESCALER_P
#define GAIN_PRESCALER_P 1
#endif
#ifndef GAIN_PRESCALER_D
#define GAIN_PRESCALER_D 1
#endif
#ifndef GAIN_PRESCALER_I
#define GAIN_PRESCALER_I 1
#endif
static void attitude_run_fb(float fb_commands[], struct FloatAttitudeGains *gains, struct FloatQuat *att_err,
                            struct FloatRates *rate_err, struct FloatRates *rate_err_d, struct FloatQuat *sum_err)
{
  /*  PID feedback */
  fb_commands[COMMAND_ROLL] =
    GAIN_PRESCALER_P * gains->p.x  * att_err->qx +
    GAIN_PRESCALER_D * gains->d.x  * rate_err->p +
    GAIN_PRESCALER_D * gains->rates_d.x  * rate_err_d->p +
    GAIN_PRESCALER_I * gains->i.x  * sum_err->qx;

  fb_commands[COMMAND_PITCH] =
    GAIN_PRESCALER_P * gains->p.y  * att_err->qy +
    GAIN_PRESCALER_D * gains->d.y  * rate_err->q +
    GAIN_PRESCALER_D * gains->rates_d.y  * rate_err_d->q +
    GAIN_PRESCALER_I * gains->i.y  * sum_err->qy;

  fb_commands[COMMAND_YAW] =
    GAIN_PRESCALER_P * gains->p.z  * att_err->qz +
    GAIN_PRESCALER_D * gains->d.z  * rate_err->r +
    GAIN_PRESCALER_D * gains->rates_d.z  * rate_err_d->r +
    GAIN_PRESCALER_I * gains->i.z  * sum_err->qz;
}

void stabilization_attitude_run(control_t *control, sensorData_t *sensor, state_t *state, setpoint_t *setpoint)
{

  /*
   * Update reference
   */
  static const float dt = (1./PERIODIC_FREQUENCY);
  
  if(on_flight){
    stab_att_sp_euler.psi += RadOfDeg(setpoint->attitude.yaw) / PERIODIC_FREQUENCY;
    if (stab_att_sp_euler.psi > M_PI)
      stab_att_sp_euler.psi -= 2 * M_PI;
    if (stab_att_sp_euler.psi < M_PI)
      stab_att_sp_euler.psi += 2 * M_PI;
  }
  else{
    stab_att_sp_euler.psi = RadOfDeg(setpoint->attitude.yaw / 10) + RadOfDeg(state->attitude.yaw);
  }
  
  stab_att_sp_euler.phi = RadOfDeg(setpoint->attitude.roll);
  stab_att_sp_euler.theta = RadOfDeg(setpoint->attitude.pitch);
  float_quat_of_eulers(&stab_att_sp_quat, &stab_att_sp_euler);
  
  attitude_ref_quat_float_update(&att_ref_quat_f, &stab_att_sp_quat, dt);

  /*
   * Compute errors for feedback
   */

  /* attitude error                          */
  struct FloatQuat att_err;
  getStateQuanternion(&state_quat.qi, &state_quat.qx, &state_quat.qy, &state_quat.qz);
  float_quat_inv_comp(&att_err, &state_quat, &att_ref_quat_f.quat);
  /* wrap it in the shortest direction       */
  float_quat_wrap_shortest(&att_err);

  /*  rate error                */
  struct FloatRates rate_err;
  RATES_ASSIGN(body_rate, RadOfDeg(sensor->gyro.x), RadOfDeg(sensor->gyro.y), RadOfDeg(sensor->gyro.z));
  RATES_DIFF(rate_err, att_ref_quat_f.rate, body_rate);
  /* rate_d error               */
  RATES_DIFF(body_rate_d, body_rate, last_body_rate);
  RATES_COPY(last_body_rate, body_rate);

#define INTEGRATOR_BOUND 1.0f
  /* integrated error */
  if (on_flight) {
    stabilization_att_sum_err_quat.qx += att_err.qx / IERROR_SCALE;
    stabilization_att_sum_err_quat.qy += att_err.qy / IERROR_SCALE;
    stabilization_att_sum_err_quat.qz += att_err.qz / IERROR_SCALE;
    Bound(stabilization_att_sum_err_quat.qx, -INTEGRATOR_BOUND, INTEGRATOR_BOUND);
    Bound(stabilization_att_sum_err_quat.qy, -INTEGRATOR_BOUND, INTEGRATOR_BOUND);
    Bound(stabilization_att_sum_err_quat.qz, -INTEGRATOR_BOUND, INTEGRATOR_BOUND);
  } else {
    /* reset accumulator */
    float_quat_identity(&stabilization_att_sum_err_quat);
  }

  attitude_run_ff(stabilization_att_ff_cmd, &stabilization_gains[gain_idx], &att_ref_quat_f.accel);

  attitude_run_fb(stabilization_att_fb_cmd, &stabilization_gains[gain_idx], &att_err, &rate_err, &body_rate_d,
                  &stabilization_att_sum_err_quat);

  stabilization_cmd[COMMAND_ROLL] = stabilization_att_fb_cmd[COMMAND_ROLL] + stabilization_att_ff_cmd[COMMAND_ROLL];
  stabilization_cmd[COMMAND_PITCH] = stabilization_att_fb_cmd[COMMAND_PITCH] + stabilization_att_ff_cmd[COMMAND_PITCH];
  stabilization_cmd[COMMAND_YAW] = stabilization_att_fb_cmd[COMMAND_YAW] + stabilization_att_ff_cmd[COMMAND_YAW];

  /* bound the result */
  BoundAbs(stabilization_cmd[COMMAND_ROLL], MAX_PPRZ);
  BoundAbs(stabilization_cmd[COMMAND_PITCH], MAX_PPRZ);
  BoundAbs(stabilization_cmd[COMMAND_YAW], MAX_PPRZ);

  control -> roll = stabilization_cmd[COMMAND_ROLL];
  control -> pitch = stabilization_cmd[COMMAND_PITCH];
  control -> yaw = stabilization_cmd[COMMAND_YAW];

  // DEBUG
  quat_control_debug.roll = stab_att_sp_euler.phi;
  quat_control_debug.pitch = stab_att_sp_euler.theta;
  quat_control_debug.yaw = stab_att_sp_euler.psi;
  quat_control_debug.err_angle[0] = att_err.qx;
  quat_control_debug.err_angle[1] = att_err.qy;
  quat_control_debug.err_angle[2] = att_err.qz;
  quat_control_debug.err_rate[0] = rate_err.p;
  quat_control_debug.err_rate[1] = rate_err.q;
  quat_control_debug.err_rate[2] = rate_err.r;
  quat_control_debug.err_rate_dd[0] = body_rate_d.p;
  quat_control_debug.err_rate_dd[1] = body_rate_d.q;
  quat_control_debug.err_rate_dd[2] = body_rate_d.r;
  quat_control_debug.u[0] = control -> roll;
  quat_control_debug.u[1] = control -> pitch;
  quat_control_debug.u[2] = control -> yaw;
}

void paparazziUpdatePIDParams(){
  VECT3_ASSIGN(stabilization_gains[0].p,
               configParam.pidAngle.roll.kp*180.f/M_PI,
               configParam.pidAngle.pitch.kp*180.f/M_PI,
               configParam.pidAngle.yaw.kp*180.f/M_PI);

  VECT3_ASSIGN(stabilization_gains[0].d,
               configParam.pidAngle.roll.kd*180.f/M_PI,
               configParam.pidAngle.pitch.kd*180.f/M_PI,
               configParam.pidAngle.yaw.kd*180.f/M_PI);

  VECT3_ASSIGN(stabilization_gains[0].i,
               configParam.pidAngle.roll.ki*180.f/M_PI,
               configParam.pidAngle.pitch.ki*180.f/M_PI,
               configParam.pidAngle.yaw.ki*180.f/M_PI);

  VECT3_ASSIGN(stabilization_gains[0].dd,
               configParam.pidRate.roll.kp*180.f/M_PI,
               configParam.pidRate.pitch.kp*180.f/M_PI,
               configParam.pidRate.yaw.kp*180.f/M_PI);
  
  VECT3_ASSIGN(stabilization_gains[0].rates_d,
               configParam.pidRate.roll.kd*180.f/M_PI,
               configParam.pidRate.pitch.kd*180.f/M_PI,
               configParam.pidRate.yaw.kd*180.f/M_PI);
}