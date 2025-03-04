/*
 * PID_Control.c
 *
 *  Created on: Mar 4, 2025
 *      Author: omart
 */

#include "PID_Control.h"

void MultivariablePID_Init(MultivariablePID *pid, float32_t *Kp_f32, float32_t *Ki_f32, float32_t *Kd_f32) {
	for (int i = 0; i < NUM_JOINTS*NUM_JOINTS; i++) {
		pid->Kp_data[i] = Kp_f32[i];
		pid->Ki_data[i] = Ki_f32[i];
		pid->Kd_data[i] = Kd_f32[i];
	}

	arm_mat_init_f32(&(pid->Kp_mat), NUM_JOINTS, NUM_JOINTS, pid->Kp_data);
	arm_mat_init_f32(&(pid->Ki_mat), NUM_JOINTS, NUM_JOINTS, pid->Ki_data);
	arm_mat_init_f32(&(pid->Kd_mat), NUM_JOINTS, NUM_JOINTS, pid->Kd_data);

	// Initialize data arrays to zero
	for (int i = 0; i < NUM_JOINTS; i++) {
		pid->setpoint_data[i] = 2048.0f;
		pid->meas_data[i] = 0.0f;
		pid->output_data[i] = 0.0f;

		pid->error_data[i] = 0.0f;
		pid->error_sum_data[i] = 0.0f;
		pid->error_prev_data[i] = 0.0f;

		pid->temp1_N_1_data[i] = 0.0f;
		pid->temp2_N_1_data[i] = 0.0f;
	}

	arm_mat_init_f32(&(pid->setpoint_mat), NUM_JOINTS, 1, pid->setpoint_data);
	arm_mat_init_f32(&(pid->meas_mat), NUM_JOINTS, 1, pid->meas_data);
	arm_mat_init_f32(&(pid->output_mat), NUM_JOINTS, 1, pid->output_data);

	arm_mat_init_f32(&(pid->error_mat), NUM_JOINTS, 1, pid->error_data);
	arm_mat_init_f32(&(pid->error_sum_mat), NUM_JOINTS, 1, pid->error_sum_data);
	arm_mat_init_f32(&(pid->error_prev_mat), NUM_JOINTS, 1, pid->error_prev_data);

	arm_mat_init_f32(&(pid->temp1_N_1_mat), NUM_JOINTS, 1, pid->temp1_N_1_data);
	arm_mat_init_f32(&(pid->temp2_N_1_mat), NUM_JOINTS, 1, pid->temp2_N_1_data);
}
void MultivariablePID_SetSetpoint(MultivariablePID *pid, float32_t *setpoint) {
	if (pid == NULL || setpoint == NULL) return;

	// Copy the new setpoint values to our internal storage
	for (int i = 0; i < NUM_JOINTS; i++) {
		pid->setpoint_data[i] = setpoint[i];
	}
}
void MultivariablePID_Compute(MultivariablePID *pid, float32_t *meas) {
  if (pid == NULL || meas == NULL) return;

  for (int i = 0; i < NUM_JOINTS; i++) {
  		pid->meas_data[i] = meas[i];
  }

  // error = setpoint - measurement
  arm_mat_sub_f32(&(pid->setpoint_mat), &(pid->meas_mat), &(pid->error_mat));

  arm_mat_mult_f32(&(pid->Kp_mat), &(pid->error_mat), &(pid->output_mat));
}




