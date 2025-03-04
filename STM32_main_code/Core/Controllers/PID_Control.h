/*
 * PID_Control.h
 *
 *  Created on: Mar 4, 2025
 *      Author: omart
 */

#ifndef CONTROLLERS_PID_CONTROL_H_
#define CONTROLLERS_PID_CONTROL_H_

#include "main.h"
#include "arm_math.h"

typedef struct {
	float32_t Kp_data[NUM_JOINTS*NUM_JOINTS];
	float32_t Ki_data[NUM_JOINTS*NUM_JOINTS];
	float32_t Kd_data[NUM_JOINTS*NUM_JOINTS];
    arm_matrix_instance_f32 Kp_mat;    // Proportional gain matrix
    arm_matrix_instance_f32 Ki_mat;    // Integral gain matrix
    arm_matrix_instance_f32 Kd_mat;    // Derivative gain matrix

    float32_t setpoint_data[NUM_JOINTS];    // Actual storage for setpoint values
	float32_t meas_data[NUM_JOINTS]; // Actual storage for measurement values
	float32_t output_data[NUM_JOINTS];     // Actual storage for output values
	arm_matrix_instance_f32 setpoint_mat;   // Desired values
	arm_matrix_instance_f32 meas_mat; // Current measured values
	arm_matrix_instance_f32 output_mat;     // Controller output

    float32_t error_data[NUM_JOINTS];
    float32_t error_sum_data[NUM_JOINTS];
    float32_t error_prev_data[NUM_JOINTS];
    arm_matrix_instance_f32 error_mat;      // Current error
    arm_matrix_instance_f32 error_sum_mat;  // Integral of error
    arm_matrix_instance_f32 error_prev_mat; // Previous error for derivative

    float32_t temp1_N_1_data[NUM_JOINTS];
    float32_t temp2_N_1_data[NUM_JOINTS];
    arm_matrix_instance_f32 temp1_N_1_mat;   // Temporary Nx1 matrix for calculations
	arm_matrix_instance_f32 temp2_N_1_mat;  // Second temporary Nx1 matrix

    float32_t dt;                  // Sample time in seconds
} MultivariablePID;

void MultivariablePID_Init(MultivariablePID *pid, float32_t *Kp_f32, float32_t *Ki_f32, float32_t *Kd_f32);
void MultivariablePID_SetSetpoint(MultivariablePID *pid, float32_t *setpoint);
void MultivariablePID_Compute(MultivariablePID *pid, float32_t *meas);
void MultivariablePID_Reset(MultivariablePID *pid);

#endif /* CONTROLLERS_PID_CONTROL_H_ */
