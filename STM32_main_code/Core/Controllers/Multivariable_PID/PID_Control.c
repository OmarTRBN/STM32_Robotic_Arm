/*
 * PID_Control.c
 *
 *  Created on: Mar 4, 2025
 *      Author: omart
 */

#include "PID_Control.h"

void MultivariablePID_Init(MultivariablePID *pid) {
	for (int i = 0; i < NUM_JOINTS*NUM_JOINTS; i++) {
		pid->Kp_data[i] = 0.0f;
		pid->Ki_data[i] = 0.0f;
		pid->Kd_data[i] = 0.0f;
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

//	pid->invert_data[0] = 1.0f;
//	pid->invert_data[1] = 1.0f;
//	pid->invert_data[2] = 1.0f;
//	pid->invert_data[3] = -1.0f;
//	arm_mat_init_f32(&(pid->invert_mat), NUM_JOINTS, 1, pid->invert_data);
}

void MultivariablePID_SetSetpoint(MultivariablePID *pid, float32_t *setpoint) {
	if (pid == NULL || setpoint == NULL) return;

	for (int i = 0; i < NUM_JOINTS; i++) {
		// Check if setpoint is within valid range
		if (setpoint[i] < SETPOINT_MIN) {
			pid->setpoint_data[i] = SETPOINT_MIN;
		} else if (setpoint[i] > SETPOINT_MAX) {
			pid->setpoint_data[i] = SETPOINT_MAX;
		} else {
			pid->setpoint_data[i] = setpoint[i];
		}
	}
}

void MultivariablePID_Compute(MultivariablePID *pid, float32_t *meas) {
  if (pid == NULL || meas == NULL) return;

  for (int i = 0; i < NUM_JOINTS; i++) {
    pid->meas_data[i] = meas[i];
  }

  // error = setpoint - measurement
  arm_mat_sub_f32(&(pid->setpoint_mat), &(pid->meas_mat), &(pid->error_mat));

  // Proportional Term: P = Kp * error
  arm_mat_mult_f32(&(pid->Kp_mat), &(pid->error_mat), &(pid->output_mat));

  // Integral Term: I = Ki * integral(error)
  // Update error sum (accumulate integral of error)
  // Scale by dt (sample time)
  arm_mat_add_f32(&(pid->error_sum_mat), &(pid->error_mat), &(pid->error_sum_mat));
  arm_mat_mult_f32(&(pid->Ki_mat), &(pid->error_sum_mat), &(pid->temp1_N_1_mat));
  arm_scale_f32(pid->temp1_N_1_data, pid->dt, pid->temp1_N_1_data, NUM_JOINTS);

  // Derivative Term: D = Kd * (error - previous_error)
  // Update the derivative term (change in error)
  // Scale by dt (sample time)
  arm_mat_sub_f32(&(pid->error_mat), &(pid->error_prev_mat), &(pid->temp2_N_1_mat));
  arm_mat_mult_f32(&(pid->Kd_mat), &(pid->temp2_N_1_mat), &(pid->temp2_N_1_mat));
  arm_scale_f32(pid->temp2_N_1_data, pid->dt, pid->temp2_N_1_data, NUM_JOINTS);

  // Sum the P, I, D terms
  arm_mat_add_f32(&(pid->output_mat), &(pid->temp1_N_1_mat), &(pid->output_mat));  // P + I
  arm_mat_add_f32(&(pid->output_mat), &(pid->temp2_N_1_mat), &(pid->output_mat));  // P + I + D

  // Optionally scale the final output if necessary (e.g., by a factor to adjust magnitude)
  float32_t output_scale_factor = 0.01f;
  arm_scale_f32(pid->output_data, output_scale_factor, pid->output_data, NUM_JOINTS);

  // Apply the inversion to the output (multiply each output by its corresponding inversion value)
  //  arm_mult_f32(pid->output_data, pid->invert_data, pid->output_data, NUM_JOINTS);

  // Save the current error as the previous error for the next iteration
  arm_copy_f32(pid->error_data, pid->error_prev_data, NUM_JOINTS);
}

//void MultivariablePID_SetParameter(MultivariablePID *pid, float32_t *new_matrix, uint16_t chosen_param) {
//    if (pid == NULL || new_matrix == NULL) return;
//
//    float32_t *target_data = NULL;
//    // Select the appropriate matrix based on the chosen parameter
//    switch (chosen_param) {
//        case CMD_SET_KP:
//            target_data = pid->Kp_data;
//            break;
//
//        case CMD_SET_KI:
//            target_data = pid->Ki_data;
//            break;
//
//        case CMD_SET_KD:
//            target_data = pid->Kd_data;
//            break;
//
//        default:
//            // Invalid parameter choice
//            return;
//    }
//
//    // Copy new values to the selected data array
//    arm_copy_f32(new_matrix, target_data, NUM_JOINTS*NUM_JOINTS);
//    // No need to re-initialize the matrix as the data pointer remains the same
//}
//
//uint8_t ParsePIDParametersFromUART(MultivariablePID *pid, char *uart_str, uint16_t len) {
//    if (pid == NULL || uart_str == NULL || len == 0) return 0;
//
//    // Make sure the string is null-terminated
//    if (uart_str[len-1] != '\0') {
//        if (len >= MAX_UART_BUFFER) {
//            // String too long, can't safely null-terminate
//            return 0;
//        }
//        uart_str[len] = '\0';
//    }
//
//    float32_t parsed_values[NUM_JOINTS*NUM_JOINTS];
//    for (int i = 0; i < NUM_JOINTS*NUM_JOINTS; i++) {
//        parsed_values[i] = 0.0f;
//    }
//
//    // Determine which parameter is being updated
//    uint16_t chosen_param;
//    char *data_start = NULL;
//
//    if (strncmp(uart_str, "KP", 2) == 0) {
//        chosen_param = CMD_SET_KP;
//    }
//    else if (strncmp(uart_str, "KI", 2) == 0) {
//        chosen_param = CMD_SET_KI;
//    }
//    else if (strncmp(uart_str, "KD", 2) == 0) {
//        chosen_param = CMD_SET_KD;
//    }
//    else {
//        // Unrecognized parameter
//        return 0;
//    }
//    data_start = uart_str + 2;
//
//    // Parse the comma-separated values
//    char *token;
//    char *rest = data_start;
//    int index = 0;
//
//    while ((token = strtok_r(rest, ",", &rest)) != NULL && index < NUM_JOINTS*NUM_JOINTS) {
//        // Convert the token to float
//        parsed_values[index] = (float32_t)atof(token);
//        index++;
//    }
//
//    // Check if we received the expected number of values
//    if (index != NUM_JOINTS*NUM_JOINTS) {
//        // Invalid number of parameters
//        return 0;
//    }
//
//    // Update the PID parameters
//    MultivariablePID_SetParameter(pid, parsed_values, chosen_param);
//
//    return 1;
//}







