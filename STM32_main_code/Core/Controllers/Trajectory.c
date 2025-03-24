/*
 * Trajectory.c
 *
 *  Created on: Mar 23, 2025
 *      Author: omart
 */

#include "Trajectory.h"

// Helper functions
static void fill_traj_time_vectors(Trajectory *traj, float32_t t) {
	traj->time_vec_pos_data[0] = 1.0f;							 // 1
	traj->time_vec_pos_data[1] = t; 							 // t
	traj->time_vec_pos_data[2] = traj->time_vec_pos_data[1] * t; // t^2
	traj->time_vec_pos_data[3] = traj->time_vec_pos_data[2] * t; // t^3
	traj->time_vec_pos_data[4] = traj->time_vec_pos_data[3] * t; // t^4
	traj->time_vec_pos_data[5] = traj->time_vec_pos_data[4] * t; // t^5

    // Only calculate velocity vector if needed
    if (traj->compute_velocity) {
        traj->time_vec_vel_data[0] = 0.0f;
        traj->time_vec_vel_data[1] = 1.0f;
        traj->time_vec_vel_data[2] = 2.0f * t;
        traj->time_vec_vel_data[3] = 3.0f * traj->time_vec_pos_data[2];
        traj->time_vec_vel_data[4] = 4.0f * traj->time_vec_pos_data[3];
        traj->time_vec_vel_data[5] = 5.0f * traj->time_vec_pos_data[4];
    }

    // Only calculate acceleration vector if needed
    if (traj->compute_acceleration) {
        traj->time_vec_acc_data[0] = 0.0f;
        traj->time_vec_acc_data[1] = 0.0f;
        traj->time_vec_acc_data[2] = 2.0f;
        traj->time_vec_acc_data[3] = 6.0f * t;
        traj->time_vec_acc_data[4] = 12.0f * traj->time_vec_pos_data[2];
        traj->time_vec_acc_data[5] = 20.0f * traj->time_vec_pos_data[3];
    }
}
static void compute_polynomial(Trajectory *traj) {
	arm_mat_mult_f32(&(traj->coeff_mat), &(traj->time_vec_pos_mat), &(traj->position_mat));
	if (traj->compute_velocity) {
		arm_mat_mult_f32(&(traj->coeff_mat), &(traj->time_vec_vel_mat), &(traj->velocity_mat));
	}
	if (traj->compute_acceleration) {
		arm_mat_mult_f32(&(traj->coeff_mat), &(traj->time_vec_acc_mat), &(traj->acceleration_mat));
	}
}

// User functions
void Trajectory_Init(Trajectory *traj) {
	if (traj == NULL) return;

	for (int i = 0; i < NUM_JOINTS_TRAJ*TRAJ_POLY_TERMS; i++) {
		traj->coeff_data[i] = 0.0f;
	}

	for (int i = 0; i < TRAJ_POLY_TERMS; i++) {
		traj->time_vec_pos_data[i] = 0.0f;
		traj->time_vec_vel_data[i] = 0.0f;
		traj->time_vec_acc_data[i] = 0.0f;
	}

	for (int i = 0; i < TRAJ_POLY_TERMS; i++) {
		traj->time_vec_pos_data[i] = 0.0f;
		traj->time_vec_vel_data[i] = 0.0f;
		traj->time_vec_acc_data[i] = 0.0f;
	}

	traj->startTime = 0.0f;
	traj->currentTime = 0.0f;
	traj->duration = 0.0f;

	traj->state = TRAJ_IDLE;

	traj->compute_velocity = 0;
	traj->compute_acceleration = 0;
}

void Trajectory_SetCoefficients(Trajectory *traj, float32_t *new_coeffs) {
	if (traj == NULL) return;
	arm_copy_f32(new_coeffs, traj->coeff_data, NUM_JOINTS_TRAJ*TRAJ_POLY_TERMS);
}
void Trajectory_SetDuration(Trajectory *traj, float32_t dur) {
	if (traj == NULL) return;

	traj->duration = dur;
}
void Trajectory_EnableDerivatives(Trajectory *traj, uint8_t vec, uint8_t acc) {
	if (traj == NULL) return;

	traj->compute_velocity = vec;
	traj->compute_acceleration = acc;
}

void Trajectory_Start(Trajectory *traj) {
	if (traj == NULL) return;

	traj->state = TRAJ_RUNNING;
}
void Trajectory_Stop(Trajectory *traj) {
	if (traj == NULL) return;

	traj->state = TRAJ_IDLE;
}

void Trajectory_Compute(Trajectory *traj, float32_t dt) {
	if (traj == NULL) return;

	switch (traj->state) {
		case TRAJ_IDLE:
			break;

		case TRAJ_RUNNING:
			traj->currentTime += dt;

			if (traj->currentTime >= traj->duration)
			{
				traj->state = TRAJ_COMPLETE;
				traj->currentTime = traj->duration;
			}

			fill_traj_time_vectors(traj, traj->currentTime);
			compute_polynomial(traj);
			break;

		case TRAJ_COMPLETE:
			break;

		default:
			break;
	}
}

uint8_t Trajectory_IsActive(Trajectory *traj) {
	if (traj == NULL) return 0;

	return traj->state == TRAJ_RUNNING;
}

HAL_StatusTypeDef Trajectory_ParseCoeffs(const char* input_string, float32_t* coeff_array, int expected_count) {
    if (input_string == NULL || coeff_array == NULL || expected_count <= 0) {
        return HAL_ERROR;
    }

    char* input_copy = strdup(input_string); // Create a copy as strtok modifies the string
    if (input_copy == NULL) {
        return HAL_ERROR; // Memory allocation failed
    }

    char* token;
    char* rest = input_copy;
    int coeff_index = 0;

    // Parse each token separated by comma
    while ((token = strtok_r(rest, ",", &rest)) && coeff_index < expected_count) {
        // Convert string to float
        coeff_array[coeff_index] = (float32_t)atof(token);
        coeff_index++;
    }

    free(input_copy); // Free the allocated memory

    // Check if we got the correct number of coefficients
    if (coeff_index == expected_count) {
        return HAL_OK;
    } else {
        return HAL_ERROR; // Incorrect number of coefficients
    }
}







