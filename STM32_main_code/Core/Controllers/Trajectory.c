/*
 * Trajectory.c
 *
 *  Created on: Mar 23, 2025
 *      Author: omart
 */

#include "Trajectory.h"

// Helper functions
static void fill_traj_time_vectors(Trajectory *traj, float32_t t) {
	traj->time_vec_pos_data[5] = 1.0f;							 // 1
	traj->time_vec_pos_data[4] = t; 							 // t
	traj->time_vec_pos_data[3] = traj->time_vec_pos_data[4] * t; // t^2
	traj->time_vec_pos_data[2] = traj->time_vec_pos_data[3] * t; // t^3
	traj->time_vec_pos_data[1] = traj->time_vec_pos_data[2] * t; // t^4
	traj->time_vec_pos_data[0] = traj->time_vec_pos_data[1] * t; // t^5

    // Only calculate velocity vector if needed
    if (traj->compute_velocity) {
        traj->time_vec_vel_data[0] = 5.0f * traj->time_vec_pos_data[1]; // 5t^4
        traj->time_vec_vel_data[1] = 4.0f * traj->time_vec_pos_data[2]; // 4t^3
        traj->time_vec_vel_data[2] = 3.0f * traj->time_vec_pos_data[3]; // 3t^2
        traj->time_vec_vel_data[3] = 2.0f * traj->time_vec_pos_data[4]; // 2t
        traj->time_vec_vel_data[4] = 1.0f;                              // 1
        // traj->time_vec_vel_data[5] = 0.0f;                              // 0
    }

    // Only calculate acceleration vector if needed
    if (traj->compute_acceleration) {
        traj->time_vec_acc_data[0] = 20.0f * traj->time_vec_pos_data[2]; // 20t^3
        traj->time_vec_acc_data[1] = 12.0f * traj->time_vec_pos_data[3]; // 12t^2
        traj->time_vec_acc_data[2] = 6.0f * traj->time_vec_pos_data[4];  // 6t
        traj->time_vec_acc_data[3] = 2.0f;                               // 2
        // traj->time_vec_acc_data[4] = 0.0f;                               // 0
        // traj->time_vec_acc_data[5] = 0.0f;                               // 0
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
static void transition_to_phase(Trajectory *traj, uint8_t phase) {
    if (traj == NULL || phase >= MAX_TRAJ_PHASES) return;

    traj->currentPhase = phase;
    traj->phaseTime = 0.0f;

    // Update coefficient matrix to point to the current phase's coefficients
    arm_mat_init_f32(&(traj->coeff_mat), NUM_JOINTS_TRAJ, TRAJ_POLY_TERMS, traj->coeff_data[phase]);
}

// User functions
void Trajectory_Init(Trajectory *traj) {
	if (traj == NULL) return;

	// Initialize all phase coefficient matrices to zero
	for (int phase = 0; phase < MAX_TRAJ_PHASES; phase++) {
		for (int i = 0; i < NUM_JOINTS_TRAJ*TRAJ_POLY_TERMS; i++) {
			traj->coeff_data[phase][i] = 0.0f;
		}
	} // Initialize the coefficient matrix with phase 0 data
	arm_mat_init_f32(&(traj->coeff_mat), NUM_JOINTS_TRAJ, TRAJ_POLY_TERMS, traj->coeff_data[0]);

	for (int i = 0; i < TRAJ_POLY_TERMS; i++) {
		traj->time_vec_pos_data[i] = 0.0f;
		traj->time_vec_vel_data[i] = 0.0f;
		traj->time_vec_acc_data[i] = 0.0f;
	}
	arm_mat_init_f32(&(traj->time_vec_pos_mat), TRAJ_POLY_TERMS, 1, traj->time_vec_pos_data);
	arm_mat_init_f32(&(traj->time_vec_vel_mat), TRAJ_POLY_TERMS, 1, traj->time_vec_vel_data);
	arm_mat_init_f32(&(traj->time_vec_acc_mat), TRAJ_POLY_TERMS, 1, traj->time_vec_acc_data);

	for (int i = 0; i < NUM_JOINTS_TRAJ; i++) {
		traj->position_data[i] = 0.0f;
		traj->velocity_data[i] = 0.0f;
		traj->acceleration_data[i] = 0.0f;
	}
	arm_mat_init_f32(&(traj->position_mat), NUM_JOINTS_TRAJ, 1, traj->position_data);
	arm_mat_init_f32(&(traj->velocity_mat), NUM_JOINTS_TRAJ, 1, traj->velocity_data);
	arm_mat_init_f32(&(traj->acceleration_mat), NUM_JOINTS_TRAJ, 1, traj->acceleration_data);

	// Initialize timing parameters
	traj->startTime = 0.0f;
	traj->currentTime = 0.0f;
	traj->phaseTime = 0.0f;
	traj->totalDuration = 0.0f;

	// Initialize all phase durations
	for (int i = 0; i < MAX_TRAJ_PHASES; i++) {
		traj->duration[i] = 0.0f;
	}

	// Initialize phase tracking
	traj->currentPhase = 0;
	traj->numPhases = MAX_TRAJ_PHASES;  // Default to single phase

	// Initialize state
	traj->state = TRAJ_IDLE;

	// Initialize derivative computation flags
	traj->compute_velocity = 0;
	traj->compute_acceleration = 0;
}

void Trajectory_Start(Trajectory *traj) {
    if (traj == NULL) return;

    traj->state = TRAJ_RUNNING;
    traj->startTime = 0.0f;
    traj->currentTime = 0.0f;
    traj->phaseTime = 0.0f;

    // Start with phase 0
    transition_to_phase(traj, 0);
}

void Trajectory_Compute(Trajectory *traj, float32_t dt) {
    if (traj == NULL) return;

    switch (traj->state) {
        case TRAJ_IDLE:
            break;

        case TRAJ_RUNNING:
            // Update both overall time and phase time
            traj->currentTime += dt;
            traj->phaseTime += dt;

            // Check if current phase is complete
            if (traj->phaseTime >= traj->duration[traj->currentPhase])
            {
                // Move to next phase
                if (traj->currentPhase < traj->numPhases - 1)
                {
                    transition_to_phase(traj, traj->currentPhase + 1);
                }
                else
                {
                    // All phases complete
                    traj->state = TRAJ_COMPLETE;
                    traj->currentTime = traj->totalDuration; // Cap at total duration
                    traj->phaseTime = traj->duration[traj->currentPhase]; // Cap at phase duration
                    break;
                }
            }

			fill_traj_time_vectors(traj, traj->phaseTime);
			compute_polynomial(traj);
            break;

        case TRAJ_COMPLETE:
            break;

        default:
            break;
    }
}
void Trajectory_SetCoefficients(Trajectory *traj, float32_t *new_coeffs, uint8_t phase) {
    if (traj == NULL || phase >= MAX_TRAJ_PHASES) return;

    // Copy coefficients for the specified phase
    arm_copy_f32(new_coeffs, traj->coeff_data[phase], NUM_JOINTS_TRAJ*TRAJ_POLY_TERMS);

    // If this is the current phase, update the coefficient matrix
    if (phase == traj->currentPhase) {
        arm_mat_init_f32(&(traj->coeff_mat), NUM_JOINTS_TRAJ, TRAJ_POLY_TERMS, traj->coeff_data[phase]);
    }
}

void Trajectory_SetDuration(Trajectory *traj, float32_t *durations, uint8_t num_phases) {
    if (traj == NULL || num_phases > MAX_TRAJ_PHASES) return;

    // Store number of phases
    traj->numPhases = num_phases;

    // Copy durations for each phase
    traj->totalDuration = 0.0f;
    for (uint8_t i = 0; i < num_phases; i++) {
        traj->duration[i] = durations[i];
        traj->totalDuration += durations[i];
    }
}

HAL_StatusTypeDef Trajectory_ParseCoeffs(const char* input_str, Trajectory* traj) {
    if (input_str == NULL || traj == NULL) return HAL_ERROR;

    const char* start = input_str;
    if (start[0] != 'T' || start[1] != 'C') return HAL_ERROR;
    start += 2;

    float32_t tempCoeffs[TOTAL_COEFFS] = {0};
    float32_t tempDurations[MAX_TRAJ_PHASES] = {0};
    int value_count = 0;
    char* end;

    // Parse coefficients
    while (*start != '\0' && *start != 'T' && value_count < TOTAL_COEFFS) {
        float32_t val = strtof(start, &end);
        if (end == start) {
            start++;  // Skip invalid char
            continue;
        }
        tempCoeffs[value_count++] = val;
        start = end;
        while (*start == ',' || *start == ' ') start++;
    }

    if (*start != 'T') return HAL_ERROR;
    start++; // Skip 'T'

    // Parse durations
    int dur_count = 0;
    while (*start != '\0' && dur_count < MAX_TRAJ_PHASES) {
        float32_t val = strtof(start, &end);
        if (end == start) {
            start++;
            continue;
        }
        tempDurations[dur_count++] = val;
        start = end;
        while (*start == ',' || *start == ' ') start++;
    }

    // Validate parsed counts
    if (value_count != TOTAL_COEFFS || dur_count != MAX_TRAJ_PHASES) return HAL_ERROR;

    for (int phase = 0; phase < MAX_TRAJ_PHASES; phase++) {
        Trajectory_SetCoefficients(traj, &tempCoeffs[phase * NUM_JOINTS_TRAJ * TRAJ_POLY_TERMS], phase);
    }

    Trajectory_SetDuration(traj, tempDurations, MAX_TRAJ_PHASES);
    return HAL_OK;
}





























