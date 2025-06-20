/*
 * Trajectory.h
 *
 *  Created on: Mar 23, 2025
 *      Author: omart
 */

#ifndef CONTROLLERS_TRAJECTORY_H_
#define CONTROLLERS_TRAJECTORY_H_

#include "main.h"
#include "arm_math.h"

#define MAX_TRAJ_PHASES 6
#define TOTAL_COEFFS (NUM_JOINTS_TRAJ * TRAJ_POLY_TERMS * MAX_TRAJ_PHASES)

typedef enum {
    TRAJ_IDLE,        // Not running, waiting for start command
    TRAJ_RUNNING,     // Actively computing trajectory points
    TRAJ_COMPLETE     // Trajectory completed, holding final position
} TrajectoryState;

typedef struct {
    // Multiple sets of coefficients (one per phase)
    float32_t coeff_data[MAX_TRAJ_PHASES][NUM_JOINTS_TRAJ*TRAJ_POLY_TERMS];
    arm_matrix_instance_f32 coeff_mat;  // Current phase coefficients matrix

    float32_t time_vec_pos_data[TRAJ_POLY_TERMS]; // [1, t, t^2, t^3, t^4, t^5]
    float32_t time_vec_vel_data[TRAJ_POLY_TERMS]; // [0, 1, 2t, 3t^2, 4t^3, 5t^4]
    float32_t time_vec_acc_data[TRAJ_POLY_TERMS]; // [0, 0, 2, 6t, 12t^2, 20t^3]
    arm_matrix_instance_f32 time_vec_pos_mat;
    arm_matrix_instance_f32 time_vec_vel_mat;
    arm_matrix_instance_f32 time_vec_acc_mat;

    // Output data
    float32_t position_data[NUM_JOINTS_TRAJ];
    float32_t velocity_data[NUM_JOINTS_TRAJ];
    float32_t acceleration_data[NUM_JOINTS_TRAJ];
    arm_matrix_instance_f32 position_mat;
    arm_matrix_instance_f32 velocity_mat;
    arm_matrix_instance_f32 acceleration_mat;

    // Timing parameters
    float32_t startTime;    // When trajectory began
    float32_t currentTime;  // Current time within entire trajectory
    float32_t phaseTime;    // Current time within current phase
    float32_t duration[MAX_TRAJ_PHASES];  // Duration of each phase
    float32_t totalDuration;  // Total trajectory duration (sum of all phases)

    // Phase tracking
    uint8_t currentPhase;   // Current active phase (0 to MAX_TRAJ_PHASES-1)
    uint8_t numPhases;      // Number of active phases (1 to MAX_TRAJ_PHASES)

    // State tracking
    TrajectoryState state;
    volatile uint8_t readyFlag;

    // Output selection flags
    uint8_t compute_velocity;     // Whether to compute velocity
    uint8_t compute_acceleration; // Whether to compute acceleration
} Trajectory;

void Trajectory_Init(Trajectory *);

void Trajectory_Start(Trajectory*);
void Trajectory_Stop(Trajectory*);
void Trajectory_Compute(Trajectory*, float32_t);

void Trajectory_SetCoefficients(Trajectory *, float32_t *, uint8_t);
void Trajectory_SetDuration(Trajectory *, float32_t *, uint8_t);
void Trajectory_EnableDerivatives(Trajectory *, uint8_t, uint8_t);

uint8_t Trajectory_IsActive(Trajectory*);
HAL_StatusTypeDef Trajectory_ParseCoeffs(const char* input_str, Trajectory* traj);

#endif /* CONTROLLERS_TRAJECTORY_H_ */






