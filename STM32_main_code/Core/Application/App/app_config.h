/*
 * app_config.h
 *
 *  Created on: Jun 19, 2025
 *      Author: omarmac
 */

#ifndef APP_APP_CONFIG_H_
#define APP_APP_CONFIG_H_

#define APP_ENCODER_FREQ 	250
#define APP_CONTROLLER_FREQ 100
#define APP_TRAJ_FREQ    	100

#define NUM_JOINTS 			4

#define CMD_TEST_LED	        ( ('T'<<8) | 'L') // "TL" Test LED
#define CMD_MOTOR_STATE 		( ('M'<<8) | 'S') // "MS" Motor State
#define CMD_SET_PID				( ('C'<<8) | 'P') // "CP" Controller Parameters
#define CMD_MOTOR_REF			( ('M'<<8) | 'R') // "MR" Motor Reference
#define CMD_START_TRAJ			( ('B'<<8) | 'T') // "ST" Start Trajectory
#define CMD_TRAJ_COEFF 			( ('T'<<8) | 'C') // Trajectory Coefficients

#endif /* APP_APP_CONFIG_H_ */
