/*
File : main.h
Author : Amelie Martin  & Carla Paillardon
Date : 16 may 2021

Initialize the different modules and starts the threads
*/
#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"


//constants for the differents parts of the project
#define IMAGE_BUFFER_SIZE		640
#define WIDTH_SLOPE				20
#define MIN_LINE_WIDTH			40

#define PXTOCM					1570.0f //experimental value
#define GOAL_DISTANCE 			7.0f //remplacer 10 par 7 pour voir ?
#define MAX_DISTANCE 			25.0f

// A SUPPRIMER (sont tous pour le fichier PI regulator)
#define KP						800.0f
#define KI 						3.5f	//must not be zero
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI) 
#define ERROR_THRESHOLD			0.1f	//[cm] because of the noise of the camera

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

void SendUint8ToComputer(uint8_t* data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif
