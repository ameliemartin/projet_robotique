/*
File : process_image.h
Author : Amelie Martin & Carla Paillardon
Date : 16 may 2021

Capture and analyse the image and returns a boolean to control_robot.c to determine if a crosswalk is detected by the camera
*/
#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

/*
* Function used to return a boolean to control_robot.c to determine if a crosswalk is detected by the camera
*/
bool crosswalk_detected(void);


//Start the process image thread
void process_image_start(void);

#endif /* PROCESS_IMAGE_H */
