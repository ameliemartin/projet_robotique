/*
File : audio_processing.h
Author : Amelie Martin  & Carla Paillardon
Date : 16 may 2021

Capture and analyse the frequency and returns the instructions to move the robot to "control_robot.c" 

*/

#ifndef AUDIO_PROCESSING_H
#define AUDIO_PROCESSING_H


#define FFT_SIZE 	1024

/*
* Those are the instruction to move the robot depending on the frequency sent to contro_robot.c
*/
#define DONT_TURN               0
#define TURN_LEFT               1
#define TURN_RIGHT              2
#define MOVE_FORWARD            3

typedef enum {
	//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
	LEFT_CMPLX_INPUT = 0,
	RIGHT_CMPLX_INPUT,
	FRONT_CMPLX_INPUT,
	BACK_CMPLX_INPUT,
	//Arrays containing the computed magnitude of the complex numbers
	LEFT_OUTPUT,
	RIGHT_OUTPUT,
	FRONT_OUTPUT,
	BACK_OUTPUT
} BUFFER_NAME_t;

// Function used to send the frequency to control_robot 
int8_t get_freq(void);

int8_t sound_remote(float* data);

void processAudioData(int16_t *data, uint16_t num_samples);

/*
*	put the invoking thread into sleep until it can process the audio datas
*/
void wait_send_to_computer(void);

/*
*	Returns the pointer to the BUFFER_NAME_t buffer asked
*/
//float* get_audio_buffer_ptr(BUFFER_NAME_t name);

#endif /* AUDIO_PROCESSING_H */
