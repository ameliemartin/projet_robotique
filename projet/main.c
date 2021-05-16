/*
File : main.c
Author : Amelie Martin  & Carla Paillardon
Date : 16 may 2021

Initialize the different modules and starts the threads
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <motors.h>
#include <camera/po8030.h>
#include <chprintf.h>

#include <communications.h>
#include <arm_math.h>

#include <sensors/proximity.h>
#include <control_robot.h>

#include <fft.h>
#include <audio/microphone.h>
#include <audio/audio_thread.h>
#include <audio_processing.h>

#include <process_image.h>

// bus used for the proximity functions 
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

void SendUint8ToComputer(uint8_t* data, uint16_t size) 
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

// COMMUNICATION
    //initialize  the inter process communication bus
    messagebus_init(&bus, &bus_lock, &bus_condvar);
    //Starts the serial and the USB communications
    serial_start();
    usb_start();

// MOTORS
	//Initialize the control of the motors
	motors_init();

// SENSORS
	// Starts the IR proximity sensors 
	proximity_start();

// AUDIO
	// Powers ON the audio amplifier and DAC peripheral
    dac_start();
    // Starts the microphones acquisition
    mic_start(&processAudioData);

// CAMERA
    // Starts the camera
    dcmi_start(); 
    po8030_start();

//THREADS
	//Starts the threads for the control of the robot and the processing of the image
	control_robot_start(); 
	process_image_start();


    /* Infinite loop. */
    while (1) {
    	//waits 1 second
        chThdSleepMilliseconds(1000);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
