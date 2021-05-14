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
//#include <camera/po8030.h>
#include <chprintf.h>

#include <fft.h>
#include <communications.h>
#include <arm_math.h>


#include <control_robot.h>
#include <audio_processing.h>
#include <sensors/proximity.h>
#include <spi_comm.h>
#include <audio/microphone.h>
#include <process_image.h>


// pour utilisation des fichiers proxi
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

// ajout du TP 5 de ça 
/*static void timer12_start(void){
    //General Purpose Timer configuration   
    //timer 12 is a 16 bit timer so we can measure time
    //to about 65ms with a 1Mhz counter
    static const GPTConfig gpt12cfg = {
        1000000,        //1MHz timer clock in order to measure uS.
        NULL,           //Timer callback.
        0,
        0
    };

    gptStart(&GPTD12, &gpt12cfg);
    //let the timer count to max value
    gptStartContinuous(&GPTD12, 0xFFFF);
}*/



int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    //initialize  the inter process communication bus
     messagebus_init(&bus, &bus_lock, &bus_condvar);

    //starts the serial communication
    serial_start();
    //start the USB communication
    usb_start();
    
	//inits the motors
	motors_init();

	// starts the IR proximity captors 
	proximity_start();

     //pour le son
    dac_start();
    //playMelodyStart();
    
    dcmi_start();
    po8030_start();

	//stars the threads for the pi regulator and the processing of the image
	//pi_regulator_start();
	control_robot_start(); 
	process_image_start(); 

	mic_start(&processAudioData);


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
