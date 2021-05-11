#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <process_image.h>
#include <sensors/proximity.h>

//wouhou
//simple PI regulator implementation
int16_t pi_regulator(float distance, float goal){

	float error = 0;
	float speed = 0;

	static float sum_error = 0;

	error = distance - goal;

	//disables the PI regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and 
	//the camera is a bit noisy
	if(fabs(error) < ERROR_THRESHOLD){
		return 0;
	}

	sum_error += error;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}

	speed = KP * error + KI * sum_error;

    return (int16_t)speed;
}

// fonction get_proxi_cm qui va récupérer la proximité et la transformer en cm pour pouvoir l'appeler dans pi_regulator
/*float get_proxi_cm(void){
	int proxi, proxi_right, proxi_left;
	float distance_cm;

	proxi_right = get_prox(0); //devant droite
	proxi_left = get_prox(7); // devant gauche

	if (proxi_rigt < proxi_left){ //on choisit de travailler avec le capteur à l'avant qui est le plus proche de l'obstacle
		proxi = proxi_right;
	}
	else {
		proxi = proxi_left;
	}

	distance_cm = DATATOCM*proxi; // on trouve DATATOCM avec des approximations
	return distance_cm;
}
*/

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed = 400;
    //int16_t speed_correction = 0;
    int diff_prox_left, diff_prox_right;
    //float distance_cm;



    while(1){

    	right_motor_set_speed(speed);
    	left_motor_set_speed(speed);

        time = chVTGetSystemTime();


        //right_motor_set_speed(0);
        //left_motor_set_speed(0);
        
        //chprintf((BaseSequentialStream *)&SD3, "capteur avant droit - 45  = %d\n", get_prox(1));
        //chprintf((BaseSequentialStream *)&SD3, "capteur avant droit = %d\n", get_prox(0));
        //chprintf((BaseSequentialStream *)&SD3, "capteur avant gauche - 45  = %d\n", get_prox(6));
        //chprintf((BaseSequentialStream *)&SD3, "capteur avant gauche = %d\n", get_prox(7));

        //chprintf((BaseSequentialStream *)&SD3, "capteur gauche = %d\n", get_prox(5));
        chprintf((BaseSequentialStream *)&SD3, "capteur droit = %d\n", get_prox(2));
        diff_prox_left = get_prox(7) - get_prox(6); 
        chprintf((BaseSequentialStream *)&SD3, " diff: %d \n", diff_prox_left);


        //distance_cm = get_prox(0)*DATATOCM;

        // Numérotation des capteurs : 
        // IR0 (front-right) + IR1 (front-right-45deg) + IR2 (right) + IR3 (back-right) 
        // IR4 (back-left) + IR5 (left) + IR6 (front-left-45deg) + IR7 (front-left)

        //s'il detecte qque chose devant lui
        if (get_prox(0) > 150 || get_prox(1) > 230 || get_prox(6) > 230 || get_prox(7) > 150 ) // à ajuster et à mesurer après
        //if (get_prox(0) > 150 || get_prox(1) > 250 || get_prox(6) > 250 || get_prox(7) > 150 ) // à ajuster et à mesurer après
        {

            //contournement d'un objet 
        	// si il detecte un objet devant lui mais pas sur les côtés : va contourner l'obstacle
        	if(get_prox(2) <= 70 && get_prox(5) <= 70 )  //quart de tour
            {

                // alignement à ce moment la ou un quart de tour 
                //avant de commencer le contournement, on voit en fonction de l'alignement du robot si on va faire 1/4 tour complet ou juste s'aligner

                //premier quart de tour 

                right_motor_set_speed(600);
                left_motor_set_speed(-600);
                chThdSleepMilliseconds(550); //durée de quart de tour (à faire varier avec la surface)
                // ca sera à supprimer si ok avec ce qui est en dessous. */


                //uniquement pour capteurs gauche 
               /* diff_prox_left = get_prox(7) - get_prox(6); 
                //diff_prox_right = get_prox(0) - get_prox(1); 

                if( fabs(diff_prox_left) > 55 /*&&  fabs(diff_prox_right) > 55*//*) { 
                    //rajout de la condition sur la frequence pour savoir de quel cote on tourne
                    chprintf((BaseSequentialStream *)&SD3, "cas 1\n diff: %d \n", diff_prox_left);
                    right_motor_set_speed(600);
                    left_motor_set_speed(-600);
                    chThdSleepMilliseconds(550); //durée de quart de tour (à faire varier avec la surface)
                }
                else /*if (fabs(diff_prox_left) < 55)*//* {
                    chprintf((BaseSequentialStream *)&SD3, "cas 2\n diff: %d \n", diff_prox_left);
                    while(fabs(diff_prox_left) < 55){
                        right_motor_set_speed(-600);
                        left_motor_set_speed(600); 
                        diff_prox_left = get_prox(7) - get_prox(6);  
                    }
                }
               /* else if (fabs(diff_prox_right) < 55) {
                    chprintf((BaseSequentialStream *)&SD3, "cas 3\n");
                    while(fabs(get_prox(0) - get_prox(1)) > 55){
                        right_motor_set_speed(600);
                        left_motor_set_speed(-600);    
                    }
                }*/
                //manque un cas si les 2 sont inf




                // voir si les 2 et 3 sont pas à changer 
                
                while(get_prox(2) > 60 && get_prox(3) > 60){ //valeurs trouvées avec essais 
                    //continue tout droit 
                    right_motor_set_speed(600);
                    left_motor_set_speed(600); 
                    chThdSleepMilliseconds(10); //peut etre adapter la durée pour éviter de nouvelles mesures 
                        
                }
                
                chThdSleepMilliseconds(300); //si on capte toujours l'angle sur le cote en tournant avec les capteurs à -45, tenter d'ajouter du temps ou baisser la sensi des capteurs
                
                //deuxième quart de tour
                right_motor_set_speed(-600);
                left_motor_set_speed(600);                    
                chThdSleepMilliseconds(550);
                   
                //continue tout droit 
                //right_motor_set_speed(600); 
                //left_motor_set_speed(600); 
                //chThdSleepMilliseconds(550);
        			
        	}
        	// quart de tour vers la gauche comme objet à droite
        	else if (get_prox(2) > 70 && get_prox(5) <= 70){
        		right_motor_set_speed(600);
        		left_motor_set_speed(-600);
        		chThdSleepMilliseconds(550);
    			
        	}
        	// quart de tour vers la droite comme objet à gauche
            else if (get_prox(5) > 70 && get_prox(2) <= 70){    
    		     right_motor_set_speed(-600);
    		     left_motor_set_speed(600);
                chThdSleepMilliseconds(550);
        		
        	}
        	// dans un cul de sac, fait demi tour et repart
        	else {
        		right_motor_set_speed(-600);
        		left_motor_set_speed(600);
        		chThdSleepMilliseconds(1100);
        		
        	}
        }

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}