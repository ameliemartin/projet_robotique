#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <audio_processing.h>
#include <control_robot.h>
//#include <process_image.h>
#include <sensors/proximity.h>





//Functions allowing the robot to rotate during the simulation
void quarter_turn_right(void){

    right_motor_set_speed(-600);
    left_motor_set_speed(600);
    chThdSleepMilliseconds(550); //durée de quart de tour (à faire varier avec la surface)

}
void quarter_turn_left(void){

    right_motor_set_speed(600);
    left_motor_set_speed(-600);
    chThdSleepMilliseconds(550);

}
void half_turn(void){
    right_motor_set_speed(-600);
    left_motor_set_speed(600);
    chThdSleepMilliseconds(1100);
}



static THD_WORKING_AREA(waControlRobot, 256);
static THD_FUNCTION(ControlRobot, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed = 600; // besoin de mettre un int ? pas juste une cte ? 
   // int diff_prox_left, diff_prox_right; // pour l'alignement
    //float distance_cm;
    int16_t turning_direction; 

    right_motor_set_speed(600);
    left_motor_set_speed(600);


    while(1){
        
        time = chVTGetSystemTime();

        
    	right_motor_set_speed(speed);
    	left_motor_set_speed(speed);

        time = chVTGetSystemTime();
        
        //chprintf((BaseSequentialStream *)&SD3, "capteur avant droit - 45  = %d\n", get_prox(1));
        //chprintf((BaseSequentialStream *)&SD3, "capteur avant droit = %d\n", get_prox(0));
        //chprintf((BaseSequentialStream *)&SD3, "capteur avant gauche - 45  = %d\n", get_prox(6));
        //chprintf((BaseSequentialStream *)&SD3, "capteur avant gauche = %d\n", get_prox(7));


        // Numérotation des capteurs : 
        // IR0 (front-right) + IR1 (front-right-45deg) + IR2 (right) + IR3 (back-right) 
        // IR4 (back-left) + IR5 (left) + IR6 (front-left-45deg) + IR7 (front-left)

        //s'il detecte qque chose devant lui
        if (get_prox(0) > 150 || get_prox(1) > 230 || get_prox(6) > 230 || get_prox(7) > 150 ) // à ajuster et à mesurer après
        //if (get_prox(0) > 150 || get_prox(1) > 250 || get_prox(6) > 250 || get_prox(7) > 150 ) // à ajuster et à mesurer après
        {
            //contournement d'un objet: si il detecte un objet devant lui mais pas sur les côtés : va contourner l'obstacle
        	if(get_prox(2) <= 70 && get_prox(5) <= 70 )
            {
                turning_direction = get_freq(); 

                if(turning_direction == 1){ //contournement par la gauche 
                    quarter_turn_left(); 


                    while(get_prox(2) > 60 && get_prox(3) > 60 && get_prox(0) < 150 && get_prox(7) < 150){ //valeurs trouvées avec essais 
                        //continue tout droit 
                        right_motor_set_speed(600);
                        left_motor_set_speed(600); 
                        chThdSleepMilliseconds(10); //peut etre adapter la durée pour éviter de nouvelles mesures
                            
                    }

                    if (get_prox(0) > 150  || get_prox(7) > 150){
                         
                    }
                    else {
                        chThdSleepMilliseconds(300); //si on capte toujours l'angle sur le cote en tournant avec les capteurs à -45, tenter d'ajouter du temps ou baisser la sensi des capteurs
                        
                        //deuxième quart de tour
                       quarter_turn_right(); // a changer si premier quart de tour gauche
                    } 
                }

                else if (turning_direction == 2){
                    quarter_turn_right(); 

                    while(get_prox(4) > 60 && get_prox(5) > 60 && get_prox(0) < 150 && get_prox(7) < 150){ //valeurs trouvées avec essais 
                        //continue tout droit 
                        right_motor_set_speed(600);
                        left_motor_set_speed(600); 
                        chThdSleepMilliseconds(10); //peut etre adapter la durée pour éviter de nouvelles mesures      
                    }

                    if (get_prox(0) > 150  || get_prox(7) > 150){

                    }
                    else {
                        
                        chThdSleepMilliseconds(300); //si on capte toujours l'angle sur le cote en tournant avec les capteurs à -45, tenter d'ajouter du temps ou baisser la sensi des capteurs
                        
                        //deuxième quart de tour
                       quarter_turn_left(); 
                    } 
                }
                else { // mettre un cas si pas de fréquence ou fréquence pas dans l'échelle
                    while (turning_direction == 0){
                        right_motor_set_speed(0);
                        left_motor_set_speed(0);
                        turning_direction = get_freq(); 
                    }
                }


                // alignement à ce moment la ou un quart de tour 
                //avant de commencer le contournement, on voit en fonction de l'alignement du robot si on va faire 1/4 tour complet ou juste s'aligner

                //premier quart de tour (dépendra de la freq pour l'instant à gauche)
                //quarter_turn_left(); // ca sera à supprimer si ok avec ce qui est en dessous. */

                // ALIGNEMENT
                //uniquement pour capteurs gauche 
               /* diff_prox_left = get_prox(7) - get_prox(6); 
                //diff_prox_right = get_prox(0) - get_prox(1); 

                if( fabs(diff_prox_left) > 55 /*&&  fabs(diff_prox_right) > 55*//*) { 
                   
                    chprintf((BaseSequentialStream *)&SD3, "cas 1\n diff: %d \n", diff_prox_left);
                    quarter_turn_left();  //rajout de la condition sur la frequence pour savoir de quel cote on tourne
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

                //if (get_prox(0) < 150  || get_prox(7) < 150){
                    
                  //  chprintf((BaseSequentialStream *)&SD3, "quelque chose bloque en face juste apres le quart de tour \n");
                     
                //}
                
                /*while(get_prox(2) > 60 && get_prox(3) > 60 && get_prox(0) < 150 && get_prox(7) < 150){ //valeurs trouvées avec essais 
                    //continue tout droit 
                    right_motor_set_speed(600);
                    left_motor_set_speed(600); 
                    chThdSleepMilliseconds(10); //peut etre adapter la durée pour éviter de nouvelles mesures

                        
                }

                if (get_prox(0) > 150  || get_prox(7) > 150){
                    
                    chprintf((BaseSequentialStream *)&SD3, "quelque chose bloque en face \n");
                     
                }
                else {
                    chprintf((BaseSequentialStream *)&SD3, "rien ne bloque  \n");
                    chprintf((BaseSequentialStream *)&SD3, "objet depasse \n");
                    
                    chThdSleepMilliseconds(300); //si on capte toujours l'angle sur le cote en tournant avec les capteurs à -45, tenter d'ajouter du temps ou baisser la sensi des capteurs
                    
                    //deuxième quart de tour
                   quarter_turn_right(); // a changer si premier quart de tour gauche
                   chprintf((BaseSequentialStream *)&SD3, "objet contourne \n");
                }  */       			
        	}
        	// quart de tour vers la gauche comme objet à droite
        	else if (get_prox(2) > 70 && get_prox(5) <= 70){
                chprintf((BaseSequentialStream *)&SD3, "objet sur la droite \n");
        		quarter_turn_left();
        	}
        	// quart de tour vers la droite quand objet à gauche
            else if (get_prox(5) > 70 && get_prox(2) <= 70){
                chprintf((BaseSequentialStream *)&SD3, "objet sur la gauche \n");
                quarter_turn_right();
        	}
        	// dans un cul de sac, fait demi tour
        	else {
                chprintf((BaseSequentialStream *)&SD3, "dans un cul de sac \n");
        		half_turn();
        	}
        }
        //100Hz


    
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}
    

void control_robot_start(void){
	chThdCreateStatic(waControlRobot, sizeof(waControlRobot), NORMALPRIO, ControlRobot, NULL);
}
