#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <audio_processing.h>
#include <control_robot.h>
#include <process_image.h>
#include <sensors/proximity.h>


#define FRONT_RIGHT_SENSOR      0
#define FRONT_RIGHT_45_SENSOR   1
#define RIGHT_SENSOR            2 
#define REAR_RIGHT_SENSOR       3
#define FRONT_LEFT_SENSOR       7
#define FRONT_LEFT_45_SENSOR    6
#define LEFT_SENSOR             5
#define REAR_LEFT_SENSOR        4


//Functions allowing the robot to rotate during the simulation
void straight_ahead(void){
    right_motor_set_speed(300);
    left_motor_set_speed(300); 
}
void robot_stop(void){
    right_motor_set_speed(0);
    left_motor_set_speed(0);
}
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
void check_before_turning_left(void){

    while(/*get_prox(4) > 60 &&*/ get_prox(5) > 60 && get_prox(0) < 150 && get_prox(7) < 150){ //valeurs trouvées avec essais 
        //continue tout droit 
        straight_ahead(); 
        chThdSleepMilliseconds(10); //peut etre adapter la durée pour éviter de nouvelles mesures      
     }

    if (get_prox(0) > 150  || get_prox(7) > 150){} // si rencontre un objet en face-> sort
    else {
                        
        chThdSleepMilliseconds(300); //si on capte toujours l'angle sur le cote en tournant avec les capteurs à -45, tenter d'ajouter du temps ou baisser la sensi des capteurs
        quarter_turn_left(); 
    }

}
void check_before_turning_right(void){
    while(get_prox(2) > 60 && /*get_prox(3) > 60 && */get_prox(0) < 150 && get_prox(7) < 150){ //valeurs trouvées avec essais 
    //continue tout droit 
    straight_ahead();
    chThdSleepMilliseconds(10); //peut etre adapter la durée pour éviter de nouvelles mesures
                            
    }

    if (get_prox(0) > 150  || get_prox(7) > 150){} // si rencontre un objet en face -> sort de la boucle
    else {
        
        chThdSleepMilliseconds(300); //si on capte toujours l'angle sur le cote en tournant avec les capteurs à -45, tenter d'ajouter du temps ou baisser la sensi des capteurs
        quarter_turn_right(); // a changer si premier quart de tour gauche
    }

}


void turn_around(bool direction, unsigned int side_sensor, unsigned int rear_sensor){ // direction 1 : gauche, 0 : droite 

    systime_t time, time_start, time_stop, time_test;
    uint32_t diff_time;

    chprintf((BaseSequentialStream *)&SD3, "dans contournement \n");
    if(direction){
        quarter_turn_left();
    }
    else {
        quarter_turn_right();
    } 
    chprintf((BaseSequentialStream *)&SD3, "premier quart de tour \n");

    while(get_prox(rear_sensor) > 60 && get_prox(side_sensor) > 60 && get_prox(FRONT_RIGHT_SENSOR) < 150 && get_prox(FRONT_LEFT_SENSOR) < 150){
        //continue tout droit 
        straight_ahead();
        chThdSleepMilliseconds(10); //peut etre adapter la durée pour éviter de nouvelles mesures 
    }

    if (get_prox(FRONT_RIGHT_SENSOR) > 150  || get_prox(FRONT_LEFT_SENSOR) > 150){

    }
    else {                  
        // sinon on continue  
        //chThdSleepMilliseconds(300); //à voir si on rajoute pas ça / si on capte toujours l'angle sur le cote en tournant avec les capteurs à -45, tenter d'ajouter du temps ou baisser la sensi des capteurs
        time_stop = chVTGetSystemTime(); 
        diff_time = time_stop - time_start; 
                        
// C deuxième quart de tour
        if(direction){
            quarter_turn_right();
        }
        else {
            quarter_turn_left();
        }
        chprintf((BaseSequentialStream *)&SD3, "deuxieme quart de tour \n");
                       
        // on le force à repartir tout droit 
// D1
        straight_ahead();
        chThdSleepMilliseconds(900);

// D2 tant qu'il y a quelque chose sur le coté, robot avance
        while(get_prox(side_sensor) > 60 /*&& get_prox(4) > 60*/ && get_prox(FRONT_RIGHT_SENSOR) < 150 && get_prox(FRONT_LEFT_SENSOR) < 150){ //tant qu'il capte sur les côtés et que rien devant 
            //continue tout droit 
            straight_ahead();
            chThdSleepMilliseconds(10); //peut etre adapter la durée pour éviter de nouvelles mesures                     
        }

        if (get_prox(FRONT_RIGHT_SENSOR) > 150  || get_prox(FRONT_LEFT_SENSOR) > 150){ // si quelque chose apparait devant, on retourne dans la boucle principale pour savoir pas où l'éviter 
                             
        }
        else {
            //pour etre sur d'être dégagé 
            chThdSleepMilliseconds(300); 

// E troixième quart de tour
            if(direction){
                quarter_turn_right();
            }
            else {
                quarter_turn_left();    
            }
            chprintf((BaseSequentialStream *)&SD3, "troixieme quart de tour \n");
            
                            
            time = chVTGetSystemTime();
            time_test = chVTGetSystemTime();
            diff_time = time_stop - time_start; 
                            
//F On avance de la meme durée 
            while (time_test < (time + (time_stop - time_start)) && get_prox(FRONT_RIGHT_SENSOR) < 150 && get_prox(FRONT_LEFT_SENSOR) < 150){ //on longe l'obstacle dans le même temps tout en vérifiant que rien devant 
                straight_ahead();
                chThdSleepMilliseconds(10); //peut etre adapter la durée pour éviter de nouvelles mesures
                time_test = chVTGetSystemTime();
                //chprintf((BaseSequentialStream *)&SD3, "time test  = %d\n", time_test);
            }
            if (get_prox(FRONT_RIGHT_SENSOR) > 150  || get_prox(FRONT_LEFT_SENSOR) > 150){ // si quelque chose apparait devant, on retourne dans la boucle principale pour savoir pas où l'éviter 
                             
            }
            else{
// G quatrieme quart de tour 
                if(direction){
                    quarter_turn_left(); 
                }
                else{
                    quarter_turn_right();    
                }
                chprintf((BaseSequentialStream *)&SD3, "quatrieme quart de tour \n");
            }
        } 
    } 
}

void left_turn_around(void){
    chprintf((BaseSequentialStream *)&SD3, "contournement gauche \n");
    turn_around(true, RIGHT_SENSOR, REAR_RIGHT_SENSOR); 
}

void right_turn_around(void){
    chprintf((BaseSequentialStream *)&SD3, "contournement droite \n");
    turn_around(false, LEFT_SENSOR, REAR_LEFT_SENSOR); 
}



static THD_WORKING_AREA(waControlRobot, 256);
static THD_FUNCTION(ControlRobot, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time, time_start, time_stop, time_test, time_crossed;
    uint32_t diff_time; 
    uint16_t lineWidth = 0;

   // int diff_prox_left, diff_prox_right; // pour l'alignement
    //float distance_cm;
    int16_t turning_direction = 0; 

    bool order = true;
    bool cross = false; 

    while(1){
        chprintf((BaseSequentialStream *)&SD3, "debut du while  \n");
        
        time = chVTGetSystemTime();
        
    	straight_ahead();
        lineWidth = get_width();
        
        //chprintf((BaseSequentialStream *)&SD3, "capteur avant droit - 45  = %d\n", get_prox(1));
        //chprintf((BaseSequentialStream *)&SD3, "capteur avant droit = %d\n", get_prox(0));
        //chprintf((BaseSequentialStream *)&SD3, "capteur avant gauche - 45  = %d\n", get_prox(6));
        //chprintf((BaseSequentialStream *)&SD3, "capteur avant gauche = %d\n", get_prox(7));


        // Numérotation des capteurs : 
        // IR0 (front-right) + IR1 (front-right-45deg) + IR2 (right) + IR3 (back-right) 
        // IR4 (back-left) + IR5 (left) + IR6 (front-left-45deg) + IR7 (front-left)

        //s'il detecte qque chose devant lui
         chprintf((BaseSequentialStream *)&SD3, "lineWidthdth 1 %d \n ", lineWidth);
        if (lineWidth > 150 || cross == true ){
            chprintf((BaseSequentialStream *)&SD3, "cas 1 \n");

            if(cross == false){
                robot_stop();
                while (get_freq() != 3)
                {}
                cross = true; 
                time_crossed = chVTGetSystemTime() + MS2ST(10000); //duree de traversée du passage piéton à determiner en fonction de la vitesse 
                straight_ahead(); 
            }
            else {
                if(chVTGetSystemTime() > time_crossed )
                {
                    cross = false; 
                }
            }
            
        }
        else if (get_prox(0) > 150 || get_prox(1) > 230 || get_prox(6) > 230 || get_prox(7) > 150 ) // à ajuster et à mesurer après
        {
            chprintf((BaseSequentialStream *)&SD3, "cas 2 \n");
            chprintf((BaseSequentialStream *)&SD3, "quelque chose en face \n");

            //contournement d'un objet: si il detecte un objet devant lui mais pas sur les côtés : va contourner l'obstacle
        	if(get_prox(2) <= 70 && get_prox(5) <= 70 )
            {
                chprintf((BaseSequentialStream *)&SD3, "rien sur les cotes \n");

                turning_direction = get_freq(); 
// Dans ces boucles, critères de la fréquence à adapter plus tard  
                if(turning_direction == 1){ //contournement par la gauche de l'obstacle  
                    chprintf((BaseSequentialStream *)&SD3, "je vais à gauche \n");
                    left_turn_around(); 
                }
                else if (turning_direction == 2){ //contournement par la droite de l'obstacle 
                    chprintf((BaseSequentialStream *)&SD3, "je vais à droite \n");   
                    right_turn_around();
                }
                else { // si pas de fréquence ou pas dans les gammes de fréquence, on attend qu'on nous en donne une
                    while (turning_direction == 0){
                        robot_stop();
                        turning_direction = get_freq(); 
                    }
                }	
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
         else { // si pas d'obstacle, avance et attend un ordre de son maitre pour tourner
            chprintf((BaseSequentialStream *)&SD3, "cas 3 \n");
            if (turning_direction == get_freq() && cross ==false){ 
                order = false;
            }
            else if (cross == false ) { 
                order = true;
                turning_direction = get_freq();
            } 
        
            if (order == 1 && cross == false) {

                if(turning_direction == 1){ //ordre de tourner à gauche
                    
                    check_before_turning_left();
                } 
                if(turning_direction == 2){ // ordre de touner à droite

                    check_before_turning_right();
                }
                order = false;
            }

        }
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}
    

void control_robot_start(void){
	chThdCreateStatic(waControlRobot, sizeof(waControlRobot), NORMALPRIO, ControlRobot, NULL);
}
