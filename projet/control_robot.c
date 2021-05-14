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

// sensors define
#define FRONT_RIGHT_SENSOR      0
#define FRONT_RIGHT_45_SENSOR   1
#define RIGHT_SENSOR            2 
#define REAR_RIGHT_SENSOR       3
#define FRONT_LEFT_SENSOR       7
#define FRONT_LEFT_45_SENSOR    6
#define LEFT_SENSOR             5
#define REAR_LEFT_SENSOR        4

// frequency define
#define DONT_TURN               0
#define TURN_LEFT               1
#define TURN_RIGHT              2
#define MOVE_FORWARD            3

// times define
#define TIME_QUARTER_TURN       550
#define TIME_HALF_TURN          1100

// proximity threshold define
#define THRESHOLD_PROX_SIDE     60
#define THRESHOLD_PROX_FRONT    150
#define THRESHOLD_PROX_FRONT_45 230

// speed define 
#define TURNING_SPEED           600
#define AHEAD_SPEED             300
#define ZERO_SPEED              0


//Functions allowing the robot to rotate during the simulation
void straight_ahead(void){
    right_motor_set_speed(AHEAD_SPEED);
    left_motor_set_speed(AHEAD_SPEED); 
}
void robot_stop(void){
    right_motor_set_speed(ZERO_SPEED);
    left_motor_set_speed(ZERO_SPEED);
}
void quarter_turn_right(void){

    right_motor_set_speed(-TURNING_SPEED);
    left_motor_set_speed(TURNING_SPEED);
    chThdSleepMilliseconds(TIME_QUARTER_TURN); //durée de quart de tour (à faire varier avec la surface)

}
void quarter_turn_left(void){

    right_motor_set_speed(TURNING_SPEED);
    left_motor_set_speed(-TURNING_SPEED);
    chThdSleepMilliseconds(TIME_QUARTER_TURN);

}
void half_turn(void){
    right_motor_set_speed(-TURNING_SPEED);
    left_motor_set_speed(TURNING_SPEED);
    chThdSleepMilliseconds(TIME_HALF_TURN);
}
void check_before_turning_left(void){

    while(/*get_prox(REAR_LEFT_SENSOR) > THRESHOLD_PROX_SIDE &&*/ get_prox(LEFT_SENSOR) > THRESHOLD_PROX_SIDE && get_prox(FRONT_RIGHT_SENSOR) < THRESHOLD_PROX_FRONT && get_prox(FRONT_LEFT_SENSOR) < THRESHOLD_PROX_FRONT){ //valeurs trouvées avec essais 
        //continue tout droit 
        straight_ahead(); 
        chThdSleepMilliseconds(10); //peut etre adapter la durée pour éviter de nouvelles mesures      
     }

    if (get_prox(FRONT_RIGHT_SENSOR) > THRESHOLD_PROX_FRONT  || get_prox(FRONT_LEFT_SENSOR) > THRESHOLD_PROX_FRONT){} // si rencontre un objet en face-> sort
    else {
                        
        chThdSleepMilliseconds(300); //si on capte toujours l'angle sur le cote en tournant avec les capteurs à -45, tenter d'ajouter du temps ou baisser la sensi des capteurs
        quarter_turn_left(); 
    }

}
void check_before_turning_right(void){
    while(get_prox(RIGHT_SENSOR) > THRESHOLD_PROX_SIDE && /*get_prox(REAR_RIGHT_SENSOR) > THRESHOLD_PROX_SIDE && */get_prox(FRONT_RIGHT_SENSOR) < THRESHOLD_PROX_FRONT && get_prox(FRONT_LEFT_SENSOR) < THRESHOLD_PROX_FRONT){ //valeurs trouvées avec essais 
    //continue tout droit 
    straight_ahead();
    chThdSleepMilliseconds(10); //peut etre adapter la durée pour éviter de nouvelles mesures
                            
    }

    if (get_prox(FRONT_RIGHT_SENSOR) > THRESHOLD_PROX_FRONT  || get_prox(FRONT_LEFT_SENSOR) > THRESHOLD_PROX_FRONT){} // si rencontre un objet en face -> sort de la boucle
    else {
        
        chThdSleepMilliseconds(300); //si on capte toujours l'angle sur le cote en tournant avec les capteurs à -45, tenter d'ajouter du temps ou baisser la sensi des capteurs
        quarter_turn_right(); // a changer si premier quart de tour gauche
    }

}


void turn_around(bool direction, unsigned int side_sensor, unsigned int rear_sensor){ // direction 1 : gauche, 0 : droite 

    systime_t time, time_start, time_stop, time_test;
    uint16_t diff_time; //on tente avec le 16 comme c'est une diff, si ca va pas, revenir au 32 ! 

    chprintf((BaseSequentialStream *)&SD3, "dans contournement \n");
    if(direction){
        quarter_turn_left();
    }
    else {
        quarter_turn_right();
    }
    chprintf((BaseSequentialStream *)&SD3, "premier quart de tour \n");

    time_start = chVTGetSystemTime(); 

    while(get_prox(rear_sensor) > THRESHOLD_PROX_SIDE && get_prox(side_sensor) > THRESHOLD_PROX_SIDE && get_prox(FRONT_RIGHT_SENSOR) < THRESHOLD_PROX_FRONT && get_prox(FRONT_LEFT_SENSOR) < THRESHOLD_PROX_FRONT && !crosswalk()){
        //continue tout droit 
        straight_ahead();
        chThdSleepMilliseconds(10); //peut etre adapter la durée pour éviter de nouvelles mesures 
    }

    if (get_prox(FRONT_RIGHT_SENSOR) > THRESHOLD_PROX_FRONT  || get_prox(FRONT_LEFT_SENSOR) > THRESHOLD_PROX_FRONT || crosswalk()){

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
        while(get_prox(side_sensor) > THRESHOLD_PROX_SIDE /*&& get_prox(4) > THRESHOLD_PROX_SIDE*/ && get_prox(FRONT_RIGHT_SENSOR) < THRESHOLD_PROX_FRONT && get_prox(FRONT_LEFT_SENSOR) < THRESHOLD_PROX_FRONT && !crosswalk()){ //tant qu'il capte sur les côtés et que rien devant 
            //continue tout droit 
            straight_ahead();
            chThdSleepMilliseconds(10); //peut etre adapter la durée pour éviter de nouvelles mesures                     
        }

        if (get_prox(FRONT_RIGHT_SENSOR) > THRESHOLD_PROX_FRONT  || get_prox(FRONT_LEFT_SENSOR) > THRESHOLD_PROX_FRONT || crosswalk()){ // si quelque chose apparait devant, on retourne dans la boucle principale pour savoir pas où l'éviter 
                             
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
            chprintf((BaseSequentialStream *)&SD3, "toisieme quart de tour \n");
                           
            time = chVTGetSystemTime();
            time_test = chVTGetSystemTime();
                            
//F On avance de la meme durée 
            while (time_test < (time + diff_time) && get_prox(FRONT_RIGHT_SENSOR) < THRESHOLD_PROX_FRONT && get_prox(FRONT_LEFT_SENSOR) < THRESHOLD_PROX_FRONT && !crosswalk()){ //on longe l'obstacle dans le même temps tout en vérifiant que rien devant 
                straight_ahead();
                chThdSleepMilliseconds(10); //peut etre adapter la durée pour éviter de nouvelles mesures
                time_test = chVTGetSystemTime();
                //chprintf((BaseSequentialStream *)&SD3, "time test  = %d\n", time_test); 
            }
            if (get_prox(FRONT_RIGHT_SENSOR) > THRESHOLD_PROX_FRONT  || get_prox(FRONT_LEFT_SENSOR) > THRESHOLD_PROX_FRONT || crosswalk()){ // si quelque chose apparait devant, on retourne dans la boucle principale pour savoir pas où l'éviter 
                             
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
    turn_around(true, RIGHT_SENSOR, REAR_RIGHT_SENSOR); //comme on tourne à gauche, on prend les capteurs de droite 
}

void right_turn_around(void){
    chprintf((BaseSequentialStream *)&SD3, "contournement droite \n");
    turn_around(false, LEFT_SENSOR, REAR_LEFT_SENSOR);  //1er argument dans turn around : true aller à gauche, false aller à droite 
}



static THD_WORKING_AREA(waControlRobot, 256);
static THD_FUNCTION(ControlRobot, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time, time_crossed = 0; //time_start, time_stop, time_test, time_crossed;
    //uint32_t diff_time; 

   // int diff_prox_left, diff_prox_right; // pour l'alignement
    //float distance_cm;
    int8_t turning_direction = 0; 

    bool order = true;
    bool cross = false; 

    while(1){
        //chprintf((BaseSequentialStream *)&SD3, "debut du while  \n");
        
        time = chVTGetSystemTime();
        
    	straight_ahead();
        
        chprintf((BaseSequentialStream *)&SD3, "capteur avant droit - 45  = %d\n", get_prox(1));
        chprintf((BaseSequentialStream *)&SD3, "capteur avant droit = %d\n", get_prox(0));
        chprintf((BaseSequentialStream *)&SD3, "capteur avant gauche - 45  = %d\n", get_prox(6));
        chprintf((BaseSequentialStream *)&SD3, "capteur avant gauche = %d\n", get_prox(7));


        if (crosswalk() && chVTGetSystemTime() > time_crossed)
        {
            cross = false; 
        }
        // Numérotation des capteurs : 
        // IR0 (front-right) + IR1 (front-right-45deg) + IR2 (right) + IR3 (back-right) 
        // IR4 (back-left) + IR5 (left) + IR6 (front-left-45deg) + IR7 (front-left)

        //s'il detecte qque chose devant lui
         chprintf((BaseSequentialStream *)&SD3, "lineWidth  %d \n ", lineWidth);
        if (crosswalk()){
            chprintf((BaseSequentialStream *)&SD3, "cas 1 \n");

            if(cross == false){
                robot_stop();
                while (get_freq() != MOVE_FORWARD)
                {}
                time_crossed = chVTGetSystemTime() + MS2ST(10000); //duree de traversée du passage piéton à determiner en fonction de la vitesse
                cross = true; 
                straight_ahead(); 
            }
            /*else {
                if(chVTGetSystemTime() > time_crossed )
                {
                    cross = false; 
                }
            }*/
            
        }
        else if (get_prox(FRONT_RIGHT_SENSOR) > THRESHOLD_PROX_FRONT || get_prox(FRONT_RIGHT_45_SENSOR) > THRESHOLD_PROX_FRONT_45 || get_prox(FRONT_LEFT_45_SENSOR) > THRESHOLD_PROX_FRONT_45 || get_prox(FRONT_LEFT_SENSOR) > THRESHOLD_PROX_FRONT ) // à ajuster et à mesurer après
        {
            chprintf((BaseSequentialStream *)&SD3, "cas 2 \n");
            //contournement d'un objet: si il detecte un objet devant lui mais pas sur les côtés : va contourner l'obstacle
        	if(get_prox(RIGHT_SENSOR) <= 70 && get_prox(LEFT_SENSOR) <= 70 )
            {
                chprintf((BaseSequentialStream *)&SD3, "rien sur les cotes \n");

                turning_direction = get_freq(); 
// Dans ces boucles, critères de la fréquence à adapter plus tard  
                if(turning_direction == TURN_LEFT){ //contournement par la gauche de l'obstacle  
                    left_turn_around(); 
                }
                else if (turning_direction == TURN_RIGHT){ //contournement par la droite de l'obstacle   
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
        	else if (get_prox(RIGHT_SENSOR) > THRESHOLD_PROX_SIDE && get_prox(LEFT_SENSOR) <= THRESHOLD_PROX_SIDE){
                chprintf((BaseSequentialStream *)&SD3, "objet sur la droite \n");
        		quarter_turn_left();
        	}
        	// quart de tour vers la droite quand objet à gauche
            else if (get_prox(LEFT_SENSOR) > THRESHOLD_PROX_SIDE && get_prox(RIGHT_SENSOR) <= THRESHOLD_PROX_SIDE){
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
        
            if (order == true && cross == false) {

                if(turning_direction == TURN_LEFT){ //ordre de tourner à gauche  
                chprintf((BaseSequentialStream *)&SD3, "on me dit de touner à gauche \n");                  
                   check_before_turning_left();
                } 
                if(turning_direction == TURN_RIGHT){ // ordre de touner à droite
                    chprintf((BaseSequentialStream *)&SD3, "on me dit de touner à droite \n");  
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
