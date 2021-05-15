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

#include <audio/play_melody.h>
//#include <audio/audio_thread.h>


/*
 *  Define used to identify each IR sensor of the epuck 
 */
#define FRONT_RIGHT_SENSOR              0
#define FRONT_RIGHT_45_SENSOR           1
#define RIGHT_SENSOR                    2 
#define REAR_RIGHT_SENSOR               3
#define FRONT_LEFT_SENSOR               7
#define FRONT_LEFT_45_SENSOR            6
#define LEFT_SENSOR                     5
#define REAR_LEFT_SENSOR                4

/*
 *  Define used for the duration of the movement of the robot
 */

#define TIME_QUARTER_TURN               550
#define TIME_HALF_TURN                  1100

/*
 *  Define used for the proximity threshold of the different sensors 
 */
#define THRESHOLD_PROX_SIDE             70
#define THRESHOLD_PROX_FRONT            150
#define THRESHOLD_PROX_FRONT_45         230

/*
 *  Define used for the different speeds of the robot
 */
#define TURNING_SPEED                   600
#define AHEAD_SPEED                     300
#define ZERO_SPEED                      0

/*
 *  Defines used for the different cases of the switch in our thread 
 */
#define CROSSWALK                       1
#define LEFT_TURN_AROUND                2
#define RIGHT_TURN_AROUND               3
#define QUARTER_TURN_LEFT               4
#define QUARTER_TURN_RIGHT              5
#define HALF_TURN                       6 
#define CHECK_BEFORE_TURNING_LEFT       7      
#define CHECK_BEFORE_TURNING_RIGHT      8
#define NO_INSTRUCTION                  10


static systime_t time_crossed; 
static bool cross = false; 
static int8_t turning_direction = 0;

/*
 *  Function allowing the robot to move straight ahead 
 */
void straight_ahead(void){
    right_motor_set_speed(AHEAD_SPEED);
    left_motor_set_speed(AHEAD_SPEED); 
}

/*
 *  Function stoping the robot 
 */
void robot_stop(void){
    right_motor_set_speed(ZERO_SPEED);
    left_motor_set_speed(ZERO_SPEED);
}

/*
 *  Functions allowing the robot to do a quarter turn to the left or to the right 
 */
void quarter_turn_right(void){

    right_motor_set_speed(-600); // voir ce qu'on fait 
    left_motor_set_speed(600);
    chThdSleepMilliseconds(550); 
}
void quarter_turn_left(void){

    right_motor_set_speed(600);
    left_motor_set_speed(-600);
    chThdSleepMilliseconds(550);

}

/*
 *  Function allowing the robot to do a half turn 
 */
void half_turn(void){
    right_motor_set_speed(-TURNING_SPEED);
    left_motor_set_speed(TURNING_SPEED);
    chThdSleepMilliseconds(TIME_HALF_TURN);
}

/*
 *  Functions checking that the space is clear before turning to the left or to the right 
 */
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

/*
 *  Function called when the robot wants to turn around an object to the left or to the right depending on the frequency 
 */
void turn_around(bool direction, unsigned int side_sensor, unsigned int rear_sensor){ // direction 1 : gauche, 0 : droite 

    systime_t time, time_start, time_stop, time_test;
    uint16_t diff_time; //on tente avec le 16 comme c'est une diff, si ca va pas, revenir au 32 ! 

    chprintf((BaseSequentialStream *)&SD3, "dans contournement \n");
    if(direction){
        chprintf((BaseSequentialStream *)&SD3, "direction = 1 \n");
        quarter_turn_left();
    }
    else {
        chprintf((BaseSequentialStream *)&SD3, "direction =  0\n");
        quarter_turn_right();
    }
    chprintf((BaseSequentialStream *)&SD3, "premier quart de tour \n");

    time_start = chVTGetSystemTime(); 

    while(get_prox(rear_sensor) > THRESHOLD_PROX_SIDE && get_prox(side_sensor) > THRESHOLD_PROX_SIDE && get_prox(FRONT_RIGHT_SENSOR) < THRESHOLD_PROX_FRONT && get_prox(FRONT_LEFT_SENSOR) < THRESHOLD_PROX_FRONT && !crosswalk_detected()){
        //continue tout droit 
        straight_ahead();
        chThdSleepMilliseconds(10); //peut etre adapter la durée pour éviter de nouvelles mesures 
    }

    if (get_prox(FRONT_RIGHT_SENSOR) > THRESHOLD_PROX_FRONT  || get_prox(FRONT_LEFT_SENSOR) > THRESHOLD_PROX_FRONT || crosswalk_detected()){
        chprintf((BaseSequentialStream *)&SD3, "j'ai vu qqchose \n");
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
        while(get_prox(side_sensor) > THRESHOLD_PROX_SIDE /*&& get_prox(4) > THRESHOLD_PROX_SIDE*/ && get_prox(FRONT_RIGHT_SENSOR) < THRESHOLD_PROX_FRONT && get_prox(FRONT_LEFT_SENSOR) < THRESHOLD_PROX_FRONT && !crosswalk_detected()){ //tant qu'il capte sur les côtés et que rien devant 
            //continue tout droit 
            straight_ahead();
            chThdSleepMilliseconds(10); //peut etre adapter la durée pour éviter de nouvelles mesures                     
        }

        if (get_prox(FRONT_RIGHT_SENSOR) > THRESHOLD_PROX_FRONT  || get_prox(FRONT_LEFT_SENSOR) > THRESHOLD_PROX_FRONT || crosswalk_detected()){ // si quelque chose apparait devant, on retourne dans la boucle principale pour savoir pas où l'éviter 
                             
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
            while (time_test < (time + diff_time) && get_prox(FRONT_RIGHT_SENSOR) < THRESHOLD_PROX_FRONT && get_prox(FRONT_LEFT_SENSOR) < THRESHOLD_PROX_FRONT && !crosswalk_detected()){ //on longe l'obstacle dans le même temps tout en vérifiant que rien devant 
                straight_ahead();
                chThdSleepMilliseconds(10); //peut etre adapter la durée pour éviter de nouvelles mesures
                time_test = chVTGetSystemTime();
                //chprintf((BaseSequentialStream *)&SD3, "time test  = %d\n", time_test); 
            }
            if (get_prox(FRONT_RIGHT_SENSOR) > THRESHOLD_PROX_FRONT  || get_prox(FRONT_LEFT_SENSOR) > THRESHOLD_PROX_FRONT || crosswalk_detected()){ // si quelque chose apparait devant, on retourne dans la boucle principale pour savoir pas où l'éviter 
                             
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

/*
 *  Function allowing our robot to "bark" when stopping at a cross walk 
 */
void bark(void){
    playNote(NOTE_CS4,400);
    playNote(NOTE_G4,200);
}

/*
 *  Function allowing our robot to wait for an order once reaching a cross walk and then crossin it 
 */
void crosswalk (void){

    robot_stop();
    bark();
    while (get_freq() != MOVE_FORWARD)
    {}
    time_crossed = chVTGetSystemTime() + MS2ST(10000); //duree de traversée du passage piéton à determiner en fonction de la vitesse
    cross = true; 
    straight_ahead(); 
}


/*
 *  Function deciding in which case our robot is in, then sending it to the thread to do the actions required
 */
uint8_t get_mode (void){
    // variables de la fonction : 

    chprintf((BaseSequentialStream *)&SD3, "capteur de droite : %d \n", get_prox(2)); 
    chprintf((BaseSequentialStream *)&SD3, "capteur de gauche : %d \n", get_prox(5)); 
    //s'il detecte qque chose devant lui
    if (crosswalk_detected()){
        chprintf((BaseSequentialStream *)&SD3, "cas 1 \n");

        if(cross == false){
            chprintf((BaseSequentialStream *)&SD3, "passage pieton \n"); 
            return CROSSWALK;
        }
    }
    else if (get_prox(FRONT_RIGHT_SENSOR) > THRESHOLD_PROX_FRONT || get_prox(FRONT_RIGHT_45_SENSOR) > THRESHOLD_PROX_FRONT_45 || get_prox(FRONT_LEFT_45_SENSOR) > THRESHOLD_PROX_FRONT_45 || get_prox(FRONT_LEFT_SENSOR) > THRESHOLD_PROX_FRONT ) // à ajuster et à mesurer après
    {
        chprintf((BaseSequentialStream *)&SD3, "objet en face \n");
        //contournement d'un objet: si il detecte un objet devant lui mais pas sur les côtés : va contourner l'obstacle
        if(get_prox(RIGHT_SENSOR) <= THRESHOLD_PROX_SIDE && get_prox(LEFT_SENSOR) <= THRESHOLD_PROX_SIDE )
        {
            chprintf((BaseSequentialStream *)&SD3, "rien sur les cotes \n");
            turning_direction = get_freq(); 
// Dans ces boucles, critères de la fréquence à adapter plus tard  
            if(turning_direction == TURN_LEFT){ //contournement par la gauche de l'obstacle 
                chprintf((BaseSequentialStream *)&SD3, "contournement par gauche \n");
                return LEFT_TURN_AROUND; //left_turn_around(); 
            }
            else if (turning_direction == TURN_RIGHT){ //contournement par la droite de l'obstacle  
                chprintf((BaseSequentialStream *)&SD3, "contournement par droite \n"); 
                return RIGHT_TURN_AROUND; //right_turn_around();
            }
            else { // si pas de fréquence ou pas dans les gammes de fréquence, on attend qu'on nous en donne une
                while (turning_direction == DONT_TURN){ //ajouter condition sur freq 3 ? 
                    //chprintf((BaseSequentialStream *)&SD3, "pas de direction \n"); 
                    robot_stop();
                    turning_direction = get_freq(); 
                }
            }   
        }
        // quart de tour vers la gauche comme objet à droite
        else if (get_prox(RIGHT_SENSOR) > THRESHOLD_PROX_SIDE && get_prox(LEFT_SENSOR) <= THRESHOLD_PROX_SIDE){
            chprintf((BaseSequentialStream *)&SD3, "objet sur la droite \n"); 
            return QUARTER_TURN_LEFT; //quarter_turn_left();
        }
        // quart de tour vers la droite quand objet à gauche
        else if (get_prox(LEFT_SENSOR) > THRESHOLD_PROX_SIDE && get_prox(RIGHT_SENSOR) <= THRESHOLD_PROX_SIDE){
            chprintf((BaseSequentialStream *)&SD3, "objet sur la gauche \n"); 
            return QUARTER_TURN_RIGHT; //quarter_turn_right();
        }
        // dans un cul de sac, fait demi tour
        else {
            chprintf((BaseSequentialStream *)&SD3, "cul de sac \n"); 
            return HALF_TURN; //half_turn();
        }
    }
    else { // si pas d'obstacle, avance et attend un ordre de son maitre pour tourner
        chprintf((BaseSequentialStream *)&SD3, "cas 3 \n");
        if (turning_direction == get_freq() || cross == true){
        }
        if (turning_direction != get_freq() && cross == false) { 
            turning_direction = get_freq();
            if(turning_direction == TURN_LEFT){ //ordre de tourner à gauche
                chprintf((BaseSequentialStream *)&SD3, "ordre de tourner à gauche \n"); 
                return CHECK_BEFORE_TURNING_LEFT; //check_before_turning_left();
            } 
            if(turning_direction == TURN_RIGHT){ // ordre de touner à droite
                chprintf((BaseSequentialStream *)&SD3, "ordre de tourner à droite \n"); 
                return CHECK_BEFORE_TURNING_RIGHT; //check_before_turning_right();
            }
        } 
    }
    return NO_INSTRUCTION;
}


static THD_WORKING_AREA(waControlRobot, 256);
static THD_FUNCTION(ControlRobot, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time; //time_start, time_stop, time_test, time_crossed;
    //uint32_t diff_time; 
    //uint16_t lineWidth = 0;

   // int diff_prox_left, diff_prox_right; // pour l'alignement
    //float distance_cm;
    //int16_t turning_direction = 0; 

    //bool order = true;
    //bool cross = false; 

    while(1){
        chprintf((BaseSequentialStream *)&SD3, "debut du while  \n");
        
        time = chVTGetSystemTime();
        
        straight_ahead();

        if (chVTGetSystemTime() > time_crossed){
            cross = false; 
        }
        
        switch(get_mode()){

            case CROSSWALK: 
            crosswalk(); 
                break; 

            case LEFT_TURN_AROUND: 
            left_turn_around();
                break; 

            case RIGHT_TURN_AROUND: 
            right_turn_around(); 
                break; 

            case QUARTER_TURN_LEFT: 
            quarter_turn_left(); 
                break; 

            case QUARTER_TURN_RIGHT: 
            quarter_turn_right(); 
                break; 

            case HALF_TURN: 
            half_turn(); 
                break; 

            case CHECK_BEFORE_TURNING_LEFT:
            check_before_turning_left(); 
                break; 

            case CHECK_BEFORE_TURNING_RIGHT:
            check_before_turning_right(); 
                break; 

            case NO_INSTRUCTION:
                break; 

        }
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}
    

void control_robot_start(void){
	chThdCreateStatic(waControlRobot, sizeof(waControlRobot), NORMALPRIO, ControlRobot, NULL);
}
