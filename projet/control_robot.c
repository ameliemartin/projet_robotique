/*
File : control_robot.c
Author : Amelie Martin  & Carla Paillardon
Date : 16 may 2021

Allows the robot to react to every stimulation it can get. 
*/

/*
 *  Different includes used in this file 
 */
#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <main.h>
#include <motors.h>
#include <control_robot.h>
#include <sensors/proximity.h>
#include <process_image.h>
#include <leds.h>
#include <audio_processing.h>
#include <audio/play_melody.h>

/*
 *  Defines used for the different cases of the switch in our thread 
 */
#define CROSSWALK                       1
#define GO_AROUND_OBSTACLE_VIA_LEFT     2
#define GO_AROUND_OBSTACLE_VIA_RIGHT    3
#define OBSTACLE_ON_RIGHT               4
#define OBSTACLE_ON_LEFT                5
#define DEAD_END                        6 
#define INSTRUCTION_LEFT                7      
#define INSTRUCTION_RIGHT               8
#define NO_INSTRUCTION                  10

/*
 *  Defines used to identify each IR sensor of the epuck2
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
 *  Defines used for the proximity threshold of the different sensors 
 */
#define THRESHOLD_PROX_SIDE             70 
#define THRESHOLD_PROX_FRONT            190
#define THRESHOLD_PROX_FRONT_45         230 
#define THRESHOLD_PROX_REAR             80

/*
 *  Defines used for the durations and speeds of the movements of the robot
 */
#define TIME_QUARTER_TURN               550
#define TIME_HALF_TURN                  1100
#define TURNING_SPEED                   600
#define AHEAD_SPEED                     300
#define ZERO_SPEED                      0

/*
 *  Defines used for the security of the robot
 */
#define SAFELY_PASS_OBSTACLE            1000
#define TIME_PASSING_CROSSWALK          10000

/*
 *  Defines used for the leds 
 */
#define ON                              1
#define OFF                             0


// Variables set as static as used in different functions in this file
static systime_t time_crossed; 
static bool cross = false; 
static int8_t turning_direction = 0;


/*
 *  Function allowing the robot to move straight ahead.
 */
void straight_ahead(void){
    right_motor_set_speed(AHEAD_SPEED);
    left_motor_set_speed(AHEAD_SPEED);  
}

/*
 *  Function stoping the robot. 
 */
void robot_stop(void){
    right_motor_set_speed(ZERO_SPEED);
    left_motor_set_speed(ZERO_SPEED);
}

/*
 *  Functions allowing the robot to do a quarter turn to the left or to the right and turning off the leds.
 */
void quarter_turn_right(void){

    right_motor_set_speed(-TURNING_SPEED); 
    left_motor_set_speed(TURNING_SPEED);
    chThdSleepMilliseconds(TIME_QUARTER_TURN);
    clear_leds(); 
    set_front_led(OFF); 
}
void quarter_turn_left(void){

    right_motor_set_speed(TURNING_SPEED);
    left_motor_set_speed(-TURNING_SPEED);
    chThdSleepMilliseconds(TIME_QUARTER_TURN);
    clear_leds(); 
    set_front_led(OFF);
}

/*
 *  Function allowing the robot to do a half turn and turning off the leds.
 */
void half_turn(void){
    right_motor_set_speed(-TURNING_SPEED);
    left_motor_set_speed(TURNING_SPEED);
    chThdSleepMilliseconds(TIME_HALF_TURN);
    clear_leds(); 
    set_front_led(OFF);
}

/*
 *  Functions used following receiving an order to turn to the left or to the right. 
 *  They check that the space is clear before turning. If an obstacle is preventing the robot from turning, it waits to have passed it. 
 */
void check_before_turning_left(void){
    // while an obstacle is still preventing the robot to turn, it keeps going ahead
    while(get_prox(LEFT_SENSOR) > THRESHOLD_PROX_SIDE && get_prox(FRONT_RIGHT_SENSOR) < THRESHOLD_PROX_FRONT && get_prox(FRONT_LEFT_SENSOR) < THRESHOLD_PROX_FRONT){ 
        straight_ahead(); 
        set_led(LED7, ON);       
     }

    if ((get_prox(FRONT_RIGHT_SENSOR) > THRESHOLD_PROX_FRONT) || (get_prox(FRONT_LEFT_SENSOR) > THRESHOLD_PROX_FRONT)){
    } //if a new obstacle comes in front of it, it goes back to the infinite loop to know how to avoid it. 
    else {
        // once the way is clear, it waits for security then turns                
        chThdSleepMilliseconds(SAFELY_PASS_OBSTACLE); 
        clear_leds();
        quarter_turn_left(); 
    }

}
void check_before_turning_right(void){ // same function but for turning to the right
    while(get_prox(RIGHT_SENSOR) > THRESHOLD_PROX_SIDE && get_prox(FRONT_RIGHT_SENSOR) < THRESHOLD_PROX_FRONT && get_prox(FRONT_LEFT_SENSOR) < THRESHOLD_PROX_FRONT){ 
    straight_ahead();
    set_led(LED3, ON);                   
    }

    if (get_prox(FRONT_RIGHT_SENSOR) > THRESHOLD_PROX_FRONT  || get_prox(FRONT_LEFT_SENSOR) > THRESHOLD_PROX_FRONT){} 
    else {
        chThdSleepMilliseconds(SAFELY_PASS_OBSTACLE);
        clear_leds();
        quarter_turn_right(); 
    }
}

/*
 *  Function called when the robot wants to turn around an object to the left or to the right depending on the order.
 *  It works in several steps, the first quarter turn, then move along the obstacle until it has passed it. Then turns again, pass the obstacle on its side. 
 *  Turns a third time, moves along the obstacle on its other side for the same duration as on the first side of the obstacle.
 *  Then, turns a last time to realign with the path the robot was initally following. 
 */
void turn_around(bool turn_left, unsigned int side_sensor){ 

    systime_t time, time_start, time_stop, time_test; 
    uint16_t diff_time;  

    if(turn_left){ //makes the first quarter turn depending on the direction 
        quarter_turn_left();
    }
    else {
        quarter_turn_right();
    }
    set_front_led(OFF); //turns off the led once the obstacle isn't in front of the robot anymore

    time_start = chVTGetSystemTime(); 

    // until the sensors aren't clear, the robot goes along the obstacle
    while(get_prox(side_sensor) > THRESHOLD_PROX_SIDE && get_prox(FRONT_RIGHT_SENSOR) < THRESHOLD_PROX_FRONT && get_prox(FRONT_LEFT_SENSOR) < THRESHOLD_PROX_FRONT && !crosswalk_detected()){
        straight_ahead();
    }

    if (get_prox(FRONT_RIGHT_SENSOR) > THRESHOLD_PROX_FRONT  || get_prox(FRONT_LEFT_SENSOR) > THRESHOLD_PROX_FRONT || crosswalk_detected()){
    } //security check : if a new obstacle comes in front of it or if it detected a crosswalk, it goes back to the infinite loop to know how to avoid it 
    else {                  
        straight_ahead(); 
        chThdSleepMilliseconds(SAFELY_PASS_OBSTACLE); 
        time_stop = chVTGetSystemTime();       
        diff_time = time_stop - time_start; //measures the time the robot took to go along the obstacle
                        
        if(turn_left){ // second quarter turn 
            quarter_turn_right();
        }
        else {
            quarter_turn_left();
        }

        straight_ahead();
        chThdSleepMilliseconds(SAFELY_PASS_OBSTACLE);  

        // robot goes along the second side of the obstacle, same as before with the sensors  
        while(get_prox(side_sensor) > THRESHOLD_PROX_SIDE && get_prox(FRONT_RIGHT_SENSOR) < THRESHOLD_PROX_FRONT && get_prox(FRONT_LEFT_SENSOR) < THRESHOLD_PROX_FRONT && !crosswalk_detected()){ 
            straight_ahead();                    
        }

        if (get_prox(FRONT_RIGHT_SENSOR) > THRESHOLD_PROX_FRONT  || get_prox(FRONT_LEFT_SENSOR) > THRESHOLD_PROX_FRONT || crosswalk_detected()){                             
        } //security check 
        else {
            straight_ahead(); 
            chThdSleepMilliseconds(SAFELY_PASS_OBSTACLE); 

            if(turn_left){ //third quarter turn 
                quarter_turn_right();
            }
            else {
                quarter_turn_left();    
            }

            time = chVTGetSystemTime();
            time_test = chVTGetSystemTime(); 
                            
            // we move for the same amount of time
            while (time_test < (time + diff_time) && get_prox(FRONT_RIGHT_SENSOR) < THRESHOLD_PROX_FRONT && get_prox(FRONT_LEFT_SENSOR) < THRESHOLD_PROX_FRONT && !crosswalk_detected()){  
                straight_ahead();
                time_test = chVTGetSystemTime(); // we check time_test at each iteration
            }
            if (get_prox(FRONT_RIGHT_SENSOR) > THRESHOLD_PROX_FRONT  || get_prox(FRONT_LEFT_SENSOR) > THRESHOLD_PROX_FRONT || crosswalk_detected()){                
            } // security check 
            else{

                if(turn_left){ // final quarter turn, back on our original path
                    quarter_turn_left(); 
                }
                else{
                    quarter_turn_right();    
                }
                straight_ahead();
            }
        } 
    } 
}
/*
 *  Functions calling the function turn_around depending if the order is to go on the left or on the right. 
 */
void left_turn_around(void){
    turn_around(true, RIGHT_SENSOR); 
}
void right_turn_around(void){
    turn_around(false, LEFT_SENSOR);  
}


/*
 *  Function allowing our robot to "bark" when stopping at a cross walk. 
 */
void bark(void){
    playNote(NOTE_CS4,400);
    playNote(NOTE_G4,200);
}

/*
 *  Function allowing our robot to wait for the order to move forward once reaching a cross walk and then crossing it.  
 */
void crosswalk (void){

    robot_stop();
    bark(); // signals the presence of a crosswalk to its master 
    while (get_freq() != MOVE_FORWARD) // wait for the order to move forward 
    {}
    time_crossed = chVTGetSystemTime() + MS2ST(TIME_PASSING_CROSSWALK); 
    cross = true; // cross is set at true, the robot will not respond to orders for the duration of the crossing. 
    straight_ahead(); 
}


/*
 *  Function deciding in which case our robot is in, then sending it to the switch in our main thread to do the required actions. 
 *  It will first check for a crosswalk, as it is the most dangerous obstacle it can encounters. 
 *  Then, if none is ahead, the robot will use its different sensors to detect the presence of obstacles in front of it and on its sides, and then move accordingly. 
 *  Finally, if the road is clear and if there are no crosswalk, the robot will look for orders to turn, given by its master. 
 */
uint8_t get_mode (void){
    
    if (crosswalk_detected()){ 
        if(cross == false){ // check if it is not already crossing a crosswalk
            return CROSSWALK; 
        }
    }
    else if (get_prox(FRONT_RIGHT_SENSOR) > THRESHOLD_PROX_FRONT || get_prox(FRONT_RIGHT_45_SENSOR) > THRESHOLD_PROX_FRONT_45 || get_prox(FRONT_LEFT_45_SENSOR) > THRESHOLD_PROX_FRONT_45 || get_prox(FRONT_LEFT_SENSOR) > THRESHOLD_PROX_FRONT) 
    { //something is in front of the robot
    
        if(get_prox(RIGHT_SENSOR) <= THRESHOLD_PROX_SIDE && get_prox(LEFT_SENSOR) <= THRESHOLD_PROX_SIDE ) // there is nothing on the sides
        {
            set_front_led(ON);
            turning_direction = get_freq(); // gets the frequency of the order to know by which side to go around 

            if(turning_direction == TURN_LEFT){ 
                return GO_AROUND_OBSTACLE_VIA_LEFT; 
            }
            else if (turning_direction == TURN_RIGHT){ 
                return GO_AROUND_OBSTACLE_VIA_RIGHT; 
            }
            else { // if there is no known frequency, the robot waits for its order
                while (turning_direction != TURN_LEFT && turning_direction != TURN_RIGHT){ 
                    robot_stop();
                    turning_direction = get_freq(); 
                }
            }   
        }
        // obstacles in front and on the right 
        else if (get_prox(RIGHT_SENSOR) > THRESHOLD_PROX_SIDE && get_prox(LEFT_SENSOR) <= THRESHOLD_PROX_SIDE){
            set_front_led(ON); // lights the corresponding leds to signal it to other users 
            set_led(LED3, ON);
            return OBSTACLE_ON_RIGHT; 
        }
        // obstacles in front and on the left 
        else if (get_prox(LEFT_SENSOR) > THRESHOLD_PROX_SIDE && get_prox(RIGHT_SENSOR) <= THRESHOLD_PROX_SIDE){
            set_front_led(ON);
            set_led(LED7, ON);
            return OBSTACLE_ON_LEFT; 
        }
        // reaches a dead end 
        else {
            set_front_led(ON);        
            set_led(LED3, ON);
            set_led(LED7, ON);
            return DEAD_END; 
        }
    }
    else {
        if (turning_direction == get_freq() || cross == true){ // check if there was a chancge in frequency and that it is not currently crossing a crosswalk at the moment 
        }
        if (turning_direction != get_freq() && cross == false) { // case of a new order 
            turning_direction = get_freq(); 

            if(turning_direction == TURN_LEFT){ 
                return INSTRUCTION_LEFT; 
            } 
            if(turning_direction == TURN_RIGHT){ 
                return INSTRUCTION_RIGHT; 
            }
        } 
    }
    return NO_INSTRUCTION; // default case 
}

/*
 *  Our main thread ControlRobot, allowing our robot to react accordingly to what comes in its way, with the help of a switch.
 */
static THD_WORKING_AREA(waControlRobot, 512);
static THD_FUNCTION(ControlRobot, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time; 
    while(1){
        
        time = chVTGetSystemTime();
        
        straight_ahead(); // we set the robot to move forward

        if (chVTGetSystemTime() > time_crossed){ // if we are still on a crosswalk, the robot doesnt respond to orders 
            cross = false; 
        }
        
        switch(get_mode()){  //uses the function get_mode to know in which case the robot is 

            case CROSSWALK:
            crosswalk();
                break; 

            case GO_AROUND_OBSTACLE_VIA_LEFT: 
            left_turn_around();
                break; 

            case GO_AROUND_OBSTACLE_VIA_RIGHT: 
            right_turn_around();
                break; 

            case OBSTACLE_ON_RIGHT: 
            quarter_turn_left();
                break; 

            case OBSTACLE_ON_LEFT:
            quarter_turn_right(); 
                break; 

            case DEAD_END:
            half_turn(); 
                break; 

            case INSTRUCTION_LEFT:
            check_before_turning_left(); 
                break; 

            case INSTRUCTION_RIGHT:
            check_before_turning_right(); 
                break; 

            case NO_INSTRUCTION:
                break; 
        }
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}
    
/*
 *  Starts our thread ControlRobot in the main.c 
 */
void control_robot_start(void){
	chThdCreateStatic(waControlRobot, sizeof(waControlRobot), NORMALPRIO, ControlRobot, NULL);
}
