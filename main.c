#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <main.h>

#include "audio/play_melody.h"
#include "audio/play_sound_file.h"
#include "epuck1x/a_d/advance_ad_scan/e_prox.h"
#include "epuck1x/motor_led/advance_one_timer/e_led.h"
#include "epuck1x/motor_led/advance_one_timer/e_motors.h"
#include "sensors/ground.h"
#include "leds.h"
#include "motors.h"
#include "selector.h"
#include "spi_comm.h"
#include "ircom/ircom.h"
#include "ircom/ircomReceive.h"
#include "ircom/ircomMessages.h"
#include "ircom/ircomSend.h"
#include "ircom/transceiver.h"

#define GROUND_THR 750
#define PROX_THR 90
#define BASE_SPEED 400
#define SAME_VALUE_THR 5

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

static THD_WORKING_AREA(demo_thd_wa, 2048);

void change_rgb(uint8_t val) {
	if(val == 0) { // Turn on red
		set_rgb_led(LED2, 100, 0, 0);
		set_rgb_led(LED4, 100, 0, 0);
		set_rgb_led(LED6, 100, 0, 0);
		set_rgb_led(LED8, 100, 0, 0);
	} else if(val == 1) { // Turn on green
		set_rgb_led(LED2, 0, 100, 0);
		set_rgb_led(LED4, 0, 100, 0);
		set_rgb_led(LED6, 0, 100, 0);
		set_rgb_led(LED8, 0, 100, 0);
	} else if(val == 2) { // Turn on blue
		set_rgb_led(LED2, 0, 0, 100);
		set_rgb_led(LED4, 0, 0, 100);
		set_rgb_led(LED6, 0, 0, 100);
		set_rgb_led(LED8, 0, 0, 100);
	} else if(val == 3) { // Turn on magenta
		set_rgb_led(LED2, 100, 0, 100);
		set_rgb_led(LED4, 100, 0, 100);
		set_rgb_led(LED6, 100, 0, 100);
		set_rgb_led(LED8, 100, 0, 100);
	} else if(val == 4) { // Turn on yellow
		set_rgb_led(LED2, 100, 100, 0);
		set_rgb_led(LED4, 100, 100, 0);
		set_rgb_led(LED6, 100, 100, 0);
		set_rgb_led(LED8, 100, 100, 0);
	} else if(val == 5) { // Turn on cyan
		set_rgb_led(LED2, 0, 100, 100);
		set_rgb_led(LED4, 0, 100, 100);
		set_rgb_led(LED6, 0, 100, 100);
		set_rgb_led(LED8, 0, 100, 100);
	} else if(val == 6) { // Turn on white
		set_rgb_led(LED2, 100, 100, 100);
		set_rgb_led(LED4, 100, 100, 100);
		set_rgb_led(LED6, 100, 100, 100);
		set_rgb_led(LED8, 100, 100, 100);
	} else if(val == 7) { // Turn off
		set_rgb_led(LED2, 0, 0, 0);
		set_rgb_led(LED4, 0, 0, 0);
		set_rgb_led(LED6, 0, 0, 0);
		set_rgb_led(LED8, 0, 0, 0);
	} else { // Problem
		set_rgb_led(LED2, 100, 0, 0);
		set_rgb_led(LED4, 0, 100, 0);
		set_rgb_led(LED6, 0, 0, 100);
		set_rgb_led(LED8, 100, 100, 100);
	}
}

void clear_red_leds(void) {
	set_led(LED1, 0);
	set_led(LED3, 0);
	set_led(LED5, 0);
	set_led(LED7, 0);
}

static THD_FUNCTION(demo_thd, arg) {
    (void) arg;
    chRegSetThreadName(__FUNCTION__);

    uint8_t demoState = 0;
    systime_t time;
    int16_t groundValues[3] = {1000,1000,1000};
    int16_t sensorSum = 0;
    int16_t motorSteps = 0;
	IrcomMessage imsg;
	uint8_t txData = 0;
	uint16_t timeout = 0;
	uint8_t prev_value = 0;
	uint8_t same_value_count = 0;

	srand(get_ground_prox(0));

	if(get_selector() == 0) { // Transmitter
		txData = rand()%7; // Give random between 0 and 6
		change_rgb(txData); // Initial color
	}

    while(1) {
    	time = chVTGetSystemTime();

    	groundValues[0] = get_ground_prox(0);
    	groundValues[2] = get_ground_prox(2);

        switch(demoState) {

            case 0: // line following
            	clear_red_leds();
                set_led(LED1, 1);
                sensorSum = groundValues[0] - groundValues[2];  // left ground - right ground
                e_set_speed_left(BASE_SPEED+(sensorSum>>1));
                e_set_speed_right(BASE_SPEED-(sensorSum>>1));
                if(groundValues[0]>GROUND_THR && groundValues[1]>GROUND_THR && groundValues[2]>GROUND_THR) {
                    e_set_speed_left(0);
                    e_set_speed_right(0);
                    timeout = 0;
                    ircomEnableProximity();
                    demoState = 1;
                }
                break;

            case 1: // waiting at line end
            	clear_red_leds();
                set_led(LED3, 1);
                if((ircom_get_calibrated_prox(0) + ircom_get_calibrated_prox(7))>PROX_THR) { // Another robot is present on front of me
                	timeout = 0;
                	same_value_count = 0;
                    demoState = 2;
                    ircomDisableProximity();
                    ircomFlushMessages();
                }
                timeout++;
                if(get_selector() == 0) { // Transmitter
					if(timeout == 600) { // 12 seconds...wait more to de-sync the robots
						e_set_steps_left(0);
						e_set_speed_left(-350);
						e_set_speed_right(350);
						motorSteps = 0;
						demoState = 3;
					}
                } else { // Receiver
					if(timeout == 350) { // 5 seconds...wait less to de-sync the robots
						e_set_steps_left(0);
						e_set_speed_left(-350);
						e_set_speed_right(350);
						motorSteps = 0;
						demoState = 3;
						change_rgb(7); // Turn off when timeout reached
					}
                }
                break;

            case 2: // Exchange data
            	clear_red_leds();
                set_led(LED5, 1);
            	if(get_selector() == 0) { // Transmitter
            		ircomSend(txData);
            		while(ircomSendDone() == 0);
                	if(timeout == 150) { // 3 seconds
                        e_set_steps_left(0);
                        e_set_speed_left(-350);
                        e_set_speed_right(350);
                        motorSteps = 0;
            			demoState = 3;
        				txData = rand()%7; // Give random between 0 and 6
        				change_rgb(txData);
        				//ircomEnableProximity();
                	}
            	} else { // Receiver
            		ircomPopMessage(&imsg);
            		if(imsg.error == 0) {
            			if(prev_value == (int) imsg.value) {
            				same_value_count++;
            			} else {
            				same_value_count = 0;
            			}
            			if(same_value_count >= SAME_VALUE_THR) {
							change_rgb((int) imsg.value);
							e_set_steps_left(0);
							e_set_speed_left(-350);
							e_set_speed_right(350);
							motorSteps = 0;
							demoState = 3;
							//ircomEnableProximity();
							break;
            			}
            			prev_value = (int) imsg.value;
            		}
                	if(timeout == 150) { // 3 seconds
                        e_set_steps_left(0);
                        e_set_speed_left(-350);
                        e_set_speed_right(350);
                        motorSteps = 0;
            			demoState = 3;
            			change_rgb(7); // Turn off when timeout reached
            			//ircomEnableProximity();
                	}
            	}
            	timeout++;
            	break;

            case 3: // rotating 180 degrees
            	clear_red_leds();
                set_led(LED7, 1);
                motorSteps = e_get_steps_left();
                if(motorSteps <= -660) {    // from field test 660 steps correspond to about 180 degrees
                    e_set_speed_left(0);
                    e_set_speed_right(0);
                    demoState = 0;
                }
                break;

        }

        chThdSleepUntilWindowed(time, time + MS2ST(20)); // Control loop @ 50Hz

    }

}

int main(void) {

    halInit();
    chSysInit();
    mpu_init();

    /** Inits the Inter Process Communication bus. */
    messagebus_init(&bus, &bus_lock, &bus_condvar);

	clear_leds();
	set_front_led(0);
	if(get_selector()==0) { // Transmitter
		set_body_led(1);
	} else {
		set_body_led(0);
	}
	motors_init();
	spi_image_transfer_disable();
	spi_comm_start();
	playMelodyStart();
	playSoundFileStart();
    ground_start();
    ircomStart();
    ircomEnableContinuousListening();
    ircomListen();
    ircomEnableProximity();

    chThdCreateStatic(demo_thd_wa, sizeof(demo_thd_wa), NORMALPRIO, demo_thd, NULL);

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
