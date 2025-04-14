/**
 * Application interface for RobotCANControl main.
 *
 * /file        application.h
 * /author      William Campbell
 * /date 2020-04-09
 * /copyright Copyright (c) 2020
 *
 * This file is an adaptation of CANopenSocket, a Linux implementation of CANopen
 * stack with master functionality. Project home page is
 * <https://github.com/CANopenNode/CANopenSocket>. CANopenSocket is based
 * on CANopenNode: <https://github.com/CANopenNode/CANopenNode>.
 *
 * The adaptation is specifically designed for use with the RobotCANControl design stack and
 * a multi limbed robot. It has been tested using a Beagle Bone black and the Fourier Intelligence X2
 * exoskelton in a lab testing setting.It can be addapted for use with other CANopen enabled linux based robotic projects.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <errno.h>
#include <fcntl.h>
#include <linux/reboot.h>
#include <net/if.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/epoll.h>
#include <sys/reboot.h>
#include <sys/time.h>
#include <termios.h>
#include <unistd.h>

#include "CANopen.h"
extern "C" {
#include "CO_Linux_tasks.h"
#include "CO_time.h"
}
#include "CO_OD_storage.h"
#include "CO_command.h"

//Include custom state machine (defined in cmake)
#include STATE_MACHINE_INCLUDE

#include "logging.h"

#ifndef CO_APPLICATION_H
#define CO_APPLICATION_H

#define NSEC_PER_SEC (1000000000)      /* The number of nanoseconds per second. */
#define NSEC_PER_MSEC (1000000)        /* The number of nanoseconds per millisecond. */
#define TMR_TASK_INTERVAL_NS (1000000) /* Interval of taskTmr in nanoseconds */
#define TMR_TASK_OVERFLOW_US (5000)    /* Overflow detect limit for taskTmr in microseconds */
#define INCREMENT_1MS(var) (var++)     /* Increment 1ms variable in taskTmr */
#define NODEID (80)

const float controlLoopPeriodInms = 2.;     //!< Define the control loop period (in ms): the period of rt_control_thread loop (and so the app update rate). In [ms].
const float CANUpdateLoopPeriodInms = 1.0;  //!< Define the CAN PDO processing period. SYNCH messages (and so actual PDO update) is set to twice this period (twice slower). In [ms].
const float activeWaitTimeInms = 0.5;       //!< Define the active wait time (busy CPU) in the control thread to get more accurate timing. Typically between 10%-50% of controlLoopPeriod, 0 for no effect. In [ms].



/**
 * /brief Function is called on program startup.
 */
void app_programStart();

/**
 * /brief Function is called before CO_init()
 */
void app_communicationReset(int argc = 0, char *argv[] = {});

/**
 * \brief Function is called just before program ends.
 */
void app_programEnd(void);

/**
 * \brief Function is called cyclically from main (low priority thread)
 *
 * \param timer1msDiff Time difference since last call
 */
void app_programAsync(uint16_t timer1msDiff);

/**
 * \brief Function is called cyclically from Control loop thread at constant intervals.
 *
 * Code inside this function must be executed fast. Take care on race conditions.
 */
void app_programControlLoop(void);


#endif /*APP_H*/
