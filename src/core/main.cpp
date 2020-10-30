/*
 *
 * /file        main.cpp
 * /author      William Campbell
 * /version 0.1
 * /date 2020-04-09
 *
 * /copyright Copyright (c) 2020
 *
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
#include "application.h"
/* Threads and thread safety variables***********************************************************/
/**
 * Mutex is locked, when CAN is not valid (configuration state).
 * May be used from other threads. RT threads may use CO->CANmodule[0]->CANnormal instead.
*/
pthread_mutex_t CO_CAN_VALID_mtx = PTHREAD_MUTEX_INITIALIZER;

static int mainline_epoll_fd; /*!< epoll file descriptor for mainline */
static CO_time_t CO_time;     /*!< Object for current time */
bool readyToStart = false;    /*!< Flag used by control thread to indicate CAN stack functional */
uint32_t tmr1msPrev = 0;

/*CAN msg processing thread variables*/
static int rtPriority = 90;             /*!< priority of rt CANmsg thread */
static void *rt_thread(void *arg);
static pthread_t rt_thread_id;
static int rt_thread_epoll_fd;          /*!< epoll file descriptor for rt thread */
/* Application Control loop thread */
static int rtControlPriority = 80;      /*!< priority of application thread */
static void *rt_control_thread(void *arg);
static pthread_t rt_control_thread_id;
const float controlLoopPeriodInms = 3; /*!< Define the control loop period (in ms): the period of rt_control_thread loop. */
const float CANUpdateLoopPeriodInms = 3; /*!< Define the CAN PDO sync message period (and so PDO update rate). In ms. Less than 3 can lead to unstable communication  */

/** @brief Task Timer used for the Control Loop*/
struct period_info {
    struct timespec next_period;
    long period_ns;
};

/** @brief Struct to hold arguments for ROS thread*/
struct ros_arg_holder {
    int argc;
    char ** argv;
};

/* Forward declartion of control loop thread timer functions*/
static void inc_period(struct period_info *pinfo);
static void periodic_task_init(struct period_info *pinfo);
static void wait_rest_of_period(struct period_info *pinfo);
/* Forward declartion of CAN helper functions*/
void configureCANopen(int nodeId, int rtPriority, int CANdevice0Index, char *CANdevice);
void CO_errExit(char const *msg);                   /*!< CAN object error code and exit program*/
void CO_error(const uint32_t info);                 /*!< send CANopen generic emergency message */
volatile uint32_t CO_timer1ms = 0U;                 /*!< Global variable increments each millisecond */
volatile sig_atomic_t CO_endProgram = 0;            /*!< Signal handler: controls the end of CAN processing thread*/
volatile sig_atomic_t endProgram = 0;               /*!< Signal handler: controls the end of application side (rt thread and main)*/
static void sigHandler(int sig) {
    endProgram = 1;
}

/******************************************************************************/
/** Mainline and threads                                                     **/
/******************************************************************************/
int main(int argc, char *argv[]) {
    //Initialise console and file logging. Name file can be specified if required (see logging.h)
    init_logging();

    //Check if running with root privilege
    if(getuid() != 0) {
        //Fallback to standard non RT thread
        rtPriority = -1;
        rtControlPriority = -1;
        spdlog::warn("Running without root privilege: using non-RT priority threads");
    }
    else {
        spdlog::info("Running with root privilege: using RT priority threads");
    }

    /* TODO : MOVE bellow definitionsTO SOME KIND OF CANobject, struct or the like*/
    CO_NMT_reset_cmd_t reset = CO_RESET_NOT;
    bool_t firstRun = true;
    bool_t rebootEnable = false; /*!< Configurable by use case */  // TODO: DO WE EVER RESET? OR NEED TO?
    int nodeId = NODEID; /*!< CAN Network NODEID */

    int can_dev_number=6;
    char CANdeviceList[can_dev_number][10] = {"vcan0\0", "can0\0", "can1\0", "can2\0", "can3\0", "can4\0"};    /*!< linux CAN device interface for app to bind to: change to can1 for bbb, can0 for BBAI vcan0 for virtual can*/
    char CANdevice[10]="";
    int CANdevice0Index;
    //Rotate through list of interfaces and select first one existing and up
    for(int i=0; i<can_dev_number; i++) {
        //Check if interface exists
        CANdevice0Index = if_nametoindex(CANdeviceList[i]);/*map linux CAN interface to corresponding int index return zero if no interface exists.*/
        if(CANdevice0Index!=0) {
            char operstate_filename[255], operstate_s[25];
            snprintf(operstate_filename, 254, "/sys/class/net/%s/operstate", CANdeviceList[i]);
            //Check if it's up
            FILE* operstate_f = fopen(operstate_filename, "r");
            if(fscanf(operstate_f, "%s", &operstate_s)>0)
            {
                spdlog::info("{}: {}", CANdeviceList[i], operstate_s);
                //Check if not "down" as will be "unknown" if up
                if(strcmp(operstate_s, "down")!=0) {
                    snprintf(CANdevice, 9, "%s", CANdeviceList[i]);
                    spdlog::info("Using: {} ({})", CANdeviceList[i], CANdevice0Index);
                    break;
                } else {
                    CANdevice0Index=0;
                }
            } else {
                CANdevice0Index=0;
            }
        } else {
            spdlog::info("{}: -", CANdeviceList[i]);
        }

    }
    configureCANopen(nodeId, rtPriority, CANdevice0Index, CANdevice);

    struct ros_arg_holder * ros_args = (ros_arg_holder*)malloc(sizeof(*ros_args));
    ros_args->argc = argc;
    ros_args->argv = argv;

    /* Set up catch of linux signals SIGINT(ctrl+c) and SIGTERM (terminate program - shell kill command)
        bind to sigHandler -> raise CO_endProgram flag and safely close application threads*/
    if (signal(SIGINT, sigHandler) == SIG_ERR)
        CO_errExit("Program init - SIGINIT handler creation failed");
    if (signal(SIGTERM, sigHandler) == SIG_ERR)
        CO_errExit("Program init - SIGTERM handler creation failed");
    spdlog::info("Starting CANopen device with Node ID {}", nodeId);

    //Set synch signal period (in us)
    CO_OD_RAM.communicationCyclePeriod=CANUpdateLoopPeriodInms*1000;

    while (reset != CO_RESET_APP && reset != CO_RESET_QUIT && endProgram == 0) {
        /* CANopen communication reset || first run of app- initialize CANopen objects *******************/
        CO_ReturnError_t err;
        /*mutex locking for thread safe OD access*/
        pthread_mutex_lock(&CO_CAN_VALID_mtx);
        /* Wait rt_thread. */
        if (!firstRun) {
            CO_LOCK_OD();
            CO->CANmodule[0]->CANnormal = false;
            CO_UNLOCK_OD();
        }
        /* initialize CANopen with CAN interface and nodeID */
        if (CO_init(CANdevice0Index, nodeId, 0) != CO_ERROR_NO) {
            char s[120];
            snprintf(s, 120, "Communication reset - CANopen initialization failed, err=%d", err);
            CO_errExit(s);
        }
        /* Configure callback functions for task control */
        CO_EM_initCallback(CO->em, taskMain_cbSignal);
        CO_SDO_initCallback(CO->SDO[0], taskMain_cbSignal);
        CO_SDOclient_initCallback(CO->SDOclient, taskMain_cbSignal);

        /* Initialize time */
        CO_time_init(&CO_time, CO->SDO[0], &OD_time.epochTimeBaseMs, &OD_time.epochTimeOffsetMs, 0x2130);

        /* First time only initialization */
        if (firstRun) {
            firstRun = false;
            /* Configure epoll for mainline */
            mainline_epoll_fd = epoll_create(4);
            if (mainline_epoll_fd == -1)
                CO_errExit("Program init - epoll_create mainline failed");

            /* Init mainline */
            taskMain_init(mainline_epoll_fd, &OD_performance[ODA_performance_mainCycleMaxTime]);
            /* Configure epoll for rt_thread */
            rt_thread_epoll_fd = epoll_create(2);
            if (rt_thread_epoll_fd == -1)
                CO_errExit("Program init - epoll_create rt_thread failed");
            /* Init taskRT */
            CANrx_taskTmr_init(rt_thread_epoll_fd, TMR_TASK_INTERVAL_NS, &OD_performance[ODA_performance_timerCycleMaxTime]);
            OD_performance[ODA_performance_timerCycleTime] = TMR_TASK_INTERVAL_NS / 1000; /* informative */

            /* Create rt_thread */
            if (pthread_create(&rt_thread_id, NULL, rt_thread, NULL) != 0)
                CO_errExit("Program init - rt_thread creation failed");
            /* Set priority for rt_thread */
            if (rtPriority > 0) {
                struct sched_param param;
                param.sched_priority = rtPriority;
                if (pthread_setschedparam(rt_thread_id, SCHED_FIFO, &param) != 0){
                    CO_errExit("Program init - rt_thread set scheduler failed (are you root?)");
                }
            }
            /* Create control_thread */
            if (pthread_create(&rt_control_thread_id, NULL, rt_control_thread, ros_args) != 0)
                CO_errExit("Program init - rt_thread_control creation failed");
            /* Set priority for control thread */
            if (rtPriority > 0) {
                struct sched_param paramc;
                paramc.sched_priority = rtControlPriority;
                if (pthread_setschedparam(rt_control_thread_id, SCHED_FIFO, &paramc) != 0){
                    CO_errExit("Program init - rt_thread set scheduler failed (are you root?)");
                }
            }
            /* start CAN */
            CO_CANsetNormalMode(CO->CANmodule[0]);
            pthread_mutex_unlock(&CO_CAN_VALID_mtx);
            reset = CO_RESET_NOT;
            /* Execute optional additional application code */
            app_communicationReset();
            readyToStart = true;
            while (reset == CO_RESET_NOT && endProgram == 0) {
                /* loop for normal program execution main epoll reading ******************************************/
                int ready;
                struct epoll_event ev;
                ready = epoll_wait(mainline_epoll_fd, &ev, 1, -1);
                if (ready != 1) {
                    if (errno != EINTR) {
                        CO_error(0x11100000L + errno);
                    }
                } else if (taskMain_process(ev.data.fd, &reset, CO_timer1ms)) {
                    uint32_t timer1msDiff;
                    timer1msDiff = CO_timer1ms - tmr1msPrev;
                    tmr1msPrev = CO_timer1ms;
                    /* Execute optional additional alication code */
                    app_programAsync(timer1msDiff);
                }

                else {
                    /* No file descriptor was processed. */
                    CO_error(0x11200000L);
                }
            }
        }
        /* program exit ***************************************************************/
        //End application first
        endProgram = 1;
        if (pthread_join(rt_control_thread_id, NULL) != 0) {
            CO_errExit("Program end - pthread_join failed");
        }
        usleep(500000); /*wait for last CAN commands to be processed if any */
        //End CAN communication processing
        CO_endProgram = 1;
        if (pthread_join(rt_thread_id, NULL) != 0) {
            CO_errExit("Program end - pthread_join failed");
        }
        /* delete objects from memory */
        CANrx_taskTmr_close();
        taskMain_close();
        CO_delete(CANdevice0Index);
        spdlog::info("Canopend on {} (nodeId={}) - finished.", CANdevice, nodeId);
        /* Flush all buffers (and reboot) */
        if (rebootEnable && reset == CO_RESET_APP) {
            sync();
            if (reboot(LINUX_REBOOT_CMD_RESTART) != 0) {
                CO_errExit("Program end - reboot failed");
            }
        }

        exit(EXIT_SUCCESS);
    }
}

/* Function for CAN send, receive and taskTmr ********************************/
static void *rt_thread(void *arg) {
    while (CO_endProgram == 0) {
        struct epoll_event ev;
        int ready = epoll_wait(rt_thread_epoll_fd, &ev, 1, -1);
        if (ready != 1) {
            if (errno != EINTR) {
                CO_error(0x12100000L + errno);
            }
        } else if (CANrx_taskTmr_process(ev.data.fd)) {
            /* code was processed in the above function. Additional code process below */
            INCREMENT_1MS(CO_timer1ms);
            /* Monitor variables with trace objects */
            CO_time_process(&CO_time);
#if CO_NO_TRACE > 0
            for (i = 0; i < OD_traceEnable && i < CO_NO_TRACE; i++) {
                CO_trace_process(CO->trace[i], *CO_time.epochTimeOffsetMs);
            }
#endif
            /* Detect timer large overflow */
            if (OD_performance[ODA_performance_timerCycleMaxTime] > TMR_TASK_OVERFLOW_US && rtPriority > 0 && CO->CANmodule[0]->CANnormal) {
                CO_errorReport(CO->em, CO_EM_ISR_TIMER_OVERFLOW, CO_EMC_SOFTWARE_INTERNAL, 0x22400000L | OD_performance[ODA_performance_timerCycleMaxTime]);
            }
        }

        else {
            /* No file descriptor was processed. */
            CO_error(0x12200000L);
        }
    }
    return NULL;
}
/* Control thread function ********************************/
static void *rt_control_thread(void *arg) {
    struct period_info pinfo;
    periodic_task_init(&pinfo);
    ros_arg_holder *ros_args = (ros_arg_holder*)arg;
    app_programStart(ros_args->argc, ros_args->argv);
    while (!readyToStart) {
        wait_rest_of_period(&pinfo);
    }
    while (endProgram == 0) {
        app_programControlLoop();
        wait_rest_of_period(&pinfo);
    }
    app_programEnd();
    return NULL;
}
/* Control thread time functions ********************************/
static void inc_period(struct period_info *pinfo) {
    pinfo->next_period.tv_nsec += pinfo->period_ns;

    while (pinfo->next_period.tv_nsec >= 1000000000) {
        /* timespec nsec overflow */
        pinfo->next_period.tv_sec++;
        pinfo->next_period.tv_nsec -= 1000000000;
    }
}
static void periodic_task_init(struct period_info *pinfo) {
    /* for simplicity, hardcoding a 1ms period */
    pinfo->period_ns = controlLoopPeriodInms*1000000;

    clock_gettime(CLOCK_MONOTONIC, &(pinfo->next_period));
}
static void wait_rest_of_period(struct period_info *pinfo) {
    inc_period(pinfo);

    /* for simplicity, ignoring possibilities of signal wakes */
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &pinfo->next_period, NULL);
}
/* CAN messaging helper functions ********************************/

void configureCANopen(int nodeId, int rtPriority, int CANdevice0Index, char *CANdevice) {
    if (nodeId < 1 || nodeId > 127) {
        fprintf(stderr, "NODE ID outside range (%d)\n", nodeId);
        exit(EXIT_FAILURE);
    }
    // rt Thread priority sanity check
    if (rtPriority != -1 && (rtPriority < sched_get_priority_min(SCHED_FIFO) || rtPriority > sched_get_priority_max(SCHED_FIFO))) {
        spdlog::critical("Wrong RT priority ({})", rtPriority);
        exit(EXIT_FAILURE);
    }

    if (CANdevice0Index == 0) {
        spdlog::critical("Can't find any CAN device");
        exit(EXIT_FAILURE);
    }

    /* Verify, if OD structures have proper alignment of initial values */
    if (CO_OD_RAM.FirstWord != CO_OD_RAM.LastWord) {
        spdlog::critical("Program init - Canopend- Error in CO_OD_RAM.");
        exit(EXIT_FAILURE);
    }
};
void CO_errExit(char const *msg) {
    spdlog::critical(msg);
    exit(EXIT_FAILURE);
}
void CO_error(const uint32_t info) {
    CO_errorReport(CO->em, CO_EM_GENERIC_SOFTWARE_ERROR, CO_EMC_SOFTWARE_INTERNAL, info);
    //fprintf(stderr, "canopend generic error: 0x%X\n", info);
    spdlog::error("canopend generic error: {}", info);
}
