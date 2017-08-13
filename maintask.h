/*
 * maintask.h
 *
 *  Created on: 2017-8-9
 *      Author: wangbo
 */

#ifndef MAINTASK_H_
#define MAINTASK_H_

#define MAX_MAINTASK_TICK 10000
#define MAX_LOOPFAST_TICK 10000
#define MAX_LOOPSLOW_TICK 10000

/*
 * maintask tick time [ms]
 */
#define MAINTASK_TICK_TIME_MS 20

/*
 * execute every LOOP_FAST_TICK*MAINTASK_TICK_TIME ms
 */
#define LOOP_FAST_TICK 15
#define LOOP_SLOW_TICK 50

struct T_MAIN_TASK
{
	unsigned char loopfast_permission;
	unsigned char loopslow_permission;

	unsigned int loopfast_cnt;
	unsigned int loopslow_cnt;

	unsigned int maintask_cnt;
};

extern struct T_MAIN_TASK main_task;
extern sem_t sem_loopfast;
extern sem_t sem_loopslow;

void maintask(void);
void init_maintask(void);
void stop_alltask(void);

void *loopfast(void*);
void *loopslow(void*);



#endif /* MAINTASK_H_ */
