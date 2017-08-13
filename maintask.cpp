/*
 * maintask.c
 *
 *  Created on: 2017-8-9
 *      Author: wangbo
 */

#include <stdio.h>
#include <sys/time.h>
#include <sys/select.h>
#include <time.h>

#include <pthread.h>
#include <semaphore.h>

#include "maintask.h"

/*extern variable*/
struct T_MAIN_TASK main_task;

sem_t sem_loopfast;
sem_t sem_loopslow;

void maintask()
{

}

void init_maintask(void)
{
	main_task.loopfast_permission=1;
	main_task.loopslow_permission=1;
}

void stop_alltask(void)
{
	main_task.loopfast_permission=0;
	main_task.loopslow_permission=0;
}

void * loopfast(void *)
{
	printf("Enter the loopfast...\n");

	while(main_task.loopfast_permission)
	{
		sem_wait(&sem_loopfast);     /*等待信号量*/

		main_task.loopfast_cnt++;

		if(main_task.loopfast_cnt>=MAX_LOOPFAST_TICK)
		{
			main_task.loopfast_cnt=0;
		}
	}

}

void *loopslow(void*)
{
	printf("Enter the loopslow...\n");

	while(main_task.loopslow_permission)
	{
		sem_wait(&sem_loopslow);     /*等待信号量*/

		main_task.loopslow_cnt++;

		if(main_task.loopslow_cnt>=MAX_LOOPSLOW_TICK)
		{
			main_task.loopslow_cnt=0;
		}
	}
}
