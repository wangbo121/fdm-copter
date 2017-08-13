/*
 * main.cpp
 *
 *  Created on: 2017-8-12
 *      Author: wangbo
 */

#include <iostream>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>//创建文件
#include <pthread.h>
#include <semaphore.h>
#include <sys/stat.h>
#include <string.h>
/*转换int或者short的字节顺序，该程序arm平台为大端模式，地面站x86架构为小端模式*/
#include <byteswap.h>

#include<sys/socket.h>
#include<netinet/in.h>
#include<arpa/inet.h>
#include <string.h>

#include "maintask.h"
#include "udp.h"
#include "global.h"

//#include "BIT_MATH.h"
//#include "utility.h"
//
//#include "aircraft.h"
//#include "quadcopter.h"
#include "fdm.h"

#include "SIM_Multicopter.h"
#include "SITL.h"

//MultiCopter multi_copter("120,48,100,10","x");
MultiCopter multi_copter("-122.357,37.6136,100,10","x");
//MultiCopter multi_copter("-122.357,37.6136,100,10","+");


//MultiCopter multi_copter;
T_FDM fdm;





#define TEST
#ifdef TEST
char udp_string[]="0123456";
#endif

T_GLOBAL  gblState;
T_AP2FG  ap2fg;
T_FG2AP fg2ap;
T_AP2FG  ap2fg_send;


T_AP2FG  ap2fg_recv;



#define D2R (3.14159 / 180.0)

double latitude;
double longitude;
double altitude;

#define MOTOR_NUM 4


//uint16_t aileron=1500;
//uint16_t elevator=1500;
//uint16_t throttle=1500;
//uint16_t rudder=1500;
//float aileron=250;
float aileron=-250;
float elevator=0;
float throttle=1500;
float rudder=0;


static double delta_0=0.1;
static double delta_1=0.1;
static double delta_2=0.1;
static double delta_3=0.1;



static int flag=1;

int main()
{
	/*
	* 这一部分写程序的初始化
	*/

	std::cout<<"multi_copter.home.lng="<<multi_copter.home.lng*1e-7<<std::endl;
	std::cout<<"multi_copter.home.lat="<<multi_copter.home.lat*1e-7<<std::endl;
	std::cout<<"multi_copter.home.alt="<<multi_copter.home.alt*1e-2<<std::endl;

	//uint16_t servos_set[16];
	float servos_set[16];
	servos_set[0]=1500;
	servos_set[1]=1500;
	servos_set[2]=1500;
	servos_set[3]=1500;

	/*
	 * //这个作为遥控器的输入信号，或者说遥控器进来后经过出去又要out给servos
	 * 虽然四旋翼最终控制的是motor，不是固定翼的servos，但是这个motor也是用servo的pwm波信号
	 * 包括其他的舵机都是servo的1000-2000pwm波信号，所以我们统一用servo作为最终电机的
	 * 有servo_in 和servo_out 其中servo_in就是等于channel_out的，然后进来后可能还要限幅什么的所以最终输出给电机的叫做
	 * servo_out
	 */
	/*
	 * channel_out[0]:aileron
	 * channel_out[1]:elevator
	 * channel_out[2]:throttle
	 * channel_out[3]:rudder
	 */
	float channel_out[16];

	uint16_t servos_set_out[4];

	Aircraft::sitl_input input;

	memcpy(input.servos,servos_set,sizeof(servos_set));

	//printf("sizeof(float)=%d\n",sizeof(float));//已测试 是4个字节 20170813
	//printf("sizeof(fdm)=%d\n",sizeof(fdm));//已测试 是408个字节 20170813

	//open_udp_dev(IP_SEND_TO, 49000, PORT_RECEIVE);
	open_udp_dev(IP_SEND_TO, PORT_SENT_TO, PORT_RECEIVE);
	//open_udp_dev(IP_SEND_TO, PORT_SENT_TO_GENERIC, PORT_RECEIVE);


	ap2fg.throttle0 = 0.2;
	ap2fg.throttle1 = 0.3;
	ap2fg.throttle2 = 0.4;
	ap2fg.throttle3 = 0.5;

	/***************************/
	/***到此初始化部分结束下面一直到while循环是创建快慢线程，中间不加任何东西，不改变******************/

	printf("Enter the maintask...\n");
	init_maintask();

	pthread_t loopfast_pthrd = 0;
	pthread_t loopslow_pthrd = 0;
	static int sem_loopfast_cnt;
	static int sem_loopslow_cnt;
	int ret=0;

	/*
	* 初始化快循环信号量
	*/
	sem_init(&sem_loopfast,0,1);/*初始化时，信号量为1*/
	ret = pthread_create (&loopfast_pthrd,          //线程标识符指针
												NULL,                     //默认属性
												loopfast,         //运行函数
												NULL);                    //无参数
	if (0 != ret)
	{
		perror ("pthread create error\n");
	}

	/*
	* 初始化慢循环信号量
	*/
	sem_init(&sem_loopslow,0,1);
	ret = pthread_create (&loopslow_pthrd,          //线程标识符指针
												NULL,                     //默认属性
												loopslow,         //运行函数
												NULL);                    //无参数
	if (0 != ret)
	{
		perror ("pthread create error\n");
	}

	int seconds=0;
	int mseconds=MAINTASK_TICK_TIME_MS*(1e3);/*每个tick为20毫秒，也就是20000微秒*/
	struct timeval maintask_tick;

	/*
	* 开始maintask任务，maintask任务按最小的tick执行，周期时间为20ms，执行一次
	*/
	while (1)
	{
		maintask_tick.tv_sec = seconds;
		maintask_tick.tv_usec = mseconds;
		select(0, NULL, NULL, NULL, &maintask_tick);

		main_task.maintask_cnt++;//20ms这个计数加1

		/*loopfast 快循环*/
		if(0 == main_task.maintask_cnt%LOOP_FAST_TICK)
		{
			sem_getvalue(&sem_loopfast,&sem_loopfast_cnt);
			if(sem_loopfast_cnt<1)
			{
				sem_post (&sem_loopfast);/*释放信号量*/
			}
		}

		/*loopslow 慢循环*/
		if(0 == main_task.maintask_cnt%LOOP_SLOW_TICK)
		{
			sem_getvalue(&sem_loopslow,&sem_loopslow_cnt);
			if(sem_loopslow_cnt<1)
			{
				sem_post (&sem_loopslow);        /*释放信号量*/
			}

			//打印当前系统运行时间
			//float system_running_time=0.0;
			//system_running_time=clock_gettime_s();
			//printf("系统从开启到当前时刻运行的时间%f[s]\n",system_running_time);

			/*
			* 可以直接把程序写在这里也可以写在loopslow函数里
			*/
			if(ap2fg.throttle0>=1||ap2fg.throttle0<=0.1)
			{
				delta_0=-delta_0;
			}
			if(ap2fg.throttle1>=1||ap2fg.throttle1<=0.1)
			{
				delta_1=-delta_1;
			}
			if(ap2fg.throttle2>=1||ap2fg.throttle2<=0.1)
			{
				delta_2=-delta_2;
			}
			if(ap2fg.throttle3>=1||ap2fg.throttle3<=0.1)
			{
				delta_3=-delta_3;
			}

			/*
			 * 切记通过generic发送给flightgear的范围
			 * 是在-1到+1之间
			 */
//			ap2fg.throttle0 = ap2fg.throttle0 +delta_0;
//			ap2fg.throttle1 = ap2fg.throttle1 +delta_1;
//			ap2fg.throttle2 = ap2fg.throttle2 +delta_2;
//			ap2fg.throttle3 = ap2fg.throttle3 +delta_3;
			ap2fg.throttle0 = (float)(servos_set_out[0])/1000.0;
			ap2fg.throttle1 = (float)(servos_set_out[1])/1000.0;
			ap2fg.throttle2 = (float)(servos_set_out[2])/1000.0;
			ap2fg.throttle3 = (float)(servos_set_out[3])/1000.0;

#if 0
			ap2fg.latitude_deg = 40;
			ap2fg.longitude_deg = 117;
			ap2fg.altitude_ft = 30;
			ap2fg.altitude_agl_ft = 100;
			ap2fg.roll_deg = 2.0;
			ap2fg.pitch_deg = 0.0;
			ap2fg.heading_deg = 38.0;
#endif

			memcpy(&ap2fg_send,&ap2fg,sizeof(ap2fg));
			ap2fg_send.throttle0=hton_double(ap2fg_send.throttle0);
			ap2fg_send.throttle1=hton_double(ap2fg_send.throttle1);
			ap2fg_send.throttle2=hton_double(ap2fg_send.throttle2);
			ap2fg_send.throttle3=hton_double(ap2fg_send.throttle3);
#if 0
			ap2fg_send.latitude_deg=hton_double(ap2fg_send.latitude_deg);
			ap2fg_send.longitude_deg=hton_double(ap2fg_send.longitude_deg);
			ap2fg_send.altitude_ft=hton_double(ap2fg_send.altitude_ft);
			ap2fg_send.altitude_agl_ft=hton_double(ap2fg_send.altitude_agl_ft);
			ap2fg_send.roll_deg=hton_double(ap2fg_send.roll_deg);
			ap2fg_send.pitch_deg=hton_double(ap2fg_send.pitch_deg);
			ap2fg_send.heading_deg=hton_double(ap2fg_send.heading_deg);
#endif

				float               _roll_factor[4]; // each motors contribution to roll
				float               _pitch_factor[4]; // each motors contribution to pitch
				float               _throttle_factor[4];
				float               _yaw_factor[4];  // each motors contribution to yaw (normally 1 or -1)

				/*
				* 这里给factor赋值-1 0 或者1
				*/
				_roll_factor[0]  =  -1;  _pitch_factor[0]  =  +1; _throttle_factor[0]=+1; _yaw_factor[0]  = +1;
				_roll_factor[1]  =   +1;  _pitch_factor[1]  =  -1; _throttle_factor[1]=+1; _yaw_factor[1]  =  +1;
				_roll_factor[2]  = +1;  _pitch_factor[2]  =  +1;  _throttle_factor[2]=+1;_yaw_factor[2]  = -1;
				_roll_factor[3]  =  -1;  _pitch_factor[3]  = -1;  _throttle_factor[3]=+1;_yaw_factor[3]  =  -1;

				float aileron_half;
				float elevator_half;

				aileron_half=aileron*0.5;
				elevator_half=elevator*0.5;

				for(int i=0;i<4;i++)
				{
					/*
					* 一定要注意这里的throttle是用的radio_out，radio_out=pwm_out+radio_trim，
					* calc是把servo_out的-4500～+4500转为pwm的-500～+500
					* radio_out=pwm_out+radio_trim=pwm_out+1500 radio_out的范围是1000-2000
					*/
					servos_set[i]=throttle           * _throttle_factor[i]+ \
											aileron_half   * _roll_factor[i]+\
											elevator_half * _pitch_factor[i]+\
											rudder            * _yaw_factor[i];
				}

				std::cout<<"servos_set[0]="<<servos_set[0]<<std::endl;
				std::cout<<"servos_set[1]="<<servos_set[1]<<std::endl;
				std::cout<<"servos_set[2]="<<servos_set[2]<<std::endl;
				std::cout<<"servos_set[3]="<<servos_set[3]<<std::endl;

				for(int i=0;i<4;i++)
				{
					if(servos_set[i]<0)
					{
						servos_set[i]=100;
					}
				}

				for(int i=0;i<4;i++)
				{
					servos_set_out[i]=(uint16_t)servos_set[i];
				}
				std::cout<<"servos_set_out[0]="<<servos_set_out[0]<<std::endl;
				std::cout<<"servos_set_out[1]="<<servos_set_out[1]<<std::endl;
				std::cout<<"servos_set_out[2]="<<servos_set_out[2]<<std::endl;
				std::cout<<"servos_set_out[3]="<<servos_set_out[3]<<std::endl;

				/*
				 * 注意这里的input.servos是uint_16类型的，无符号short型，
				 * 所以servos_set_out一定不要有负数，所以需要提前处理servos_set_out
				 */
				memcpy(input.servos,servos_set_out,sizeof(servos_set));

				multi_copter.update(input);

				//sitl_fdm fdm_sitl;
				//multi_copter.fill_fdm(fdm_sitl);//这个是apm自身带的软件仿真，以后删除掉不需要

				multi_copter.fill_fdm_flightgear(fdm);

				fdm.version = htonl(FG_NET_FDM_VERSION);
				fdm.latitude = htond(fdm.latitude);
				fdm.longitude = htond(fdm.longitude);
				fdm.altitude = htond(fdm.altitude);
				fdm.phi = htonf(fdm.phi );
				fdm.theta = htonf(fdm.theta);
				fdm.psi = htonf(fdm.psi);
				fdm.num_engines = htonl(1);
				fdm.num_tanks = htonl(1);
				fdm.fuel_quantity[0] = htonf(100.0);
				fdm.num_wheels = htonl(3);
				fdm.cur_time = htonl(time(0));
				fdm.warp = htonl(1);
				fdm.visibility = htonf(5000.0);

				sendto(fd_sock_send, &fdm, sizeof(fdm), 0, (struct sockaddr *)&udp_sendto_addr, sizeof(udp_sendto_addr));

				//sendto(fd_sock_send, &ap2fg_send, sizeof(ap2fg_send), 0, (struct sockaddr *)&udp_sendto_addr, sizeof(udp_sendto_addr));
				//send_udp_data((unsigned char*)&ap2fg_send,sizeof(ap2fg_send));

				//send_udp_data((unsigned char*)&fdm,sizeof(fdm));
				//sendto(fd_sock_send, &fdm, sizeof(fdm), 0, (struct sockaddr *)&udp_sendto_addr, sizeof(udp_sendto_addr));

		}

		if(main_task.maintask_cnt>=MAX_MAINTASK_TICK)
		{
			main_task.maintask_cnt=0;
		}
	}


	return 0;
}

