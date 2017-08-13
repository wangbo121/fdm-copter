/*
 * global.h
 *
 *  Created on: 2017-8-9
 *      Author: wangbo
 */

#ifndef GLOBAL_H_
#define GLOBAL_H_

#define QUAD_MOTORS 4


typedef struct tagGLOBAL
{
	double cnt;

}T_GLOBAL;

typedef struct tagAP2FG
{
  double throttle0;//[0..1], 0-3为四个电机的控制量
  double throttle1;
  double throttle2;
  double throttle3;
#if 0
  double latitude_deg;//[deg],飞行器当前纬度坐标
  double longitude_deg;//[deg],飞行器当前经度坐标
  double altitude_ft;//[ft],飞行器当前飞行高度
  double altitude_agl_ft;//[ft],
  double roll_deg;//[deg]滚转角
  double pitch_deg;//[deg]俯仰角
  double heading_deg;//[deg]机头朝向
#endif
}T_AP2FG;

typedef struct tagFG2AP
{
  double latitude_deg;//机场的纬度坐标，只在初始化时使用一次
  double longitude_deg;//机场的经度坐标，只在初始化时使用一次
  double pitch_deg;
  double heading_deg;//机头初始朝向
}T_FG2AP;


extern T_GLOBAL  gblState;
extern T_AP2FG  ap2fg;
extern T_FG2AP fg2ap;
extern T_AP2FG  ap2fg_send;


extern T_AP2FG  ap2fg_recv;


extern double latitude;
extern double longitude;
extern double altitude;



#endif /* GLOBAL_H_ */
