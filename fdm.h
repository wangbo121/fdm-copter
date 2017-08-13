/*
 * fdm.h
 *
 *  Created on: 2017-8-12
 *      Author: wangbo
 */

#ifndef FDM_H_
#define FDM_H_


/*
 * 简单的四旋翼的飞行动力学模型flight dynamics model
 * 这个是需要跟flightgear的版本是一致的，所以用的时候小心点
 */



#include <time.h> // time_t
#include <stdint.h>

// NOTE: this file defines an external interface structure.  Due to
// variability between platforms and architectures, we only used fixed
// length types here.  Specifically, integer types can vary in length.
// I am not aware of any platforms that don't use 4 bytes for float
// and 8 bytes for double.

#define  FG_NET_FDM_VERSION 24


// Define a structure containing the top level flight dynamics model
// parameters

//const int FG_MAX_ENGINES = 4;
//const int FG_MAX_WHEELS = 3;
//const int FG_MAX_TANKS = 4;
#if 1
enum fg_enum
{
	FG_MAX_ENGINES = 4,
	FG_MAX_WHEELS = 3,
	FG_MAX_TANKS = 4
};
#endif
typedef struct FGNetFDM
{
    //enum fg_enum fg;


    uint32_t version;		// increment when data values change
    uint32_t padding;		// padding  8

    // Positions
    double longitude;		// geodetic (radians) 16
    double latitude;		// geodetic (radians)
    double altitude;		// above sea level (meters)  32
    float agl;			// above ground level (meters)
    float phi;			// roll (radians)
    float theta;		// pitch (radians)
    float psi;			// yaw or true heading (radians)
    float alpha;                // angle of attack (radians)
    float beta;                 // side slip angle (radians)

    // Velocities
    float phidot;		// roll rate (radians/sec)
    float thetadot;		// pitch rate (radians/sec)
    float psidot;		// yaw rate (radians/sec)
    float vcas;		        // calibrated airspeed
    float climb_rate;		// feet per second
    float v_north;              // north velocity in local/body frame, fps
    float v_east;               // east velocity in local/body frame, fps
    float v_down;               // down/vertical velocity in local/body frame, fps
    float v_body_u;    // ECEF velocity in body frame
    float v_body_v;    // ECEF velocity in body frame    96

    float v_body_w;    // ECEF velocity in body frame +40+28=100


    // Accelerations
    float A_X_pilot;		// X accel in body frame ft/sec^2  //104

    float A_Y_pilot;		// Y accel in body frame ft/sec^2
    float A_Z_pilot;		// Z accel in body frame ft/sec^2 112

    // Stall
    float stall_warning;        // 0.0 - 1.0 indicating the amount of stall
    float slip_deg;		// slip ball deflection

    // Pressure


    // Engine status
    uint32_t num_engines;	     // Number of valid engines
    uint32_t eng_state[FG_MAX_ENGINES];// Engine state (off, cranking, running)

    float rpm[FG_MAX_ENGINES];	     // Engine RPM rev/min  132

    float fuel_flow[FG_MAX_ENGINES]; // Fuel flow gallons/hr
    float fuel_px[FG_MAX_ENGINES];   // Fuel pressure psi
    float egt[FG_MAX_ENGINES];	     // Exhuast gas temp deg F
    float cht[FG_MAX_ENGINES];	     // Cylinder head temp deg F
    float mp_osi[FG_MAX_ENGINES];    // Manifold pressure
    float tit[FG_MAX_ENGINES];	     // Turbine Inlet Temperature
    float oil_temp[FG_MAX_ENGINES];  // Oil temp deg F
    float oil_px[FG_MAX_ENGINES];    // Oil pressure psi

    // Consumables
    uint32_t num_tanks;		// Max number of fuel tanks
    float fuel_quantity[FG_MAX_TANKS];

    // Gear status
    uint32_t num_wheels;
    uint32_t wow[FG_MAX_WHEELS]; //+40=140
    float gear_pos[FG_MAX_WHEELS];
    float gear_steer[FG_MAX_WHEELS];
    float gear_compression[FG_MAX_WHEELS];

    // Environment
    uint32_t cur_time;           // current unix time
                                 // FIXME: make this uint64_t before 2038
    int32_t warp;                // offset in seconds to unix time
    float visibility;            // visibility in meters (for env. effects)

    // Control surface positions (normalized values)
    float elevator;
    float elevator_trim_tab;
    float left_flap;
    float right_flap;
    float left_aileron;
    float right_aileron;
    float rudder;
    float nose_wheel;
    float speedbrake;
    float spoilers; //+16*4=204



}T_FDM;

extern T_FDM fdm;





#endif /* FDM_H_ */
