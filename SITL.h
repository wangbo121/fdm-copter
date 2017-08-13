/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __SITL_H__
#define __SITL_H__

#include <stdint.h>

#include "BIT_MATH.h"

// scaling factor from 1e-7 degrees to meters at equater
// == 1.0e-7 * DEG_TO_RAD * RADIUS_OF_EARTH
#define LOCATION_SCALING_FACTOR 0.011131884502145034f
// inverse of LOCATION_SCALING_FACTOR
#define LOCATION_SCALING_FACTOR_INV 89.83204953368922f

// acceleration due to gravity in m/s/s
#define GRAVITY_MSS     9.80665f

// radius of earth in meters
#define RADIUS_OF_EARTH 6378100

// convert a longitude or latitude point to meters or centimeteres.
// Note: this does not include the longitude scaling which is dependent upon location
#define LATLON_TO_M     0.01113195f
#define LATLON_TO_CM    1.113195f

#define ToRad(x) radians(x)	// *pi/180
#define ToDeg(x) degrees(x)	// *180/pi

//// degrees -> radians
//static inline float radians(float deg)
//{
//    return deg * DEG_TO_RAD;
//}
//
//// radians -> degrees
//static inline float degrees(float rad)
//{
//    return rad * RAD_TO_DEG;
//}

struct  sitl_fdm {
    // this is the packet sent by the simulator
    // to the APM executable to update the simulator state
    // All values are little-endian
    uint64_t timestamp_us;
    double latitude, longitude; // degrees
    double altitude;  // MSL
    double heading;   // degrees
    double speedN, speedE, speedD; // m/s
    double xAccel, yAccel, zAccel;       // m/s/s in body frame
    double rollRate, pitchRate, yawRate; // degrees/s/s in body frame
    double rollDeg, pitchDeg, yawDeg;    // euler angles, degrees
    double airspeed; // m/s
    uint32_t magic; // 0x4c56414f
};

// number of rc output channels
#define SITL_NUM_CHANNELS 14


class SITL
{
public:

    SITL() {
//        // set a default compass offset
//        mag_ofs.set(Vector3f(5, 13, -18));
//        AP_Param::setup_object_defaults(this, var_info);
    }

    enum GPSType {
        GPS_TYPE_NONE  = 0,
        GPS_TYPE_UBLOX = 1,
        GPS_TYPE_MTK   = 2,
        GPS_TYPE_MTK16 = 3,
        GPS_TYPE_MTK19 = 4,
        GPS_TYPE_NMEA  = 5,
        GPS_TYPE_SBP   = 6,
    };

    struct sitl_fdm state;

    //static const struct AP_Param::GroupInfo var_info[];


    // noise levels for simulated sensors
    float baro_noise;  // in metres
    float baro_drift;  // in metres per second
    float baro_glitch; // glitch in meters
    float gyro_noise;  // in degrees/second
    float accel_noise; // in m/s/s
    float accel2_noise; // in m/s/s
    Vector3f accel_bias; // in m/s/s
    float aspd_noise;  // in m/s
    float aspd_fail;   // pitot tube failure

    float mag_noise;   // in mag units (earth field is 818)
    float mag_error;   // in degrees
    Vector3f mag_mot;  // in mag units per amp
    Vector3f mag_ofs;  // in mag units
    float servo_rate;  // servo speed in degrees/second

    float sonar_glitch;// probablility between 0-1 that any given sonar sample will read as max distance
    float sonar_noise; // in metres
    float sonar_scale; // meters per volt

    float drift_speed; // degrees/second/minute
    float drift_time;  // period in minutes
    float engine_mul;  // engine multiplier
    uint8_t  gps_disable; // disable simulated GPS
    uint8_t  gps2_enable; // enable 2nd simulated GPS
    uint8_t  gps_delay;   // delay in samples
    uint8_t  gps_type;    // see enum GPSType
    float gps_byteloss;// byte loss as a percent
    uint8_t  gps_numsats; // number of visible satellites
    Vector3f  gps_glitch;  // glitch offsets in lat, lon and altitude
    uint8_t  gps_hertz;   // GPS update rate in Hz
    float batt_voltage; // battery voltage base
    float accel_fail;  // accelerometer failure value
    uint8_t  rc_fail;     // fail RC input
    uint8_t  baro_disable; // disable simulated barometer
    uint8_t  float_exception; // enable floating point exception checks
    uint8_t  flow_enable; // enable simulated optflow
    uint16_t flow_rate; // optflow data rate (Hz)
    uint8_t  flow_delay; // optflow data delay
    uint8_t  terrain_enable; // enable using terrain for height

    // wind control
    float wind_speed;
    float wind_direction;
    float wind_turbulance;
    float gps_drift_alt;

    uint16_t a;
    uint16_t  baro_delay; // barometer data delay in ms
    uint16_t  mag_delay; // magnetometer data delay in ms
    uint16_t  wind_delay; // windspeed data delay in ms

//    void simstate_send(mavlink_channel_t chan);

   // void Log_Write_SIMSTATE(DataFlash_Class &dataflash);

    // convert a set of roll rates from earth frame to body frame
    static void convert_body_frame(double rollDeg, double pitchDeg,
                                   double rollRate, double pitchRate, double yawRate,
                                   double *p, double *q, double *r);

    // convert a set of roll rates from body frame to earth frame
    static Vector3f convert_earth_frame(const Matrix3f &dcm, const Vector3f &gyro);
};

#endif // __SITL_H__
