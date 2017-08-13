/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  parent class for aircraft simulators
*/


#include "SIM_Aircraft.h"
#include <unistd.h>
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h> //#define	RAND_MAX	2147483647
#include <iostream>

float longitude_scale(const struct Location &loc)
{
    float scale = cosf(loc.lat * 1.0e-7f * DEG_TO_RAD);
    return constrain_float(scale, 0.01f, 1.0f);
}

unsigned char is_zero(float x)
{
	if(x==0.0)
	{
		return 1;
	}
	else
	{
		return 0;
	}

}

/*
 *  extrapolate latitude/longitude given distances north and east
 */
void location_offset(struct Location &loc, float ofs_north, float ofs_east)
{
    if (!is_zero(ofs_north) || !is_zero(ofs_east))
    {
        int32_t dlat = ofs_north * LOCATION_SCALING_FACTOR_INV;
        int32_t dlng = (ofs_east * LOCATION_SCALING_FACTOR_INV) / longitude_scale(loc);
        loc.lat += dlat;
        loc.lng += dlng;
    }
}

void location_update(struct Location &loc, float bearing, float distance)
{
    float ofs_north = cosf(radians(bearing))*distance;
    float ofs_east  = sinf(radians(bearing))*distance;
    location_offset(loc, ofs_north, ofs_east);
}

/*
  parent class for all simulator types
 */

/*
  constructor
 */
Aircraft::Aircraft(const char *home_str, const char *frame_str) :
    ground_level(0),
    frame_height(0),
    dcm(),
    gyro(),
    velocity_ef(),
    mass(0),
    accel_body(0, 0, -GRAVITY_MSS),
    time_now_us(0),
    gyro_noise(radians(0.1f)),
    accel_noise(0.3),
    rate_hz(400),
    autotest_dir(NULL),
    frame(frame_str),
#ifdef __CYGWIN__
    min_sleep_time(20000)
#else
    min_sleep_time(5000)
#endif
{
    char *saveptr=NULL;
    char *s = strdup(home_str);
    char *lat_s = strtok_r(s, ",", &saveptr);
    char *lon_s = strtok_r(NULL, ",", &saveptr);
    char *alt_s = strtok_r(NULL, ",", &saveptr);
    char *yaw_s = strtok_r(NULL, ",", &saveptr);

    memset(&home, 0, sizeof(home));
    home.lat = atof(lat_s) * 1.0e7;
    home.lng = atof(lon_s) * 1.0e7;
    home.alt = atof(alt_s) * 1.0e2;
    location = home;
    ground_level = home.alt*0.01;

    std::cout<<"location.lng="<<location.lng<<std::endl;
    std::cout<<"location.lat="<<location.lat<<std::endl;

    dcm.from_euler(0, 0, radians(atof(yaw_s)));
    free(s);

    set_speedup(1);

    last_wall_time_us = get_wall_time_us();
    frame_counter = 0;
}

/*
   return true if we are on the ground
*/
bool Aircraft::on_ground(const Vector3f &pos) const
{
    return (-pos.z) + home.alt*0.01f <= ground_level + frame_height;
}

/*
   update location from position
*/
void Aircraft::update_position(void)
{
    float bearing = degrees(atan2f(position.y, position.x));
    float distance = sqrtf(sq(position.x) + sq(position.y));

    location = home;
    location_update(location, bearing, distance);

    location.alt  = home.alt - position.z*100.0f;

    // we only advance time if it hasn't been advanced already by the
    // backend
    if (last_time_us == time_now_us) {
        time_now_us += frame_time_us;
    }
    last_time_us = time_now_us;
    sync_frame_time();
}

/*
   rotate to the given yaw
*/
void Aircraft::set_yaw_degrees(float yaw_degrees)
{
    float roll, pitch, yaw;
    dcm.to_euler(&roll, &pitch, &yaw);

    yaw = radians(yaw_degrees);
    dcm.from_euler(roll, pitch, yaw);
}

/* advance time by deltat in seconds */
void Aircraft::time_advance(float deltat)
{
    time_now_us += deltat * 1.0e6f;
}

/* setup the frame step time */
void Aircraft::setup_frame_time(float new_rate, float new_speedup)
{
    rate_hz = new_rate;
    target_speedup = new_speedup;
    frame_time_us = 1.0e6f/rate_hz;

    scaled_frame_time_us = frame_time_us/target_speedup;
    last_wall_time_us = get_wall_time_us();
    achieved_rate_hz = rate_hz;
}

/* adjust frame_time calculation */
void Aircraft::adjust_frame_time(float new_rate)
{
    if (rate_hz != new_rate) {
        rate_hz = new_rate;
        frame_time_us = 1.0e6f/rate_hz;
        scaled_frame_time_us = frame_time_us/target_speedup;
    }
}

/* 
   try to synchronise simulation time with wall clock time, taking
   into account desired speedup 
   This tries to take account of possible granularity of
   get_wall_time_us() so it works reasonably well on windows
*/
void Aircraft::sync_frame_time(void)
{
    frame_counter++;
    uint64_t now = get_wall_time_us();
    if (frame_counter >= 40 &&
        now > last_wall_time_us) {
        float rate = frame_counter * 1.0e6f/(now - last_wall_time_us);
        achieved_rate_hz = (0.99f*achieved_rate_hz) + (0.01f*rate);
        if (achieved_rate_hz < rate_hz * target_speedup) {
            scaled_frame_time_us *= 0.999f;
        } else {
            scaled_frame_time_us /= 0.999f;
        }
#if 0
        ::printf("achieved_rate_hz=%.3f rate=%.2f rate_hz=%.3f sft=%.1f\n",
                 (double)achieved_rate_hz, 
                 (double)rate,
                 (double)rate_hz,
                 (double)scaled_frame_time_us);
#endif
        uint32_t sleep_time = scaled_frame_time_us*frame_counter;
        if (sleep_time > min_sleep_time) {
            usleep(sleep_time);
        }
        last_wall_time_us = now;
        frame_counter = 0;
    }
}

/* add noise based on throttle level (from 0..1) */
void Aircraft::add_noise(float throttle)
{
    gyro += Vector3f(rand_normal(0, 1),
                     rand_normal(0, 1),
                     rand_normal(0, 1)) * gyro_noise * throttle;
    accel_body += Vector3f(rand_normal(0, 1),
                           rand_normal(0, 1),
                           rand_normal(0, 1)) * accel_noise * throttle;
}

/*
  normal distribution random numbers
  See
  http://en.literateprograms.org/index.php?title=Special:DownloadCode/Box-Muller_transform_%28C%29&oldid=7011
*/
double Aircraft::rand_normal(double mean, double stddev)
{
    static double n2 = 0.0;
    static int n2_cached = 0;
    if (!n2_cached)
    {
        double x, y, r;
        do
        {
            x = 2.0*rand()/RAND_MAX - 1;
            y = 2.0*rand()/RAND_MAX - 1;

            r = x*x + y*y;
        }
        while (r == 0.0 || r > 1.0);
        {
            double d = sqrt(-2.0*log(r)/r);
            double n1 = x*d;
            n2 = y*d;
            double result = n1*stddev + mean;
            n2_cached = 1;
            return result;
        }
    }
    else
    {
        n2_cached = 0;
        return n2*stddev + mean;
    }
}




/*
   fill a sitl_fdm structure from the simulator state
*/
void Aircraft::fill_fdm(struct sitl_fdm &fdm) const
{
    fdm.timestamp_us = time_now_us;
    fdm.latitude  = location.lat * 1.0e-7;
    fdm.longitude = location.lng * 1.0e-7;
    fdm.altitude  = location.alt * 1.0e-2;
    fdm.heading   = degrees(atan2f(velocity_ef.y, velocity_ef.x));
    fdm.speedN    = velocity_ef.x;
    fdm.speedE    = velocity_ef.y;
    fdm.speedD    = velocity_ef.z;
    fdm.xAccel    = accel_body.x;
    fdm.yAccel    = accel_body.y;
    fdm.zAccel    = accel_body.z;
    fdm.rollRate  = degrees(gyro.x);
    fdm.pitchRate = degrees(gyro.y);
    fdm.yawRate   = degrees(gyro.z);
    float r, p, y;
    dcm.to_euler(&r, &p, &y);
    fdm.rollDeg  = degrees(r);
    fdm.pitchDeg = degrees(p);
    fdm.yawDeg   = degrees(y);
    fdm.airspeed = airspeed;
    fdm.magic = 0x4c56414f;



    std::cout<<"fdm.longitude="<<fdm.longitude<<std::endl;
    std::cout<<"fdm.latitude="<<fdm.latitude<<std::endl;
    std::cout<<"fdm.altitude="<<fdm.altitude<<std::endl;
    std::cout<<"fdm.heading ="<<fdm.heading <<std::endl;
}

/*
   fill a sitl_fdm structure from the simulator state
*/
void Aircraft::fill_fdm_flightgear( T_FDM &fdm) const
{


	   std::cout<<"location.longitude="<<location.lng<<std::endl;
	    std::cout<<"location.latitude="<<location.lat<<std::endl;



	fdm.version=24;
	fdm.longitude=radians(location.lng * 1.0e-7);
	fdm.latitude=radians(location.lat * 1.0e-7);


	std::cout<<"fdm.longitude wangbo="<<fdm.longitude<<std::endl;
	std::cout<<"fdm.latitude wangbo="<<fdm.latitude<<std::endl;


	fdm.altitude=location.alt * 1.0e-2;

	 float r, p, y;
	dcm.to_euler(&r, &p, &y);
	fdm.phi=r;
	fdm.theta=p;
	fdm.psi=y;

	fdm.phidot=gyro.x;
	fdm.thetadot=gyro.y;
	fdm.psidot=gyro.z;

	fdm.v_north=velocity_ef.x;
	fdm.v_east=velocity_ef.y;
	fdm.v_down=velocity_ef.z;

	fdm.A_X_pilot=accel_body.x;
	fdm.A_Y_pilot=accel_body.y;
	fdm.A_Z_pilot=accel_body.z;

	fdm.cur_time=time_now_us;

	fdm.visibility=5000.0; // meters 可见范围

	std::cout<<"fdm.phi="<<fdm.phi<<std::endl;
	std::cout<<"fdm.theta="<<fdm.theta<<std::endl;
	std::cout<<"fdm.psi="<<fdm.psi<<std::endl;
	std::cout<<"fdm.altitude="<<fdm.altitude<<std::endl;


#if 0
    fdm.timestamp_us = time_now_us;
    fdm.latitude  = location.lat * 1.0e-7;
    fdm.longitude = location.lng * 1.0e-7;
    fdm.altitude  = location.alt * 1.0e-2;
    fdm.heading   = degrees(atan2f(velocity_ef.y, velocity_ef.x));
    fdm.speedN    = velocity_ef.x;
    fdm.speedE    = velocity_ef.y;
    fdm.speedD    = velocity_ef.z;
    fdm.xAccel    = accel_body.x;
    fdm.yAccel    = accel_body.y;
    fdm.zAccel    = accel_body.z;
    fdm.rollRate  = degrees(gyro.x);
    fdm.pitchRate = degrees(gyro.y);
    fdm.yawRate   = degrees(gyro.z);
    float r, p, y;
    dcm.to_euler(&r, &p, &y);
    fdm.rollDeg  = degrees(r);
    fdm.pitchDeg = degrees(p);
    fdm.yawDeg   = degrees(y);
    fdm.airspeed = airspeed;
    fdm.magic = 0x4c56414f;
#endif





}

uint64_t Aircraft::get_wall_time_us() const
{
#ifdef __CYGWIN__
    static DWORD tPrev;
    static uint64_t last_ret_us;
    if (tPrev == 0) {
        tPrev = timeGetTime();
        return 0;
    }
    DWORD now = timeGetTime();
    last_ret_us += (uint64_t)((now - tPrev)*1000UL);
    tPrev = now;
    return last_ret_us;
#else
    struct timeval tp;
    gettimeofday(&tp,NULL);
    return tp.tv_sec*1.0e6 + tp.tv_usec;
#endif
}

/*
  set simulation speedup
 */
void Aircraft::set_speedup(float speedup)
{
    setup_frame_time(rate_hz, speedup);
}

