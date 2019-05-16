
#include <stdlib.h>

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Baro/AP_Baro.h>
#include <Filter/Filter.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_Declination/AP_Declination.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_Mission/AP_Mission.h>
#include <StorageManager/StorageManager.h>
#include <AP_Terrain/AP_Terrain.h>
#include <AP_Math/AP_Math.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_Notify/AP_BoardLED.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <AP_BoardConfig/AP_BoardConfig.h>


void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static AP_BoardConfig board_config;

AP_BoardLED board_led;

static AP_GPS gps;

static AP_SerialManager serial_manager;

void setup()
{
	hal.console->printf("My GPS AUTO library test!\n");
	hal.console->printf("Hello World!\n");
	
	board_config.init();
	
	board_led.init();
	
	serial_manager.init();
	
	gps.init(serial_manager);
	
	hal.console->printf("Setup init finished!\n");
}

void print_data_latlon(AP_HAL::BetterStream *s, int32_t lat_or_lon)
{
	int32_t der_portion, frac_portion;
	int32_t abs_lat_or_lon = labs(lat_or_lon);
	
	dec_portion = abs_lat_or_lon / 10000000UL;
	frac_portion = abs_lat_or_lon - dec_portion * 10000000UL;
	
	if(lat_or_lon < 0)
	{
		s->printf("-");
	}
	s->printf("%ld.%07ld",(long)dec_portion,(long)frac_portion);
}

void loop()
{
	static uint32_t last_msg_ms;
	
	gps.update();
	
	if(last_msg_ms != gps.last_message_time_ms())
	{
		last_msg_ms = gps.last_message_time_ms();
		
		const Location &loc = gps.location();
		
		hal.console->printf("Lat: ");
		print_data_latlon(hal.console, loc.lat);
		hal.console->printf("Lon: ");
		print_data_latlon(hal.console, loc.lng);
		
		hal.console->printf("Alt: %.2fm GSP: %.2fm/s CoG: %d SAT: %d TIM: %u/%lu STATUS: %u\n",
						   (double)(loc.alt * 0.01f),
						   (double)gps.ground_speed(),
						   (int)gps.ground_course_cd() / 100,
						    gps.num_sats(),
						    gps.time_week(),
						    (long unsigned int)gps.time_week_ms(),
							gps.status());	
	}
	
	hal.scheduler->delay(10);
	
}

AP_HAL_MAIN();
