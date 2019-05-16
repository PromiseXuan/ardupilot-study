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
     Test for AP_GPS_AUTO
*/

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

const AP_HAL::HAL& hal = AP_HAL::get_HAL(); //hal����

static AP_BoardConfig board_config; //board��ʼ������

// create board led object
AP_BoardLED board_led;//����һ����LED����

// This example uses GPS system. Create it.
static AP_GPS gps;  //����һ��GPS����
// Serial manager is needed for UART communications    
//����ͨ����Ҫ���й�����
static AP_SerialManager serial_manager; //����һ�����й�������

//to be called only once on boot for initializing objects
//ֻ����������ʼ������ʱ����һ��
void setup()
{
	
    hal.console->printf("GPS AUTO library test\n");//����̨�����Ϣ
	//console ����һ��UARTDriverָ�����

    board_config.init();//��ʼ����

    // Initialise the leds
    board_led.init();//LED��ʼ��

    // Initialize the UART for GPS system
    serial_manager.init();//���й�������ʼ��
    gps.init(serial_manager);//GPS��ʼ��
}


/*
  print a int32_t lat/long in decimal degrees
 */
//longitude������
//latitude��γ��
void print_latlon(AP_HAL::BetterStream *s, int32_t lat_or_lon)
{
    int32_t dec_portion, frac_portion;//ʮλ�����ֺ�С������
    int32_t abs_lat_or_lon = labs(lat_or_lon);//����ֵ

    // extract decimal portion (special handling of negative numbers to ensure we round towards zero)
	//��ȡʮλ�����֣��Ը����������⴦������ȷ�������������룩
    dec_portion = abs_lat_or_lon / 10000000UL;

    // extract fractional portion����ȡ��������
    frac_portion = abs_lat_or_lon - dec_portion*10000000UL;

    // print output including the minus sign
    if( lat_or_lon < 0 ) {
        s->printf("-");
    }
    s->printf("%ld.%07ld",(long)dec_portion,(long)frac_portion);//��ӡ����
}

// loop
void loop()
{
    static uint32_t last_msg_ms;

    // Update GPS state based on possible bytes received from the module.
    gps.update();//����gps��Ϣ

    // If new GPS data is received, output it's contents to the console
    // Here we rely on the time of the message in GPS class and the time of last message
    // saved in static variable last_msg_ms. When new message is received, the time
    // in GPS class will be updated.
    if (last_msg_ms != gps.last_message_time_ms()) {
        // Reset the time of message
        last_msg_ms = gps.last_message_time_ms();

        // Acquire location
        const Location &loc = gps.location();//��ȡλ����Ϣ

        // Print the contents of message
        hal.console->printf("Lat: ");
        print_latlon(hal.console, loc.lat);//��ӡγ�ȣ�hal.console�������
        hal.console->printf(" Lon: ");
        print_latlon(hal.console, loc.lng);//��ӡ����
        hal.console->printf(" Alt: %.2fm GSP: %.2fm/s CoG: %d SAT: %d TIM: %u/%lu STATUS: %u\n",
                            (double)(loc.alt * 0.01f),
                            (double)gps.ground_speed(),
                            (int)gps.ground_course_cd() / 100,
                            gps.num_sats(),
                            gps.time_week(),
                            (long unsigned int)gps.time_week_ms(),
                            gps.status());
    }

    // Delay for 10 mS will give us 100 Hz invocation rate
	//�ӳ�10 mS��Ϊ�����ṩ100 Hz�ĵ�����
    hal.scheduler->delay(10);
}

// Register above functions in HAL board level
AP_HAL_MAIN();