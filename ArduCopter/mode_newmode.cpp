#include "Copter.h"

bool Copter::ModeNewMode::init(bool ignore_checks)
{
	if(motors->armed() && ap.land_complete && !_copter.flightmode->has_manual_throttle() &&
	  (get_pilot_desired_throttle(channel_throttle->get_control_in()) > get_non_takeoff_throttle()))
	{
		return false;
	}
	
	pos_control->set_alt_target(0);
	return true;
}


void Copter::ModeNewMode::run()
{
	float target_roll, target_pitch;
	float target_yaw_rate;
	float pilot_throttle_scaled;
	
	get_pilot_desired_lean_angles(channel_roll->get_control_in(),channel_pitch->get_control_in(),target_roll,target_pitch,
								 aparm.angle_amx);
	
	target_yaw_rate  = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
	
	pilot_throttle_scaled = get_pilot_desired_throttle(channel_throttle->get_control_in());
	
	attitude_control->input_euler_angle_roll_pitch_yaw(target_roll,target_pitch,target_yaw, get_smoothing_gain());
	
	attitude_control->set_throttle_out(pilot_throttle_scaled, true, g.throttle_filt);

}
