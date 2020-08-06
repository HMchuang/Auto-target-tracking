#include "position_controller.h"

int system_id = 1;
int autopilot_id = 1;
int companion_id = 0;

Position_Controller::Position_Controller(Multithreaded_Interface *mti)
{
	this->mti = mti;

	desired_position.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION;
	desired_position.coordinate_frame = MAV_FRAME_LOCAL_NED;

	desired_position_periodic = new Periodic_Message(mti, desired_position_message, 10);
	desired_attitude_periodic = new Periodic_Message(mti, desired_attitude_message, 10);

	update_current_position(0, 0, 0, 0);
	update_current_position_attitude(0, 0, 0, 0, 0, 0);
	update_desired_position(0, 0, 0, 0);
	
	float q_ini[4] = {1, 0, 0, 0};
        update_desired_attitude(q_ini, 0);
}

void Position_Controller::update_current_position(float x, float y, float z, float yaw)
{
	current_position.x = x;
	current_position.y = y;
	current_position.z = z;
	current_position.yaw = yaw;

	mavlink_msg_vision_position_estimate_encode(system_id, companion_id, &current_position_message, &current_position);
	mti->write_message(current_position_message);
}

void Position_Controller::update_current_position_attitude(float x, float y, float z, float roll, float pitch, float yaw)
{
	current_position.x = x;
	current_position.y = y;
	current_position.z = z;
	current_position.roll = roll;
	current_position.pitch = pitch;
	current_position.yaw = yaw;

	mavlink_msg_vision_position_estimate_encode(system_id, companion_id, &current_position_message, &current_position);
	mti->write_message(current_position_message);
}

void Position_Controller::update_desired_position(float x, float y, float z, float yaw)
{
	desired_position.x = x;
	desired_position.y = y;
	desired_position.z = z;
	desired_position.yaw = yaw;
	desired_position.target_system = 1;
	desired_position.target_component = 0;

	mavlink_msg_set_position_target_local_ned_encode(system_id, companion_id, &desired_position_message, &desired_position);
	desired_position_periodic->update_message(desired_position_message);
}

void Position_Controller::update_desired_attitude(float q[4], float thrust)
{
	//cout << "q0: " << q[0] << " q1: " << q[1] << " q2: " << q[2] << " q3: " << q[3] << endl;
	desire_attitude.q[0] = q[0];
	desire_attitude.q[1] = q[1];
	desire_attitude.q[2] = q[2];
	desire_attitude.q[3] = q[3];
	desire_attitude.thrust = thrust;
	desire_attitude.target_system = 1;
	desire_attitude.target_component = 0;

	mavlink_msg_set_attitude_target_encode(system_id, companion_id, &desired_attitude_message, &desire_attitude);
	desired_attitude_periodic->update_message(desired_attitude_message);
}

void Position_Controller::toggle_offboard_control(bool flag)
{
	// Prepare command for off-board mode
	mavlink_command_long_t com = {0};
	com.target_system = system_id;
	com.target_component = autopilot_id;
	com.command = MAV_CMD_NAV_GUIDED_ENABLE;
	com.confirmation = true;
	com.param1 = (float)flag; // flag >0.5 => start, <0.5 => stop

	// Encode
	mavlink_message_t message;
	mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

	// Send the message
	mti->write_message(message);
}

float Position_Controller::getLastAttitudeYaw()
{
	auto search = mti->last_messages.find(MAVLINK_MSG_ID_ATTITUDE);
    if (search != mti->last_messages.end())
    {
        mavlink_attitude_t att;
        mavlink_msg_attitude_decode(&(search->second), &att);
        //printf("[Roll Pitch Yaw] = [%f %f %f] \n", att.roll, att.pitch, att.yaw);
		return att.yaw;
    }
    else
    {
        std::cout << "Attitude Message Not found\n";
		return 0;
    }
}

void Position_Controller::shutdown()
{
	mti->shutdown();
}
