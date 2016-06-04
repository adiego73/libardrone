#include "classes/robot_configuration.hpp"

using namespace robot;
using namespace boost;

RobotConfiguration::RobotConfiguration ( std::string path ) : path ( path )
{
	this->parse();
}

RobotConfiguration::~RobotConfiguration()
{

}

RobotConfig RobotConfiguration::get()
{
	return this->configuration;
}

void RobotConfiguration::parse()
{
	// http://theboostcpplibraries.com/boost.propertytree
	try {
		RobotConfig config;

		PIDConfig pid_config;

		PIDValues roll_values;
		PIDValues pitch_values;
		PIDValues yaw_values;
		PIDValues altitude_values;

		property_tree::ptree ptree;
		property_tree::read_json ( this->path, ptree );

		// if there is no address in the robot config, use the default ARDrone address
		config.address = ptree.get ( "address", "192.168.1.1" );

		roll_values.Kp = ptree.get<float> ( "PID.Roll.Kp", 0.0f );
		roll_values.Ki = ptree.get<float> ( "PID.Roll.Ki", 0.0f );
		roll_values.Kd = ptree.get<float> ( "PID.Roll.Kd", 0.0f );
		roll_values.P_limit = ptree.get<float> ( "PID.Roll.P_limit", 0.0f );
		roll_values.I_max = ptree.get<long> ( "PID.Roll.I_max", 0 );
		roll_values.I_min = ptree.get<long> ( "PID.Roll.I_min", 0 );

		pitch_values.Kp = ptree.get<float> ( "PID.Pitch.Kp", 0.0f );
		pitch_values.Ki = ptree.get<float> ( "PID.Pitch.Ki", 0.0f );
		pitch_values.Kd = ptree.get<float> ( "PID.Pitch.Kd", 0.0f );
		pitch_values.P_limit = ptree.get<float> ( "PID.Pitch.P_limit", 0.0f );
		pitch_values.I_max = ptree.get<long> ( "PID.Pitch.I_max", 0 );
		pitch_values.I_min = ptree.get<long> ( "PID.Pitch.I_min", 0 );

		yaw_values.Kp = ptree.get<float> ( "PID.Yaw.Kp", 0.0f );
		yaw_values.Ki = ptree.get<float> ( "PID.Yaw.Ki", 0.0f );
		yaw_values.Kd = ptree.get<float> ( "PID.Yaw.Kd", 0.0f );
		yaw_values.P_limit = ptree.get<float> ( "PID.Yaw.P_limit", 0.0f );
		yaw_values.I_max = ptree.get<long> ( "PID.Yaw.I_max", 0 );
		yaw_values.I_min = ptree.get<long> ( "PID.Yaw.I_min", 0 );

		altitude_values.Kp = ptree.get<float> ( "PID.Altitude.Kp", 0.0f );
		altitude_values.Ki = ptree.get<float> ( "PID.Altitude.Ki", 0.0f );
		altitude_values.Kd = ptree.get<float> ( "PID.Altitude.Kd", 0.0f );
		altitude_values.P_limit = ptree.get<float> ( "PID.Altitude.P_limit", 0.0f );
		altitude_values.I_max = ptree.get<long> ( "PID.Altitude.I_max", 0 );
		altitude_values.I_min = ptree.get<long> ( "PID.Altitude.I_min", 0 );

		pid_config.Roll = roll_values;
		pid_config.Pitch = pitch_values;
		pid_config.Yaw = yaw_values;
		pid_config.Altitude = altitude_values;

		config.PID = pid_config;

		this->configuration = config;
	} catch ( std::exception const& e ) {
		std::cerr << e.what() << std::endl;
	}
}

