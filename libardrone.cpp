#include "include/libardrone.hpp"

using namespace robot;

void run()
{
    RobotConfiguration robotConfig( "" );
    RobotConfig config = robotConfig.get();

    MessageClient msgClient;

    msgClient.announce( "robot/go_next_destination" );

    PID_Y pid_y( config.PID.Yaw.Kp, config.PID.Yaw.Ki, config.PID.Yaw.Kd, config.PID.Yaw.Derivator, config.PID.Yaw.Integrator, config.PID.Yaw.P_limit, config.PID.Yaw.I_max, config.PID.Yaw.I_min, 0 );
    PID_RP pid_r( config.PID.Roll.Kp, config.PID.Roll.Ki, config.PID.Roll.Kd, config.PID.Roll.Derivator, config.PID.Roll.Integrator, config.PID.Roll.P_limit, config.PID.Roll.I_max, 0 );
    PID_RP pid_p( config.PID.Pitch.Kp, config.PID.Pitch.Ki, config.PID.Pitch.Kd, config.PID.Pitch.Derivator, config.PID.Pitch.Integrator, config.PID.Pitch.P_limit, config.PID.Pitch.I_max, 0 );
    PID_Z pid_z( config.PID.Altitude.Kp, config.PID.Altitude.Kd, config.PID.Altitude.P_limit, 0 );

    ARDrone robot(config.address.c_str());
    
    robot.setVideoRecord(false);
    robot.setOutdoorMode(false);
    
    
    msgClient.publish( "robot/go_next_destination", "true" );





}
