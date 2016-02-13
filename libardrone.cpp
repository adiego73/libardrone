#include "include/libardrone.hpp"

using namespace robot;

void run(tesis::MessageServer *msgServer)
{
    RobotConfiguration robotConfig( "" );
    RobotConfig config = robotConfig.get();

    bool quit = false;

//     MessageClient msgClient;

    msgServer->announce( "robot/go_next_destination" );

    PID_Y pid_y( config.PID.Yaw.Kp, config.PID.Yaw.Ki, config.PID.Yaw.Kd, config.PID.Yaw.Derivator, config.PID.Yaw.Integrator, config.PID.Yaw.P_limit, config.PID.Yaw.I_max, config.PID.Yaw.I_min, 0 );
    PID_RP pid_r( config.PID.Roll.Kp, config.PID.Roll.Ki, config.PID.Roll.Kd, config.PID.Roll.Derivator, config.PID.Roll.Integrator, config.PID.Roll.P_limit, config.PID.Roll.I_max, 0 );
    PID_RP pid_p( config.PID.Pitch.Kp, config.PID.Pitch.Ki, config.PID.Pitch.Kd, config.PID.Pitch.Derivator, config.PID.Pitch.Integrator, config.PID.Pitch.P_limit, config.PID.Pitch.I_max, 0 );
    PID_Z pid_z( config.PID.Altitude.Kp, config.PID.Altitude.Kd, config.PID.Altitude.P_limit, 0 );

    ARDrone robot( config.address.c_str() );

    robot.setFlatTrim();

    msgServer->publish( "robot/go_next_destination", "true" );

    long elapsed_time = 0;
    
    while( !quit )
    {
        elapsed_time = std::stol(msgServer->get( "camera/elapsed_time", "0" ));
        
        std::string destination_x = msgServer->get( "camera/destination/x", "0" );
        std::string destination_y = msgServer->get( "camera/destination/y", "0" );

        Point destination;
        destination.x = std::stof( destination_x );
        destination.y = std::stof( destination_y );

        std::string finish_msg = msgServer->get( "gui/finish", "false" );
        std::istringstream( finish_msg ) >> std::boolalpha >> quit;
    }
}
