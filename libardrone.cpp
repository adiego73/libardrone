#include "include/libardrone.hpp"

using namespace robot;

extern "C" void run( tesis::MessageServer* msgServer )
{
    RobotConfiguration robotConfig( "./config/libardrone.json" );
    RobotConfig config = robotConfig.get();

    bool quit = false;
    bool land = false;
    bool is_visible = false;
    bool hover = true;

    double fixYaw = 0;
    double yawValue = 0;
    double vx = 0, vy = 0, vz = 0;
    float yaw_set = 0, roll_set = 0, pitch_set = 0, altitude_set = 0;

    msgServer->announce( "robot/altitude" );

    msgServer->announce( "robot/pitch/kp" );
    msgServer->announce( "robot/pitch/ki" );
    msgServer->announce( "robot/pitch/kd" );
    msgServer->announce( "robot/pitch/set" );
    msgServer->announce( "robot/pitch/value" );

    msgServer->announce( "robot/roll/kp" );
    msgServer->announce( "robot/roll/ki" );
    msgServer->announce( "robot/roll/kd" );
    msgServer->announce( "robot/roll/set" );
    msgServer->announce( "robot/roll/value" );

    msgServer->announce( "robot/altitude/kp" );
    msgServer->announce( "robot/altitude/ki" );
    msgServer->announce( "robot/altitude/kd" );
    msgServer->announce( "robot/altitude/set" );
    msgServer->announce( "robot/altitude/value" );

    msgServer->announce( "robot/yaw/set" );
    msgServer->announce( "robot/yaw/value" );

    msgServer->announce( "robot/velocity/x" );
    msgServer->announce( "robot/velocity/y" );

    PID_Y pid_y( config.PID.Yaw.Kp, config.PID.Yaw.Ki, config.PID.Yaw.Kd, config.PID.Yaw.P_limit, config.PID.Yaw.I_max, config.PID.Yaw.I_min );
    PID_RP pid_r( config.PID.Roll.Kp, config.PID.Roll.Ki, config.PID.Roll.Kd, config.PID.Roll.P_limit, config.PID.Roll.I_max );
    PID_RP pid_p( config.PID.Pitch.Kp, config.PID.Pitch.Ki, config.PID.Pitch.Kd, config.PID.Pitch.P_limit, config.PID.Pitch.I_max );
    PID_Z pid_z( config.PID.Altitude.Kp, config.PID.Altitude.Kd, config.PID.Altitude.P_limit );

    ARDrone robot;

    if( robot.open( config.address.c_str() ) != 1 )
    {
        std::cerr << "ERROR when connecting to ARDrone" << std::endl;
    }

    robot.setFlatTrim();

    msgServer->publish( "robot/altitude", std::to_string( robot.getAltitude() ) );

    long elapsed_time = 0;

    pid_z.setPoint( 800 );

    std::cout << "Bateria: " << robot.getBatteryPercentage() <<  std::endl;

    while( !quit )
    {
//         std::cout << "Bateria: " << robot.getBatteryPercentage() << " %" << std::endl;
        elapsed_time = std::stol( msgServer->get( "camera/elapsed_time", "0" ) );

        land = msgServer->get( "gui/action/land", "false" ).find( "false" ) == std::string::npos;

        if( !land )
        {
            is_visible = msgServer->get( "camera/robot_found", "false" ).find( "true" ) != std::string::npos;

            // if the robot is visible and is flying
            if( is_visible && robot.onGround() ==  0 )
            {
                if (fixYaw ==  0)
                    fixYaw = Util::rad_to_deg( -robot.getYaw());
                
                yawValue = Util::rad_to_deg( -robot.getYaw()) - fixYaw;
                yawValue = Util::normalize_angle(yawValue);
                
		
		Point position;
		position.x = Util::getMsgFloat(msgServer, "camera/robot_position/x");
		position.y = Util::getMsgFloat(msgServer, "camera/robot_position/y");
                position.z = Util::getMsgFloat(msgServer, "camera/robot_position/z");

                Point destination;
                destination.x = Util::getMsgFloat(msgServer, "camera/destination/x", "0");
                destination.y = Util::getMsgFloat(msgServer, "camera/destination/y", "0");

                
                float setpoint_yaw = Util::get_angle_as_deg( position, destination, yawValue );
                pid_y.setPoint( setpoint_yaw );

                pid_r.setPoint( destination.x );
                pid_p.setPoint( destination.y );

                float angle = setpoint_yaw;
                angle -= yawValue;
                angle = Util::normalize_angle( angle );

                float distance = Util::distance( position, destination );

                altitude_set = pid_z.update( robot.getAltitude(), 0 /* this value is not used */, elapsed_time );

                Velocity velocity;
                robot.getVelocity( &vx, &vy, &vz );

                velocity.x = vx / 0.001;
                velocity.y = -vy / 0.001;
                velocity.z = -vz / 0.001;

                if( distance < 10 )
                {
                    hover = true;
                    roll_set = 0;
                    pitch_set = 0;

                    pid_p.reset();
                    pid_r.reset();
                }
                else
                {
                    hover = false;

                    Point relative_position = Util::get_point( distance, angle );
                    roll_set = pid_r.update( ( relative_position.x * -1 ), velocity.x, elapsed_time ) ;
                    pitch_set = pid_p.update( relative_position.y, velocity.y, elapsed_time );
                }

                
                std::cout << "--------------" << robot.getYaw() << " " << yawValue << std::endl;
                
                msgServer->publish( "robot/altitude", std::to_string( robot.getAltitude() ) );

                msgServer->publish( "robot/pitch/kp", std::to_string( pid_p.getKp() ) );
                msgServer->publish( "robot/pitch/ki", std::to_string( pid_p.getKi() ) );
                msgServer->publish( "robot/pitch/kd", std::to_string( pid_p.getKd() ) );
                msgServer->publish( "robot/pitch/set", std::to_string( pitch_set ) );
                msgServer->publish( "robot/pitch/value", std::to_string( Util::rad_to_deg( -robot.getPitch()) ) );

                msgServer->publish( "robot/roll/kp", std::to_string( pid_r.getKp() ) );
                msgServer->publish( "robot/roll/ki", std::to_string( pid_r.getKi() ) );
                msgServer->publish( "robot/roll/kd", std::to_string( pid_r.getKd() ) );
                msgServer->publish( "robot/roll/set", std::to_string( roll_set ) );
                msgServer->publish( "robot/roll/value", std::to_string( Util::rad_to_deg( robot.getRoll()) ) );

                msgServer->publish( "robot/altitude/kp", std::to_string( pid_z.getKp() ) );
                msgServer->publish( "robot/altitude/ki", std::to_string( pid_z.getKi() ) );
                msgServer->publish( "robot/altitude/kd", std::to_string( pid_z.getKd() ) );
                msgServer->publish( "robot/altitude/set", std::to_string( altitude_set ) );
                msgServer->publish( "robot/altitude/value", std::to_string( robot.getAltitude() ) );

                msgServer->publish( "robot/yaw/set", std::to_string( yaw_set ) );
                msgServer->publish( "robot/yaw/value", std::to_string( yawValue ) );

                msgServer->publish( "robot/velocity/x", std::to_string( velocity.x ) );
                msgServer->publish( "robot/velocity/y", std::to_string( velocity.y ) );
            }
            // if it is visible and is on ground, wait for the takeoff message.
            else if( is_visible && robot.onGround() ==  1 )
            {
                bool takeoff = msgServer->get( "gui/action/takeoff", "false" ).find( "true" ) != std::string::npos;

                if( takeoff )
                {
                    robot.takeoff();
                }
            }
            else
            {
                // if not visible and is flying, land.
                if( robot.onGround() == 0 ) 
                {
                    fixYaw = 0;
                    robot.landing();
                }
            }

            if( hover )
            {
                // if hove is set on true, and is flying, hover.
                if( robot.onGround() == 0 ) robot.move3D( 0, 0, altitude_set, yaw_set );
            }
            else
            {
                // if should move.
                if( robot.onGround() == 0 ) robot.move3D( roll_set / -0.2, pitch_set / -0.2, altitude_set, yaw_set );
            }
        }
        else
        {
            // if it should land.
            if( robot.onGround() == 0 ) robot.landing();
        }

        std::string finish_msg = msgServer->get( "gui/finish", "false" );
        std::istringstream( finish_msg ) >> std::boolalpha >> quit;
    }

    robot.close();
}

