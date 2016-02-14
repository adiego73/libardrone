#include "include/libardrone.hpp"

using namespace robot;

void run( tesis::MessageServer* msgServer )
{
    RobotConfiguration robotConfig( "" );
    RobotConfig config = robotConfig.get();

    bool quit = false;
    bool land = false;
    bool is_visible = false;
    bool hover = true;

    double vx = 0, vy = 0, vz = 0;
    float yaw_set = 0, roll_set = 0, pitch_set = 0, altitude_set = 0;

    msgServer->announce( "robot/altitude" );

    PID_Y pid_y( config.PID.Yaw.Kp, config.PID.Yaw.Ki, config.PID.Yaw.Kd, config.PID.Yaw.Derivator, config.PID.Yaw.Integrator, config.PID.Yaw.P_limit, config.PID.Yaw.I_max, config.PID.Yaw.I_min, 0 );
    PID_RP pid_r( config.PID.Roll.Kp, config.PID.Roll.Ki, config.PID.Roll.Kd, config.PID.Roll.Derivator, config.PID.Roll.Integrator, config.PID.Roll.P_limit, config.PID.Roll.I_max, 0 );
    PID_RP pid_p( config.PID.Pitch.Kp, config.PID.Pitch.Ki, config.PID.Pitch.Kd, config.PID.Pitch.Derivator, config.PID.Pitch.Integrator, config.PID.Pitch.P_limit, config.PID.Pitch.I_max, 0 );
    PID_Z pid_z( config.PID.Altitude.Kp, config.PID.Altitude.Kd, config.PID.Altitude.P_limit, 0 );

    ARDrone robot( config.address.c_str() );

    robot.setFlatTrim();

    msgServer->publish( "robot/altitude", std::to_string( robot.getAltitude() ) );

    long elapsed_time = 0;

    while( !quit )
    {
        elapsed_time = std::stol( msgServer->get( "camera/elapsed_time", "0" ) );

        land = msgServer->get( "gui/action/land", "false" ).find( "false" ) == std::string::npos;

        if( !land )
        {
            is_visible = msgServer->get( "camera/robot_found", "true" ).find( "true" ) != std::string::npos;

            if( is_visible )
            {
                std::string robot_position_x = msgServer->get( "camera/robot_position/x" );
                std::string robot_position_y = msgServer->get( "camera/robot_position/y" );
                std::string robot_position_z = msgServer->get( "camera/robot_position/z" );

                Point position;
                position.x = std::stof( robot_position_x );
                position.y = std::stof( robot_position_y );
                position.z = std::stof( robot_position_z );

                std::string destination_x = msgServer->get( "camera/destination/x", "0" );
                std::string destination_y = msgServer->get( "camera/destination/y", "0" );

                Point destination;
                destination.x = std::stof( destination_x );
                destination.y = std::stof( destination_y );

                float setpoint_yaw = Util::get_angle_as_deg( position, destination, robot.getYaw() );
                pid_y.setPoint( setpoint_yaw );

                pid_r.setPoint( destination.x );
                pid_p.setPoint( destination.y );

                float angle = setpoint_yaw;
                angle -= robot.getYaw();
                angle = Util::normalize_angle( angle );

                float distance = Util::distance( position, destination );
                Point relative_position = Util::get_point( distance, angle );

                Velocity velocity;
                robot.getVelocity( &vx, &vy, &vz );
                velocity.x = vx;
                velocity.y = vy;
                velocity.z = vz;

                roll_set = pid_r.update( ( relative_position.x * -1 ), velocity.x, elapsed_time );
                pitch_set = pid_p.update( relative_position.y, velocity.y, elapsed_time );
                altitude_set = pid_z.update( robot.getAltitude(), 0 /* this value is not used */, elapsed_time );

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
                }
            }
            else
            {
                if( robot.onGround() == 0 ) robot.landing();
            }

            if( hover )
            {
                robot.move3D( 0, 0, altitude_set, yaw_set );
            }
            else
            {
                robot.move3D(roll_set, pitch_set, altitude_set, yaw_set);
            }
        }
        else
        {
            if( robot.onGround() == 0 ) robot.landing();
        }

        std::string finish_msg = msgServer->get( "gui/finish", "false" );
        std::istringstream( finish_msg ) >> std::boolalpha >> quit;
    }
    
    robot.close();
}
