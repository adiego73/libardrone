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

    double vx = 0, vy = 0, vz = 0;
    float yaw_set = 0, roll_set = 0, pitch_set = 0, altitude_set = 0;

    msgServer->announce( "robot/altitude" );

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

    msgServer->publish( "robot/altitude", robot.getAltitude() );

    long elapsed_time = 0;

    while( !quit )
    {
        elapsed_time = msgServer->get( "camera/elapsed_time", 0.0f );

        land = msgServer->get( "gui/action/land", false );

        if( !land )
        {
            is_visible = msgServer->get( "camera/robot_found", false );

            // if the robot is visible and is flying
            if( is_visible && robot.onGround() ==  0 )
            {
                Point position;
                position.x = msgServer->get( "camera/robot_position/x", -1.0f );
                position.y = msgServer->get( "camera/robot_position/y", -1.0f );
                position.z = msgServer->get( "camera/robot_position/z", -1.0f );

                Point destination;
                destination.x = msgServer->get( "camera/destination/x", 0.0f );
                destination.y = msgServer->get( "camera/destination/y", 0.0f );

                float setpoint_yaw = Util::get_angle_as_deg( position, destination, robot.getYaw() );
                pid_y.setPoint( setpoint_yaw );

                pid_r.setPoint( destination.x );
                pid_p.setPoint( destination.y );

                float angle = setpoint_yaw;
                angle -= robot.getYaw();
                angle = Util::normalize_angle( angle );

                float distance = Util::distance( position, destination );

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
                    
                    Velocity velocity;
                    robot.getVelocity( &vx, &vy, &vz );
                    velocity.x = vx;
                    velocity.y = vy;
                    velocity.z = vz;

                    Point relative_position = Util::get_point( distance, angle );
                    roll_set = pid_r.update( ( relative_position.x * -1 ), velocity.x, elapsed_time );
                    pitch_set = pid_p.update( relative_position.y, velocity.y, elapsed_time );
                }
            }
            // if it is visible and is on ground, wait for the takeoff message.
            else if( is_visible && robot.onGround() ==  1 )
            {
                bool takeoff = msgServer->get( "gui/action/takeoff", false );

                if( takeoff )
                {
                    robot.takeoff();
                }
            }
            else
            {
                // if not visible and is flying, land.
                if( robot.onGround() == 0 ) robot.landing();
            }

            if( hover )
            {
                // if hove is set on true, and is flying, hover.
                if( robot.onGround() == 0 ) robot.move3D( 0, 0, altitude_set, yaw_set );
            }
            else
            {
                // if should move.
                if( robot.onGround() == 0 ) robot.move3D( roll_set, pitch_set, altitude_set, yaw_set );
            }
        }
        else
        {
            // if it should land.
            if( robot.onGround() == 0 ) robot.landing();
        }

        quit = msgServer->get( "gui/finish", false );;
    }

    robot.close();
}
