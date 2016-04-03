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
    bool auto_control = false;
    bool aire_y_estabilizado = false;

    const long ROBOT_STATE_READY = -1;
    long lastRobotState = 0;
    long actualRobotState = 0;
    struct timeval start, end;
    long seconds, useconds, mtime = 0;

    int destinoId = -1;
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
    msgServer->announce( "robot/velocity/z" );
    msgServer->announce( "robot/onground" );
    msgServer->announce( "robot/state" );
    msgServer->announce( "robot/battery" );

    msgServer->announce( "robot/destino/dist/x" );
    msgServer->announce( "robot/destino/dist/y" );
    msgServer->announce( "robot/destino/dist/total" );
    msgServer->announce( "robot/destino/angulo" );
    msgServer->announce( "robot/destino/setpointyaw" );

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

    //  pid_z.setPoint( 700 );

    std::cout << "Bateria: " << robot.getBatteryPercentage() <<  std::endl;

    while( !quit )
    {
//         std::cout << "Bateria: " << robot.getBatteryPercentage() << " %" << std::endl;

        gettimeofday( &end, NULL );

        seconds = end.tv_sec - start.tv_sec;
        useconds = end.tv_usec - start.tv_usec;
        mtime = ( ( seconds ) * 1000 + useconds / 1000.0 ) + 0.5;

        if( mtime > 35 ) //0.035 segundos
        {
            gettimeofday( &start, NULL );

            elapsed_time = msgServer->getLong( "camera/elapsed_time", 0 );
            land = msgServer->getBool( "gui/action/land", false );

            Velocity velocity;
            robot.getVelocity( &vx, &vy, &vz );
            velocity.x = -vy / 0.001;
            velocity.y = vx / 0.001;
            velocity.z = -vz / 0.001;


            if( lastRobotState == 458752 && actualRobotState == 458753 ) //458752: estabilizando - 458753: estabilizado
            {
                aire_y_estabilizado = true;
                fixYaw = Util::rad_to_deg( -robot.getYaw() );
            }

            yawValue = Util::rad_to_deg( -robot.getYaw() ) - fixYaw;
            yawValue = Util::normalize_angle( yawValue );


            if( !land )
            {
                is_visible = msgServer->getBool( "camera/robot_found", false );
                auto_control = msgServer->getBool( "gui/action/autocontrol", false );

                // if the robot is visible and is flying
                if( auto_control &&  is_visible && robot.onGround() ==  0 && aire_y_estabilizado )
                {
                    Point position;
                    position.x = msgServer->getFloat( "camera/robot_position/x" );
                    position.y = msgServer->getFloat( "camera/robot_position/y" );
                    position.z = msgServer->getFloat( "camera/robot_position/z" );

                    Point destination;
                    destination.x = msgServer->getFloat( "camera/destination/x", 0 );
                    destination.y = msgServer->getFloat( "camera/destination/y", 0 );
                    destination.z = msgServer->getFloat( "camera/destination/z", 0 );

                    int newDestino = msgServer->getFloat( "camera/destination/id", 0 );
                    newDestino = destination.z;

                    float setpoint_yaw = Util::get_angle_as_deg( position, destination );

                    //********** Codigo q faltaba ************************

                    if( newDestino != destinoId )
                    {
                        destinoId = newDestino;
                        pid_p.setPoint( destination.y );
                        pid_r.setPoint( destination.x );
                        pid_z.setPoint( destination.z );
                    }
                    else
                    {
                        pid_p.updateSamePoint( destination.y );
                        pid_r.updateSamePoint( destination.x );
                    }

                    float ang = setpoint_yaw;
                    ang -= yawValue;
                    ang = Util::normalize_angle( ang );

                    float distance = Util::distance( position, destination );

                    altitude_set = pid_z.update( robot.getAltitude() * 1000, 0 /* this value is not used */, elapsed_time );

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

                        //calculo la distancia en x y en y segun la rotacion
                        Point relative_position = Util::get_point( distance, ang );
                        std::cout << "X: " << relative_position.x << " Y: " << relative_position.y << "Angulo: " << ang << std::endl;
                        roll_set = pid_r.update( relative_position.x , -velocity.x, elapsed_time ) ;
                        pitch_set = pid_p.update( relative_position.y, -velocity.y, elapsed_time );

                        msgServer->publish( "robot/destino/setpointyaw", std::to_string( setpoint_yaw ) );

                        msgServer->publish( "robot/destino/angulo", std::to_string( ang ) );
                        msgServer->publish( "robot/destino/dist/x", std::to_string( relative_position.x ) );
                        msgServer->publish( "robot/destino/dist/y", std::to_string( relative_position.y ) );
                        msgServer->publish( "robot/destino/dist/total", std::to_string( distance ) );

                        //    roll_set = pid_r.update( ( relative_position.x * -1 ), velocity.x, elapsed_time ) ;
                        //    pitch_set = pid_p.update( relative_position.y, velocity.y, elapsed_time );
                    }

                    msgServer->publish( "robot/pitch/kp", std::to_string( pid_p.getKp() ) );
                    msgServer->publish( "robot/pitch/ki", std::to_string( pid_p.getKi() ) );
                    msgServer->publish( "robot/pitch/kd", std::to_string( pid_p.getKd() ) );
                    msgServer->publish( "robot/pitch/set", std::to_string( pitch_set ) );

                    msgServer->publish( "robot/roll/kp", std::to_string( pid_r.getKp() ) );
                    msgServer->publish( "robot/roll/ki", std::to_string( pid_r.getKi() ) );
                    msgServer->publish( "robot/roll/kd", std::to_string( pid_r.getKd() ) );
                    msgServer->publish( "robot/roll/set", std::to_string( roll_set ) );

                    msgServer->publish( "robot/altitude/kp", std::to_string( pid_z.getKp() ) );
                    msgServer->publish( "robot/altitude/ki", std::to_string( pid_z.getKi() ) );
                    msgServer->publish( "robot/altitude/kd", std::to_string( pid_z.getKd() ) );
                    msgServer->publish( "robot/altitude/set", std::to_string( altitude_set ) );

                    msgServer->publish( "robot/yaw/set", std::to_string( yaw_set ) );

                }
                // if it is visible and is on ground, wait for the takeoff message.
                else if( is_visible && robot.onGround() ==  1 )
                {
                    // if the takeoff signal was gave
                    if( msgServer->getBool( "gui/action/takeoff", false ) )
                    {
                        robot.setFlatTrim();
                        aire_y_estabilizado = false;
                        robot.takeoff();
                        roll_set = 0;
                        pitch_set = 0;
                        yaw_set = 0;
                        altitude_set = 0;
                    }
                }
                // if not visible and is flying, land.
                else if( !is_visible && robot.onGround() ==  0 || msgServer->getBool( "routine/destination/last", false ) )
                {
                    fixYaw = 0;
                    aire_y_estabilizado = false;

                    // move down to 30cm before land
                    while( robot.getAltitude() >= 0.3f )
                        robot.move3D( 0, 0, -0.1, 0 );

                    // land
                    robot.landing();

                    roll_set = 0;
                    pitch_set = 0;
                    yaw_set = 0;
                    altitude_set = 0;
                }

                // when it should be over the checkpoint
                if( hover )
                {
                    // if the destination is on the floor, it should land
                    if( msgServer->getFloat( "camera/destination/z", 0 ) == 0 )
                    {
                        while( robot.getAltitude() >= 0.3f )
                            robot.move3D( 0, 0, -0.1, yaw_set );

                        robot.landing();
                    }
                    // else, it should stay.
                    else
                    {
                        robot.move3D( 0, 0, altitude_set, yaw_set );
                    }
                }
                else
                {
                    // if the robot is already on the floor, it should takeoff first
                    if( robot.getAltitude() <= 0.1 )
                    {
                        robot.takeoff();
                    }
                    
                    // if should move.
                    if( robot.onGround() == 0 ) robot.move3D( pitch_set / 0.2, -roll_set / 0.2, altitude_set, yaw_set );
                }
            }
            else
            {
                // if it should land.
                if( robot.onGround() == 0 )
                {
                    // move down to 30cm before land
                    while( robot.getAltitude() >= 0.3f )
                        robot.move3D( 0, 0, -0.1, 0 );

                    aire_y_estabilizado = false;
                    robot.landing();
                    roll_set = 0;
                    pitch_set = 0;
                    yaw_set = 0;
                    altitude_set = 0;
                }
            }

            msgServer->publish( "robot/altitude", std::to_string( robot.getAltitude() ) );
            msgServer->publish( "robot/pitch/value", std::to_string( Util::rad_to_deg( robot.getPitch() ) ) );
            msgServer->publish( "robot/roll/value", std::to_string( Util::rad_to_deg( robot.getRoll() ) ) );
            msgServer->publish( "robot/altitude/value", std::to_string( robot.getAltitude() ) );
            msgServer->publish( "robot/velocity/x", std::to_string( velocity.x ) );
            msgServer->publish( "robot/velocity/y", std::to_string( velocity.y ) );
            msgServer->publish( "robot/velocity/z", std::to_string( velocity.z ) );
            msgServer->publish( "robot/yaw/value", std::to_string( yawValue ) );
            msgServer->publish( "robot/onground", std::to_string( robot.onGround() ) );
            msgServer->publish( "robot/state", std::to_string( robot.getNavdataState() ) );
            msgServer->publish( "robot/battery", std::to_string( robot.getBatteryPercentage() ) );

            lastRobotState = actualRobotState ;
            actualRobotState = robot.getNavdataState();

            quit = msgServer->getBool( "gui/finish", false );
        }

        usleep( 500 ); //0,035 segundos//0,010
    }

    robot.close();
}
