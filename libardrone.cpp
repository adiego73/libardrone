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
    long lastRobotState = 0;
    long actualRobotState = 0;
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
    msgServer->announce( "robot/onground" );
    msgServer->announce( "robot/state" );
    msgServer->announce( "robot/battery" );
    
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

    pid_z.setPoint( 700 );

    std::cout << "Bateria: " << robot.getBatteryPercentage() <<  std::endl;

    while( !quit )
    {
//         std::cout << "Bateria: " << robot.getBatteryPercentage() << " %" << std::endl;
        elapsed_time = msgServer->getLong( "camera/elapsed_time", 0 );

        land = msgServer->getBool( "gui/action/land", false );
        
	Velocity velocity;
	robot.getVelocity( &vx, &vy, &vz );
	velocity.x = vx / 0.001;
	velocity.y = -vy / 0.001;
	velocity.z = -vz / 0.001;
	
	
	if(lastRobotState == 458752 && actualRobotState == 458753)
	    fixYaw = Util::rad_to_deg( -robot.getYaw());
	  
	yawValue = Util::rad_to_deg( -robot.getYaw()) - fixYaw;
        yawValue = Util::normalize_angle(yawValue);

	
	if( !land )
        {
            is_visible = msgServer->getBool( "camera/robot_found", false );
	    auto_control = msgServer->getBool( "gui/action/autocontrol", false );

            // if the robot is visible and is flying
            if(auto_control &&  is_visible && robot.onGround() ==  0 )
            {
                
                
                
                
		
		Point position;
		position.x = msgServer->getFloat( "camera/robot_position/x");
		position.y = msgServer->getFloat( "camera/robot_position/y");
                position.z = msgServer->getFloat( "camera/robot_position/z");

                Point destination;
                destination.x = msgServer->getFloat("camera/destination/x", 0);
                destination.y = msgServer->getFloat("camera/destination/y", 0);
		destination.z = msgServer->getFloat("camera/destination/z", 0);
		
		int newDestino = msgServer->getFloat("camera/destination/id", 0);
		newDestino = destination.z;
                
                float setpoint_yaw = Util::get_angle_as_deg( position, destination, yawValue );
		
		//********** Codigo q faltaba ************************
		
		
	//	printf("set yaw: %.2f", set_point_yaw);

		if(newDestino   != destinoId)
		{
			destinoId = newDestino; 
			pid_p.setPoint(destination.y);
			pid_r.setPoint(destination.x);

		}/*no lo estamos usando
		if(destino.z != threadAttr->data.destino.z)
		{
			destino.z = threadAttr->data.destino.z;

			//	float dist = Util::distancia(pos, destino);
			z_pid->setPoint(destino.z);

		}*/

		Point relative = position;
		relative.x -= destination.x;
		relative.y -= destination.y;

		float ang = setpoint_yaw;
		ang -= yawValue;
		ang = Util::normalize_angle( ang);

		float distance = Util::distance(position, destination);

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
		    
		        roll_set = pid_r.update( relative_position.y , velocity.y, elapsed_time ) ;
			pitch_set = pid_p.update( -relative_position.x, velocity.x, elapsed_time );
                
		    
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
                bool takeoff = msgServer->getBool( "gui/action/takeoff", false );

                if( takeoff )
                {
		    robot.setFlatTrim();
                    robot.takeoff();
                }
            }
            // if not visible and is flying, land.
            else if(!is_visible && robot.onGround() ==  0)
            {
                fixYaw = 0;
		robot.landing();
	      
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
	
	msgServer->publish( "robot/altitude", std::to_string( robot.getAltitude() ) );
	msgServer->publish( "robot/pitch/value", std::to_string( Util::rad_to_deg( -robot.getPitch()) ) );
	msgServer->publish( "robot/roll/value", std::to_string( Util::rad_to_deg( robot.getRoll()) ) );
	msgServer->publish( "robot/altitude/value", std::to_string( robot.getAltitude() ) );
	msgServer->publish( "robot/velocity/x", std::to_string( velocity.x ) );
	msgServer->publish( "robot/velocity/y", std::to_string( velocity.y ) );
	msgServer->publish( "robot/yaw/value", std::to_string( yawValue ) );
	msgServer->publish( "robot/onground", std::to_string( robot.onGround() ) );
	msgServer->publish( "robot/state", std::to_string( robot.getNavdataState()) );
	msgServer->publish( "robot/battery", std::to_string( robot.getBatteryPercentage()) );
	
	lastRobotState = actualRobotState ; 
	actualRobotState = robot.getNavdataState();
	std::string squit = msgServer->get( "gui/finish", "false");
        quit = msgServer->getBool( "gui/finish", false);
    }

    robot.close();
}

