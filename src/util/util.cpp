#include "util/util.hpp"

using namespace robot;
Util::Util()
{

}

Util::~Util()
{

}

float Util::rad_to_deg ( float rad )
{
	return rad * 180.0 / M_PI;
}

float Util::deg_to_rad ( float deg )
{
	return deg * M_PI / 180.0;
}

float Util::distance ( Point opoint, Point dpoint )
{
	Point point;
	point.x = opoint.x - dpoint.x;
	point.y = opoint.y - dpoint.y;
	return std::sqrt ( std::abs ( point.x * point.x + point.y * point.y ) );
}

float Util::get_angle_as_deg ( Point opoint, Point dpoint )
{
	Point point;
	float angle = 0;

	point.x = opoint.x - dpoint.x;
	point.y = opoint.y - dpoint.y;

	if ( point.x != 0 ) {
		angle = std::atan ( std::abs ( ( float ) point.y / ( float ) point.x ) );
		angle = Util::rad_to_deg ( angle );

		if ( point.x > 0 ) {
			if ( point.y < 0 ) {
				angle = ( angle + 90 ) * -1;
			} else {
				angle = ( 90 - angle ) * -1;
			}
		} else {
			if ( point.y < 0 ) {
				angle += 90;
			} else {
				angle = 90 - angle;
			}
		}
	}

	return angle;
}

Point Util::get_point ( int dist, float deg )
{
	Point point;
	float rad = Util::deg_to_rad ( deg );

	point.x = dist * std::sin ( rad );
	point.y = dist * std::cos ( rad );

	return point;
}

float Util::normalize_angle ( float angle )
{
	if ( angle > 180 ) {
		angle -= 360;
	} else if ( angle < -180 ) {
		angle += 360;
	}

	return angle;
}
float Util::VelXY( Velocity vel){
	return sqrt(vel.x * vel.x + vel.y * vel.y);
		
}

