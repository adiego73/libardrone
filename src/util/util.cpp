#include "util/util.hpp"

using namespace robot;

Util::Util()
{

}

Util::~Util()
{

}

float Util::rad_to_deg( float rad )
{
    return rad * 180.0 / M_PI;
}

float Util::deg_to_rad( float deg )
{
    return deg * M_PI / 180.0;
}

float Util::distance( Point pt, Point pt2 )
{
    pt.x -= pt2.x;
    pt.y -= pt2.y;
    return std::sqrt( std::abs( pt.x * pt.x + pt.y * pt.y ) );
}

float Util::get_angle_as_deg( Point pt, Point pt2, float deg )
{
    float angle = 0;

    pt.x -= pt2.x;
    pt.y -= pt2.y;

    if( pt.x != 0 )
    {
        angle = std::atan( std::abs( ( float )pt.y / ( float )pt.x ) );
        angle = Util::rad_to_deg( angle );

        if( pt.x > 0 )
        {
            if( pt.y < 0 )
                angle = -( angle + 90 );
            else
                angle = -( 90 - angle );
        }
        else if( pt.y < 0 )
            angle += 90;
        else
            angle = 90 - angle;
    }

    return angle;
}

Point Util::get_point( int dist, float deg )
{
    Point pt;
    float rad = Util::deg_to_rad( deg );

    pt.y = dist * std::sin( rad );
    pt.x = dist * std::cos( rad );
    return pt;
}
