#ifndef UTIL_H
#define UTIL_H

#include <cmath>

#include "structures.hpp"

namespace robot
{

class Util
{
    public:
        static float rad_to_deg( float rad );
        static float deg_to_rad( float deg );
        static float get_angle_as_deg( Point pt, Point pt2, float deg );
        static float distance( Point pt, Point pt2 );
        static Point get_point( int dist, float deg );
    private:
        Util();
        ~Util();
};


}
#endif // UTIL_H
