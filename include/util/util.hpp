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
        static float get_angle_as_deg( Point opoint, Point dpoint, float deg );
        static float distance( Point opoint, Point dpoint );
        static Point get_point( int dist, float deg );
        static float normalize_angle(float angle);
    private:
        Util();
        ~Util();
};


}
#endif // UTIL_H
