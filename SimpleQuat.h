#include "math.h"

class Quaternion{
    public:

    Quaternion();
    Quaternion( double inQuatMatrix[][4]);

    private:
    double _w;
    double _x;
    double _y;
    double _z;
    double quatMatrix[][4] = {_w,_x,_y,_z};


};
