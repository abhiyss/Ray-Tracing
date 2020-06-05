#include <vector>
#include "Eigen/Dense" 
#include "cameraObject.hpp"

using namespace Eigen;

void cameraObject::set_eye_values(double value1, double value2, double value3)
{
    cameraObject::eye << value1,value2,value3;
}
void cameraObject::set_look_values(double value1, double value2, double value3)
{
    cameraObject::look << value1,value2,value3;
}
void cameraObject::set_up_values(double value1, double value2, double value3)
{
    cameraObject::up << value1,value2,value3;
}
void cameraObject::set_bounds_values(double value1, double value2, double value3, double value4)
{
    cameraObject::left = value1;
    cameraObject::bottom = value2;
    cameraObject::right = value3;
    cameraObject::top = value4;
}
void cameraObject::set_res_values(int value1, int value2)
{
    cameraObject::width = value1;
    cameraObject::height = value2;
}
void cameraObject::compute_orientation()
{
    cameraObject::wCamera = eye-look; //w axis is obtained by looking from the look at point towards the eye
    cameraObject::wCamera.normalize();
    cameraObject::uCamera = up.cross(wCamera); //u axis is obtained as the mutually perpendicular vector to w and the up vector 
    cameraObject::uCamera.normalize();
    cameraObject::vCamera = wCamera.cross(uCamera); //v axis is obtained as the mutually perpendicular vector to w and the u vector 
}

int cameraObject::get_height()
{
    return cameraObject::height;
}

int cameraObject::get_width()
{
    return cameraObject::width;
}

double cameraObject::get_left()
{
    return cameraObject::left;
}

double cameraObject::get_bottom()
{
    return cameraObject::bottom;
}

double cameraObject::get_right()
{
    return cameraObject::right;
}
        
double cameraObject::get_top()
{
    return cameraObject::top;
}

RowVector3d cameraObject::get_look()
{
    return cameraObject::look;   
}

RowVector3d cameraObject::get_eye()
{
    return cameraObject::eye;   
}

RowVector3d cameraObject::get_up()
{
    return cameraObject::up;   
}

RowVector3d cameraObject::get_wCamera()
{
    return cameraObject::wCamera;   
}

RowVector3d cameraObject::get_vCamera()
{
    return cameraObject::vCamera;   
}

RowVector3d cameraObject::get_uCamera()
{
    return cameraObject::uCamera;   
}