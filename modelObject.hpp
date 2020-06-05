#pragma once 

#include "Eigen/Dense"
#include <stdlib.h>

using namespace Eigen;
using namespace std;

class modelObject
{
    private:
        double wx, wy, wz, theta, scale, tx, ty, tz; 
        bool sharpFlag;
        string objectFileName;
        //WX, WY, WZ provide the axis along which rotation is to be done.
        //Theta tells the angle to be which the object is to be rotated
        //Scale provides the uniform-scaling factor
        //TX, TY, TZ provide the values across respective basis axes by which translation is to be performed
        //sharpFlag tells whether a particular object should be smoothened or to be left with sharp edges
        //objectFileName provides the name of the file in which the vertices and the faces of the object are stored 

    public:
        void set_values(double value1, double value2, double value3, double value4, double value5, double value6, double value7, double value8, string value9, string value10);

        Matrix4d translation_Matrix_Generator(); //Generates the translation matrix

        Matrix4d scaling_Matrix_Generator(); //Generates the scaling matrix

        Matrix4d rotation_Matrix_Generator(); //Generates the rotation matrix

        string get_objectFileName();

        bool get_sharpFlag();
};