#include "Eigen/Dense"
#include <stdlib.h>
#include "modelObject.hpp"

using namespace Eigen;
using namespace std;

#define PI 3.14159265

void modelObject::set_values(double value1, double value2, double value3, double value4, double value5, double value6, double value7, double value8, string value9, string value10)
{
    modelObject::wx = value1;
    modelObject::wy = value2;
    modelObject::wz = value3;
    modelObject::theta = value4;
    modelObject::scale = value5;
    modelObject::tx = value6;
    modelObject::ty = value7;
    modelObject::tz = value8;
    if(!value9.compare("sharp"))
        modelObject::sharpFlag = true;
    else
        modelObject::sharpFlag = false;
    modelObject::objectFileName = value10;
}

Matrix4d modelObject::translation_Matrix_Generator() //Generates the translation matrix
{
    Matrix4d translationMatrixTemp;
    translationMatrixTemp << 1, 0, 0, tx,
                            0, 1, 0, ty,
                            0, 0, 1, tz,
                            0, 0, 0, 1;
    return translationMatrixTemp;
}

Matrix4d modelObject::scaling_Matrix_Generator() //Generates the scaling matrix
{
    Matrix4d scalingMatrixTemp;
    scalingMatrixTemp << scale, 0, 0, 0,
                        0, scale, 0, 0,
                        0, 0, scale, 0,
                        0, 0, 0, 1;
    return scalingMatrixTemp;
}

Matrix4d modelObject::rotation_Matrix_Generator() //Generates the rotation matrix
{
    RowVector3d w(wx, wy, wz); //Vector for the axis of rotation
    w.normalize();

    int minimumInW = INT_MAX, indexMinimumInW = -1; //Find the the minimum among X, Y, Z to get the vector orgonal to W
    for (int i = 0; i < 3; i++)
    {
        if(minimumInW >= abs(w(i)))
        {
            minimumInW = w(i);
            indexMinimumInW = i;
        }
    }

    RowVector3d wOrthogonalMatrix;
    for(int i = 0; i < 3; i++)
    {
        if (i != indexMinimumInW)
            wOrthogonalMatrix(i) = w(i);
        else
            wOrthogonalMatrix(i) = 1.0;
            
    }

    RowVector3d u;
    u = w.cross(wOrthogonalMatrix); //u is perpendixular to w
    u.normalize();

    RowVector3d v;
    v = w.cross(u); //v is perpendicular to both u and w

    Matrix3d rotation2D;
    rotation2D << u, v, w;

    Matrix4d rotation3D;

    rotation3D << rotation2D(0), rotation2D(3), rotation2D(6), 0,
                rotation2D(1), rotation2D(4), rotation2D(7), 0,
                rotation2D(2), rotation2D(5), rotation2D(8), 0,
                0, 0, 0, 1;

    Matrix4d rotation3DTranspose;

    rotation3DTranspose = rotation3D.transpose();
    
    double angle = theta*PI/180;
    Matrix4d rotationTheta;
    rotationTheta << cos(angle), -sin(angle), 0, 0,
                    sin(angle), cos(angle), 0, 0,
                    0, 0, 1, 0,
                    0, 0, 0, 1;

    return rotation3DTranspose * rotationTheta * rotation3D;
}

string modelObject::get_objectFileName()
{
    return modelObject::objectFileName;
}

bool modelObject::get_sharpFlag()
{
    return modelObject::sharpFlag;
}