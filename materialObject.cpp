#include "Eigen/Dense"
#include "CImg/CImg.h"
#include "materialObject.hpp"
#include <stdlib.h>
#include <iostream>
using namespace std;

using namespace Eigen;
using namespace cimg_library;

void materialObject::set_values_Ka(double value1, double value2, double value3)
{
    materialObject::Ka = Matrix3d::Zero();
    materialObject::Ka(0,0) = value1;
    materialObject::Ka(1,1) = value2;
    materialObject::Ka(2,2) = value3;
}
void materialObject::set_values_Kd(double value1, double value2, double value3)
{
    materialObject::Kd = Matrix3d::Zero();
    materialObject::Kd(0,0) = value1;
    materialObject::Kd(1,1) = value2;
    materialObject::Kd(2,2) = value3;
}
void materialObject::set_values_Kd_individual(double value1, int value2, int value3)
{
    materialObject::Kd(value2,value3) = value1;
}
void materialObject::set_values_Ks(double value1, double value2, double value3)
{
    materialObject::Ks = Matrix3d::Zero();
    materialObject::Ks(0,0) = value1;
    materialObject::Ks(1,1) = value2;
    materialObject::Ks(2,2) = value3;
}
void materialObject::set_values_Kr(double value1, double value2, double value3)
{
    materialObject::Kr = Matrix3d::Zero();
    materialObject::Kr(0,0) = value1;
    materialObject::Kr(1,1) = value2;
    materialObject::Kr(2,2) = value3;
}
void materialObject::set_values_Ko(double value1, double value2, double value3)
{
    materialObject::Ko = Matrix3d::Zero();
    materialObject::Ko(0,0) = value1;
    materialObject::Ko(1,1) = value2;
    materialObject::Ko(2,2) = value3;
}
void materialObject::set_value_eta(double value1)
{
    materialObject::eta = value1;
}
void materialObject::set_value_Ns(double value1)
{
    materialObject::Ns = value1;
}
void materialObject::set_textureImage(string value1)
{
    materialObject::textureImage.load_png(value1.c_str());
}

void materialObject::set_textureMapping(bool value1)
{
    materialObject::textureMapping = value1;
}

Matrix3d materialObject::get_Ka()
{
    return materialObject::Ka;
}

Matrix3d materialObject::get_Kr()
{
    return materialObject::Kr;
}

Matrix3d materialObject::get_Ks()
{
    return materialObject::Ks;
}

Matrix3d materialObject::get_Kd()
{
    return materialObject::Kd;
}

Matrix3d materialObject::get_Ko()
{
    return materialObject::Ko;
}

double materialObject::get_eta()
{
    return materialObject::eta;
}

double materialObject::get_Ns()
{
    return materialObject::Ns;
}

CImg<unsigned char> materialObject::get_textureImage()
{
    return materialObject::textureImage;
}

bool materialObject::get_textureMapping()
{
    return materialObject::textureMapping;
}