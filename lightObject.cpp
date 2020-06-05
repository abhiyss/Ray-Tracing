#include "lightObject.hpp"

void lightObject::set_values(double value1, double value2, double value3, int value4, double value5, double value6, double value7)
{
    lightObject::xLoc = value1;
    lightObject::yLoc = value2;
    lightObject::zLoc = value3;
    lightObject::option = value4;
    lightObject::rVal = value5;
    lightObject::gVal = value6;
    lightObject::bVal = value7;
}

void lightObject::set_xLoc(double value1)
{
    lightObject::xLoc = value1;
}

void lightObject::set_yLoc(double value1)
{
    lightObject::yLoc = value1;
}

void lightObject::set_zLoc(double value1)
{
    lightObject::zLoc = value1;
}

double lightObject::get_rVal()
{
    return lightObject::rVal;
}

double lightObject::get_gVal()
{
    return lightObject::gVal;
}

double lightObject::get_bVal()
{
    return lightObject::bVal;
}

int lightObject::get_option()
{
    return lightObject::option;
}

double lightObject::get_xLoc()
{
    return lightObject::xLoc;
}

double lightObject::get_yLoc()
{
    return lightObject::yLoc;
}

double lightObject::get_zLoc()
{
    return lightObject::zLoc;
}