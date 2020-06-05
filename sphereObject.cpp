#pragma once

#include "Eigen/Dense"
#include "sphereObject.hpp"

using namespace Eigen;

void sphereObject::set_values(RowVector3d value1, double value2)
{
    sphereObject::center = value1;
    sphereObject::radius = value2;
}
        
bool sphereObject::refraction_entry(RowVector3d W, RowVector3d Normal, double eta1, double eta2, RowVector3d *T)
{
    double etar = eta1/eta2, a = -1 * etar, wn = W.dot(Normal), radsq = (pow(etar,2)*(pow(wn,2)-1))+1;
    if (radsq < 0.0)
        return false;
    else
    {
        *T = a * W + ((etar*wn)-sqrt(radsq)) * Normal;
        return true;
    }
}

bool sphereObject::refraction_exit(RowVector3d W, RowVector3d point, double etaInside, RowVector3d *exitPoint, RowVector3d *T)
{
    RowVector3d T1;
    if(refraction_entry(W, (point-this->sphereObject::center).normalized(), 1.0, etaInside, &T1))
    {
        *exitPoint = point + 2 * T1.dot(this->sphereObject::center - point) * T1;
        RowVector3d NormalIn(this->sphereObject::center);
        NormalIn = NormalIn - *exitPoint;
        refraction_entry((-1 * T1), NormalIn.normalized(), etaInside, 1.0, T);
        return true;
    }
    else
        return false;
}

RowVector3d sphereObject::get_center()
{
    return sphereObject::center;
}

double sphereObject::get_radius()
{
    return sphereObject::radius;
}

