#pragma once

#include "Eigen/Dense"

using namespace Eigen;

class sphereObject
{
    private:
    
        RowVector3d center; // X, Y, Locations of the center of the sphere
        double radius; // Radius of the sphere
        
    public:
    
        void set_values(RowVector3d value1, double value2);
        
        bool refraction_entry(RowVector3d W, RowVector3d Normal, double eta1, double eta2, RowVector3d *T);
        
        bool refraction_exit(RowVector3d W, RowVector3d point, double etaInside, RowVector3d *exitPoint, RowVector3d *T);

        RowVector3d get_center();

        double get_radius();
};