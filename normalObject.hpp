#pragma once

#include "Eigen/Dense"
#include "displayObject.hpp"

using namespace Eigen;

class normalObject
{
    private:
        RowVector3d *surfaceNormal; //Storing a set of normals, where normal at index i is the normal for the face i
        int number_of_normals;
    public:
        void set_size_of_normals(int faceCounter);

        void generate_normals(displayObject object);

        RowVector3d get_normal(int index);
        
};