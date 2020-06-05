#pragma once

#include "Eigen/Dense"
#include "normalObject.hpp"
#include "displayObject.hpp"

using namespace Eigen;

void normalObject::set_size_of_normals(int faceCounter)
{
    normalObject::surfaceNormal=(RowVector3d*)malloc(faceCounter*sizeof(RowVector3d));
    normalObject::number_of_normals = faceCounter;
}

void normalObject::generate_normals(displayObject object)
{
    for(int i = 0; i < normalObject::number_of_normals; i++)
    {
        RowVector3d A(object.vertex[object.faces[i][0]][0],object.vertex[object.faces[i][0]][1],object.vertex[object.faces[i][0]][2]);
        RowVector3d B(object.vertex[object.faces[i][1]][0],object.vertex[object.faces[i][1]][1],object.vertex[object.faces[i][1]][2]);
        RowVector3d C(object.vertex[object.faces[i][2]][0],object.vertex[object.faces[i][2]][1],object.vertex[object.faces[i][2]][2]);
        surfaceNormal[i] = (A-B).cross(A-C); //Assuming the order in which the vertices for a face are given is cyclic with the surface normal pointing outwards
        surfaceNormal[i].normalize();
    }
}
RowVector3d normalObject::get_normal(int index)
{
    return normalObject::surfaceNormal[index];
}