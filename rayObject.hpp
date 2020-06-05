#pragma once

#include "Eigen/Dense"
#include "materialObject.hpp"
#include "sphereObject.hpp"
#include "displayObject.hpp"
#include "normalObject.hpp"
#include "lightObject.hpp"
#include "sharedVertexObject.hpp"
#include "rayObject.hpp"
#include <vector>

class rayObject
{
    private:
        RowVector3d L, D, intersectionPoint;
        double best_tVal;
        sphereObject best_sphere;
        materialObject bestMaterial;
    public:
        void set_values(RowVector3d value1, RowVector3d value2, double value3);

        bool sphere_ray(sphereObject sphere);

        bool intersectionDetector(RowVector3d A, RowVector3d B, RowVector3d C, RowVector3d D, RowVector3d pixpt, double *betaO, double *gammaO);

        bool ray_trace(double pixels[3], Vector3d refatt, int level, vector <displayObject> object, vector < vector < materialObject > > material, vector < normalObject > normals, vector < sphereObject > sphere, vector < materialObject > sphereMaterial, vector < lightObject > light, RowVector3d ambient, vector < sharedVertexObject > sharedVertex);
};