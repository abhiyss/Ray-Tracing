#pragma once

#include "Eigen/Dense"
#include "normalObject.hpp"
#include <vector>

using namespace Eigen;
using namespace std;

class sharedVertexObject
{
    private:
        vector < vector < int > > sharedVertices; // 2 dimensional vector where values in the ith row are the faces that use the ith vertex

    public:
        void set_row_size(int value); // Using 2 dimensional vector instead of 2D array to save memory. Initializing the number of rows to access later
        
        void printer(); // Function to print the values inside the 2 dimensional vector

        RowVector3d normal_generator(double beta, double gamma, vector < int > faces, int faceNumber, normalObject normals); // Generates average normal in case of smoothing

        void push_value_in(int value1, int value2);
};
