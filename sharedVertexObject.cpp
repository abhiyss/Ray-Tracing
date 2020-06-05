#pragma once

#include <vector>
#include <stdlib.h>
#include <iostream>
#include "Eigen/Dense"
#include "sharedVertexObject.hpp"
#include "normalObject.hpp"

using namespace Eigen;
using namespace std;
void sharedVertexObject::set_row_size(int value) // Using 2 dimensional vector instead of 2D array to save memory. Initializing the number of rows to access later
{
    sharedVertexObject::sharedVertices.resize(value);
}
void sharedVertexObject::printer() // Function to print the values inside the 2 dimensional vector
{
    for (int i = 0; i < sharedVertexObject::sharedVertices.size(); i++)
    {
        for (int j = 0; j < sharedVertexObject::sharedVertices[i].size(); j++)
        {
            cout << sharedVertexObject::sharedVertices[i][j] << " - ";
        }
        cout << endl;
    }
}
RowVector3d sharedVertexObject::normal_generator(double beta, double gamma, vector < int > faces, int faceNumber, normalObject normals) // Generates average normal in case of smoothing
{
    double N[3][3]; // Setting the value of normals at each vertex to zero to add upon later on
    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
            N[i][j] = 0;

    RowVector3d actualNormal = normals.get_normal(faceNumber); // The actual normal for a given face obtained from the file
    
    for(int i = 0; i < 3; i++) // For every vertex
    {
        int count = 0;
        for(int j = 0; j < sharedVertexObject::sharedVertices[faces[i]].size(); j++) // Go through every face shared by that vertex
        {
            RowVector3d normalInConsideration = normals.get_normal(sharedVertexObject::sharedVertices[faces[i]][j]); // Find normal of the face in consideration that is shared by the vertex
            if(normalInConsideration == actualNormal) // If the normal is the same, add it to the sum and increase count
            {
                count++;
                for(int k = 0; k < 3; k++)
                    N[i][k] += normalInConsideration(k);
            }
            else
            {
                double angle = (normalInConsideration.dot(actualNormal)/(normalInConsideration.norm()*actualNormal.norm()))*1.0; // Calculate the angle between the two normals
                if((angle > 0.923879 && angle < 1.0) || angle == 1) // If the angle is greater than 22.5 degress add that normal to the sum and increase the count
                {
                    for(int k = 0; k < 3; k++)
                        N[i][k] += normalInConsideration(k);
                    count++;
                }
            }
        }
        for(int j = 0; j < 3; j++) // FInd the average of the normal for that vertex
            N[i][j] = N[i][j]/count;
    }
    RowVector3d Na(N[0][0],N[0][1],N[0][2]), Nb(N[1][0],N[1][1],N[1][2]), Nc(N[2][0],N[2][1],N[2][2]);
    return ((1.0-beta-gamma)*Na+beta*Nb+gamma*Nc).normalized(); // Send back the average normal
}

void sharedVertexObject::push_value_in(int value1, int value2)
{
    sharedVertexObject::sharedVertices[value1].push_back(value2);
}
