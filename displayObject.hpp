#pragma once

#include "Eigen/Dense"
#include <vector>

using namespace Eigen;
using namespace std;

class displayObject
{
    private:
        vector < vector < double > > vertex; //Each row is a vertex, with 3 columns, one for each basis axes
        vector < vector < int > > faces; //Each row is a face, with 3 coulmns, one for every vertex of the traingular face
        
        vector < vector < double > > textureVertex;
        vector < vector < int > > textureFaces;
        
        vector < int > faceMaterial; //Indicator to which material to use in case of multiple materials in a file
        string material_file_name; //Name of the file in which the material can be found
        bool sharpFlag; //We have sharpFlag in both modelObject and displayObject as we do not pass the modelObject to ray_trace() but we pass displayObject
        
    public:
        void set_material_file_name(string value1);

        void vertex_push_back(double value1, double value2, double value3);

        void textureVertex_push_back(double value1, double value2);

        void textureFaces_push_back(int value1, int value2, int value3);

        void faces_push_back(int value1, int value2, int value3);

        void faceMaterial_push_back(int value1);

        void set_sharpFlag(bool);

        int get_vertex_size();

        int get_faces_size();

        int get_face(int value1, int value2);

        string get_material_file_name();

        double get_vertex(int value1, int value2);

        int get_faces(int value1, int value2);

        int get_faceMaterial(int value1);

        int get_textureFaces(int value1, int value2);

        double get_textureVertex(int value1, int value2);

        vector<int> get_face_vector(int value1);

        bool get_sharpFlag();
};