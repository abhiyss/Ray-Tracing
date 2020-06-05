#include "Eigen/Dense"
#include <vector>

using namespace Eigen;
using namespace std;

class displayObject
{
    public:
        vector < vector < double > > vertex; //Each row is a vertex, with 3 columns, one for each basis axes
        vector < vector < int > > faces; //Each row is a face, with 3 coulmns, one for every vertex of the traingular face
        
        vector < vector < double > > textureVertex;
        vector < vector < int > > textureFaces;
        
        vector < int > faceMaterial; //Indicator to which material to use in case of multiple materials in a file
        string material_file_name; //Name of the file in which the material can be found
        bool sharpFlag; //We have sharpFlag in both modelObject and displayObject as we do not pass the modelObject to ray_trace() but we pass displayObject
        
        void set_material_file_name(string value1);
};