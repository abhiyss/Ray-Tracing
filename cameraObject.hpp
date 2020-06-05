#include "Eigen/Dense" 
#include <vector>

using namespace Eigen;

class cameraObject
{
    private:
        RowVector3d eye, look, up, wCamera, vCamera, uCamera;
        double left,bottom,right,top;
        int width, height;
        //eye is the vector with the X,Y,and Z values of the location where the eye is present
        //look is the vector with the X,Y,and Z values of the location where to look at
        //up is the vector with the X,Y,and Z values of the vector pointing towards the upwards direction of the camera, to get the idea of the orientation
        //wCamera, vCamera, uCamera are the vectors that act as the basis axis
        //left,bottom,righ, and top provide the bounds of the image plane
        //width and height gives us the number of pixels along the width and th height of the image
        
    public:
        void set_eye_values(double value1, double value2, double value3);

        void set_look_values(double value1, double value2, double value3);

        void set_up_values(double value1, double value2, double value3);

        void set_bounds_values(double value1, double value2, double value3, double value4);

        void set_res_values(int value1, int value2);

        void compute_orientation();

        int get_height();

        int get_width();

        double get_left();

        double get_bottom();

        double get_right();

        double get_top();

        RowVector3d get_look();

        RowVector3d get_eye();

        RowVector3d get_up();

        RowVector3d get_wCamera();

        RowVector3d get_vCamera();

        RowVector3d get_uCamera();
};