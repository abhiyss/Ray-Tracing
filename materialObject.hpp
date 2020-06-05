#pragma once

#include "Eigen/Dense"
#define cimg_use_png
#include "CImg/CImg.h"

using namespace std;
using namespace Eigen;
using namespace cimg_library;

class materialObject
{
    private:
        Matrix3d Ka,Kd,Ks,Kr,Ko; //Matrices for Ambient, Diffuse, Specular, and Reflection values for the RGB spectrum
        double eta;
        double Ns; //Phong constant for a particular object
        bool textureMapping;
        CImg<unsigned char> textureImage;

    public:
        void set_values_Ka(double value1, double value2, double value3);
        
        void set_values_Kd(double value1, double value2, double value3);

        void set_values_Kd_individual(double value1, int value2, int value3);
        
        void set_values_Ks(double value1, double value2, double value3);
        
        void set_values_Kr(double value1, double value2, double value3);
        
        void set_values_Ko(double value1, double value2, double value3);
        
        void set_value_eta(double value1);
        
        void set_value_Ns(double value1);

        void set_textureImage(string value1);

        void set_textureMapping(bool value1);

        Matrix3d get_Ka(); 

        Matrix3d get_Kd();

        Matrix3d get_Ks(); 

        Matrix3d get_Kr();

        Matrix3d get_Ko();

        double get_eta();

        double get_Ns();

        CImg<unsigned char> get_textureImage();

        bool get_textureMapping();
             
};