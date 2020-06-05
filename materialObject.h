class materialObject
{
    public:
        Matrix3d Ka,Kd,Ks,Kr,Ko; //Matrices for Ambient, Diffuse, Specular, and Reflection values for the RGB spectrum
        double eta;
        bool textureMapping;
        CImg<unsigned char> textureImage;
        double Ns; //Phong constant for a particular object
        void set_values_Ka(double value1, double value2, double value3)
        {
            Ka = Matrix3d::Zero();
            Ka(0,0) = value1;
            Ka(1,1) = value2;
            Ka(2,2) = value3;
        }
        void set_values_Kd(double value1, double value2, double value3)
        {
            Kd = Matrix3d::Zero();
            Kd(0,0) = value1;
            Kd(1,1) = value2;
            Kd(2,2) = value3;
        }
        void set_values_Ks(double value1, double value2, double value3)
        {
            Ks = Matrix3d::Zero();
            Ks(0,0) = value1;
            Ks(1,1) = value2;
            Ks(2,2) = value3;
        }
        void set_values_Kr(double value1, double value2, double value3)
        {
            Kr = Matrix3d::Zero();
            Kr(0,0) = value1;
            Kr(1,1) = value2;
            Kr(2,2) = value3;
        }
        void set_values_Ko(double value1, double value2, double value3)
        {
            Ko = Matrix3d::Zero();
            Ko(0,0) = value1;
            Ko(1,1) = value2;
            Ko(2,2) = value3;
        }
        void set_value_eta(double value1)
        {
            eta = value1;
        }
        void set_value_Ns(double value1)
        {
            Ns = value1;
        }
        void set_image(string value1)
        {
            textureImage.load_png(((value1).c_str()));
        }
};