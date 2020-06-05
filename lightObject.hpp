#pragma once
class lightObject
{
    private:
        double xLoc,yLoc,zLoc; //X,Y and Z coordinates for the light object
        int option; //Flag for Near or Infinite distance light #Considering changing to bool
        double rVal,gVal,bVal;
    public:
        void set_values(double value1, double value2, double value3, int value4, double value5, double value6, double value7);

        void set_xLoc(double value1);

        void set_yLoc(double value1);

        void set_zLoc(double value1);
        
        double get_rVal();

        double get_gVal();

        double get_bVal();
        
        int get_option();

        double get_xLoc();

        double get_yLoc();

        double get_zLoc();

};