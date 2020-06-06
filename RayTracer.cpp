/*---------------------------------------------------------------------------*/
/*           Developed By: Sri Sagar Abhishek Yeluri                         */
/*                       Version : 5.2                                       */
/*                           CS 410                                          */
/*---------------------------------------------------------------------------*/

#pragma once

#include <iostream>
#include <fstream>
#include "Eigen/Dense"
#include <iomanip>
#include <sys/stat.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/param.h>
#include <cfloat>
#include <cstring>
#include <ctime>
#include <vector>
#include <sys/ioctl.h>
#include <thread>
#include <mutex>
#define cimg_use_png
#include "CImg/CImg.h"
#include "materialObject.hpp"
#include "lightObject.hpp"
#include "displayObject.hpp"
#include "cameraObject.hpp"
#include "normalObject.hpp"
#include "sharedVertexObject.hpp"
#include "sphereObject.hpp"
#include "modelObject.hpp"
#include "rayObject.hpp"

using namespace std;
using namespace Eigen;
using namespace cimg_library;

#define PI 3.14159265

#ifdef __APPLE__
    string command = "say -v \"Ava\" ";
    string command2 = "open ";
#elif __linux__
    string command = "echo ";
    string command2 = "xdg-open ";
#endif

static mutex barrier;

void progress_bar(int count, int pixelCount, int row, int col, time_t startTime)
{
    int barWidth = col/2;
    double progress = (count*10.00)/pixelCount;
    double pastprogress = ((count-1)*10.00)/pixelCount;;
    if ((((int)progress % 10) != ((int)pastprogress % 10)) || (count == pixelCount) || (count == 1))
    {
        int pos = barWidth * progress/10;
        for (int spaces = 0; spaces < (barWidth/2)-8; spaces++)
            cout<<" ";
        cout << "[";
        for (int progressBar = 0; progressBar < barWidth; progressBar++) 
        {
            if (progressBar < pos)
                if(count != pixelCount) 
                    cout << "\033[0;31m\u2588\033[0m";
                else
                    cout << "\033[0;32m\u2588\033[0m";
            else 
                cout << " ";
        }
        cout << "] " << int(progress * 10) <<" % complete.";
        cout <<"\r";
        if(count == pixelCount)
        {
            cout <<"\n\n";
            cout <<setw((col/2)+17)<<"Pixels Calculated. Writing image file\n";
            int endTime = time(0)-startTime*1.0;
            string time = to_string(endTime)+" seconds is the time taken to calculate";
            cout << "\n"<<setw((col/2)+time.length()/2-2)<<time;
        }
        cout.flush();
    }
}

void threading(int threadCount, int ***pixels, int recursionLevel, vector <displayObject> object, vector < vector < materialObject > > material, vector < normalObject > normals, vector < sphereObject > sphere, vector < materialObject > sphereMaterial, vector < lightObject > light, RowVector3d ambient, vector < sharedVertexObject > sharedVertex, double d, cameraObject camera, struct winsize w, time_t startTime, int *count, int affordableThreads)
{        
    for (int i = threadCount*(camera.get_height()/affordableThreads); i < (threadCount+1)*camera.get_height()/affordableThreads; i++)
    {
        for (int j = 0 ; j < camera.get_width(); j++)
        {
            double px = j/(camera.get_width()-1.0)*(camera.get_right()-camera.get_left())+camera.get_left();
            double py = i/(camera.get_height()-1.0)*(camera.get_bottom()-camera.get_top())+camera.get_top();
            RowVector3d pixpt = camera.get_eye() + (-d *camera.get_wCamera()) + (px * camera.get_uCamera()) + (py * camera.get_vCamera());
            
            RowVector3d D = pixpt-camera.get_eye();
            D.normalize();

            rayObject ray;
            ray.set_values(pixpt,D,DBL_MAX);
            double temp_pixels[]={0.0,0.0,0.0};
            Vector3d refatt(1.0,1.0,1.0);
            if(ray.ray_trace(temp_pixels, refatt, recursionLevel, object, material, normals, sphere, sphereMaterial, light, ambient, sharedVertex))
            {
                for(int pixelIndex = 0; pixelIndex < 3; pixelIndex++)
                {
                    if(temp_pixels[pixelIndex]<0)
                        pixels[i][j][pixelIndex] = (int)round(0);
                    else if(temp_pixels[pixelIndex]>1)
                        pixels[i][j][pixelIndex] = (int)round(255);
                    else
                        pixels[i][j][pixelIndex] = (int)round(temp_pixels[pixelIndex]*255);
                }
            }
            else
            {
                for (int pixelIndex = 0; pixelIndex < 3; pixelIndex++)
                {
                    pixels[i][j][pixelIndex] = 0;
                }
            }
            (*count)++;
            lock_guard<mutex> block_threads_until_finish_this_job(barrier);
            if(*count != camera.get_height()*camera.get_width())
                progress_bar(*count, camera.get_height()*camera.get_width(), w.ws_row, w.ws_col, startTime);
        }
    }
}

int main(int argc, char** argv)
{
    double d;
    int recursionLevel = 0; 
    string driver_file_name = argv[1], output_file_name = argv[2], material_file_name, line1, line2, line3, typeOfInput, typeOfParameter, typeOfMaterial;
    string output_string;
    time_t startTime = time(0);
    
    vector < lightObject > light;
    vector < vector < materialObject > > material;
    vector < materialObject > sphereMaterial;
    vector < sphereObject > sphere;
    vector < modelObject > model;
    vector < displayObject > object;
    vector < normalObject > normals;
    vector < sharedVertexObject > sharedVertex;
    cameraObject camera;
    ifstream driverfile (driver_file_name);
    RowVector3d ambient;
    while(!driverfile.eof())
    {        
        getline (driverfile,line1);
        istringstream driverline(line1);
        typeOfInput = "";
        driverline >> typeOfInput;
        if(typeOfInput == "eye")
        {
            string value1, value2, value3;
            driverline >> value1 >> value2 >> value3;
            camera.set_eye_values(stod(value1),stod(value2),stod(value3));
        }
        else if(typeOfInput == "look")
        {
            string value1, value2, value3;
            driverline >> value1 >> value2 >> value3;
            camera.set_look_values(stod(value1),stod(value2),stod(value3));
        }
        else if(typeOfInput == "up")
        {
            string value1, value2, value3;
            driverline >> value1 >> value2 >> value3;
            camera.set_up_values(stod(value1),stod(value2),stod(value3));
        }
        else if(typeOfInput == "d")
        {
            string value1;
            driverline >> value1;
            d = stod(value1);
        }
        else if(typeOfInput == "bounds")
        {
            string value1, value2, value3, value4;
            driverline >> value1 >> value2 >> value3 >> value4;
            camera.set_bounds_values(stod(value1), stod(value2), stod(value3), stod(value4));
        }
        else if(typeOfInput == "res")
        {
            string value1, value2;
            driverline >> value1 >> value2;
            camera.set_res_values(stoi(value1),stoi(value2));
        }
        else if(typeOfInput == "ambient")
        {
            string value1, value2, value3;
            driverline >> value1 >> value2 >> value3;
            ambient << stod(value1),stod(value2),stod(value3);
        }

        else if(typeOfInput == "light")
        {
            string value1, value2, value3, value4, value5, value6, value7;
            driverline >> value1 >> value2 >> value3 >> value4 >> value5 >> value6 >> value7;
            lightObject lightTemp;
            lightTemp.set_values(stod(value1), stod(value2), stod(value3), stoi(value4), stod(value5), stod(value6), stod(value7));
            if(lightTemp.get_option() == 0)
            {
                lightTemp.set_xLoc(camera.get_look()(0) + INT_MAX*lightTemp.get_xLoc());
                lightTemp.set_yLoc(camera.get_look()(1) + INT_MAX*lightTemp.get_yLoc());
                lightTemp.set_zLoc(camera.get_look()(2) + INT_MAX*lightTemp.get_zLoc());
            }
            light.push_back(lightTemp);
        }
        else if(typeOfInput == "recursionLevel")
        {
            string value1;
            driverline >> value1;
            recursionLevel = stoi(value1);
        }
        else if(typeOfInput == "model")
        {
            string a_wx, a_wy, a_wz, a_theta, a_scale, a_tx, a_ty, a_tz, flag, objectFileName;
            driverline >> a_wx >> a_wy >> a_wz >> a_theta >> a_scale >> a_tx >> a_ty >> a_tz >> flag >> objectFileName;
            modelObject modelTemp;
            modelTemp.set_values(stod(a_wx),stod(a_wy),stod(a_wz),stod(a_theta),stod(a_scale),stod(a_tx),stod(a_ty),stod(a_tz), flag, objectFileName);
            model.push_back(modelTemp);
        }
        else if(typeOfInput == "sphere")
        {
            string value1, value2, value3, value4, value5, value6, value7, value8, value9, value10, value11, value12, value13, value14, value15, value16, value17, value18, value19, value20;
            driverline >> value1 >> value2 >> value3 >> value4 >> value5 >> value6 >> value7 >> value8 >> value9 >> value10 >> value11 >> value12 >> value13 >> value14 >> value15 >> value16 >> value17 >> value18 >> value19 >> value20;
            RowVector3d center(stod(value1),stod(value2),stod(value3));
            materialObject sphereMaterialTemp;
            sphereMaterialTemp.set_values_Ka(stod(value5), stod(value6), stod(value7));
            sphereMaterialTemp.set_values_Kd(stod(value8), stod(value9), stod(value10));
            sphereMaterialTemp.set_values_Ks(stod(value11), stod(value12), stod(value13));
            sphereMaterialTemp.set_values_Kr(stod(value14), stod(value15), stod(value16));
            sphereMaterialTemp.set_values_Ko(stod(value17), stod(value18), stod(value19));
            sphereMaterialTemp.set_value_eta(stod(value20));
            sphereMaterialTemp.set_value_Ns(16);
            sphereMaterial.push_back(sphereMaterialTemp);
            sphereObject sphereTemp;
            sphereTemp.set_values(center,stod(value4));
            sphere.push_back(sphereTemp);
        }
    }
    
    camera.compute_orientation();
    for(int iterator = 0; iterator < model.size(); iterator++)
    {
        int materialIndex = -1;
        vector < string > materialName;
        
        displayObject objectTemp;
        object.push_back(objectTemp);
        
        MatrixXd object_Matrix(4,1);
        MatrixXd final_Matrix(4,1);

        Matrix4d translationMatrix;
        translationMatrix = model[iterator].translation_Matrix_Generator();

        Matrix4d scalingMatrix;
        scalingMatrix = model[iterator].scaling_Matrix_Generator();
        Matrix4d rotation_Matrix;
        rotation_Matrix = model[iterator].rotation_Matrix_Generator();
        Matrix4d transformation_Matrix = translationMatrix * scalingMatrix * rotation_Matrix;
        
        ifstream objectfile (model[iterator].get_objectFileName());
        {
            while(!objectfile.eof())
            {
                getline (objectfile,line2);
                istringstream objectLine(line2);
                typeOfParameter = "";
                objectLine >> typeOfParameter;
                if(typeOfParameter == "mtllib")
                {
                    objectLine >> material_file_name;
                    object[iterator].set_material_file_name(material_file_name);
                }
                else if (typeOfParameter == "v")
                {
                    string temporary_coordinates[3];
                    objectLine >> temporary_coordinates[0] >> temporary_coordinates[1] >> temporary_coordinates[2];
                    object_Matrix(0) = stod(temporary_coordinates[0]);
                    object_Matrix(1) = stod(temporary_coordinates[1]);
                    object_Matrix(2) = stod(temporary_coordinates[2]);
                    object_Matrix(3) = 1;
                    
                    final_Matrix = (transformation_Matrix * object_Matrix).transpose();
                    
                    object[iterator].vertex_push_back(final_Matrix(0),final_Matrix(1),final_Matrix(2));
                }
                else if (typeOfParameter == "vt")
                {
                    string temporary_coordinates[2];
                    double temp1, temp2;
                    objectLine >> temporary_coordinates[0] >> temporary_coordinates[1];
                    temp1 = stod(temporary_coordinates[0]);
                    temp2 = stod(temporary_coordinates[1]);
                    object[iterator].textureVertex_push_back(temp1,temp2);
                }
                else if (typeOfParameter == "usemtl")
                {
                    string materialNameTemp;
                    objectLine >> materialNameTemp;
                    materialName.push_back(materialNameTemp);
                    materialIndex++;
                }
                else if (typeOfParameter == "f")
                {  
                    vector<int> facesTemp(3), textureFacesTemp(3);
                    for (int i = 0; i < 3; i++)
                    {
                        string temporary_face;
                        objectLine >> temporary_face;
                        string temp = "";
                        int ender;
                        for(int j = 0; j < temporary_face.length(); j++)
                        {
                            if(temporary_face.at(j) != '/')
                                temp = temp + temporary_face.at(j);
                            else
                            {
                                ender = j;
                                break;
                            }
                        }
                        facesTemp[i] = stoi(temp)-1;
                        
                        while(temporary_face.at(ender) == '/')
                            ender++;
                        string tempTexture = "";
                        for(int j = ender; j < temporary_face.length(); j++)
                        {
                            if(temporary_face.at(j) != '/')
                                tempTexture = tempTexture + temporary_face.at(j);
                            else
                                break;
                        }
                        textureFacesTemp[i] = stoi(tempTexture)-1;
                    }
                    
                    object[iterator].faces_push_back(facesTemp[0],facesTemp[1],facesTemp[2]);
                    object[iterator].textureFaces_push_back(textureFacesTemp[0],textureFacesTemp[1],textureFacesTemp[2]);
                    
                    if(materialIndex != -1)
                        object[iterator].faceMaterial_push_back(materialIndex);
                    else
                        object[iterator].faceMaterial_push_back(0);
                }
            }
        }
        object[iterator].set_sharpFlag(model[iterator].get_sharpFlag());


        sharedVertexObject sharedVertexTemp;
        sharedVertexTemp.set_row_size(object[iterator].get_vertex_size());
        for(int i = 0; i < object[iterator].get_faces_size(); i++)
        {
            for( int j = 0; j < 3; j++)
            {
                sharedVertexTemp.push_value_in(object[iterator].get_face(i,j),i);
            }
        }
        sharedVertex.push_back(sharedVertexTemp);

        normalObject normalsTemp;
        normalsTemp.set_size_of_normals(object[iterator].get_faces_size());
        
        normalsTemp.generate_normals(object[iterator]);
        normals.push_back(normalsTemp);

        ifstream materialfile (object[iterator].get_material_file_name());
        {
            vector <materialObject > materialTemp(1);
            if (materialIndex != -1)
                materialTemp.resize(materialName.size());
            
            int materialValueIndex = 0;
            
            for(int materialIndex = 0; materialIndex < materialName.size(); materialIndex++)
            {
                materialTemp[materialIndex].set_values_Kr(1.0,1.0,1.0);
                materialTemp[materialIndex].set_values_Ko(1.0,1.0,1.0);
                materialTemp[materialIndex].set_textureMapping(false);
            }

            while(!materialfile.eof())
            {
                typeOfMaterial = "";
                getline (materialfile,line3);
                istringstream materialLine(line3);
                materialLine >> typeOfMaterial;
                
                if(materialIndex != -1 && typeOfMaterial == "newmtl")
                {
                    string materialNameTemp;
                    materialLine >> materialNameTemp;
                    for(int materialIndex = 0; materialIndex < materialName.size(); materialIndex++)
                    {
                        if( !materialName[materialIndex].compare(materialNameTemp))
                        {
                            materialValueIndex = materialIndex;
                            break;
                        }
                    }
                }
                
                else if(typeOfMaterial == "map_Kd")
                {
                    string value1;
                    materialLine >> value1;
                    materialTemp[materialValueIndex].set_textureImage(value1);
                    //materialTemp[materialValueIndex].set_textureImage("/Users/abhishek/Documents/GitHub/Ray-Tracing/Vision.png");
                    materialTemp[materialValueIndex].set_textureMapping(true);
                }
                else if(typeOfMaterial == "Ka")
                {
                    string value1, value2, value3;
                    materialLine >> value1 >> value2 >> value3;
                    materialTemp[materialValueIndex].set_values_Ka(stod(value1),stod(value2),stod(value3));
                }
                else if(typeOfMaterial == "Kd")
                {
                    string value1, value2, value3;
                    materialLine >> value1 >> value2 >> value3;
                    materialTemp[materialValueIndex].set_values_Kd(stod(value1),stod(value2),stod(value3));
                }
                else if(typeOfMaterial == "Ks")
                {
                    string value1, value2, value3;
                    materialLine >> value1 >> value2 >> value3;
                    materialTemp[materialValueIndex].set_values_Ks(stod(value1),stod(value2),stod(value3));
                }
                else if(typeOfMaterial == "Kr")
                {
                    string value1, value2, value3;
                    materialLine >> value1 >> value2 >> value3;
                    materialTemp[materialValueIndex].set_values_Kr(stod(value1),stod(value2),stod(value3));
                }
                else if(typeOfMaterial == "Ko")
                {
                    string value1, value2, value3;
                    materialLine >> value1 >> value2 >> value3;
                    materialTemp[materialValueIndex].set_values_Ko(stod(value1),stod(value2),stod(value3));
                }
                else if(typeOfMaterial == "Ns")
                {
                    string value1;
                    materialLine >> value1;
                    materialTemp[materialValueIndex].set_value_Ns(stod(value1));
                }
            }
            material.push_back(materialTemp);
        }
    }
    

    int ***pixels = (int ***)malloc(camera.get_height()*(sizeof(int**)));
    for (int i = 0; i < camera.get_height(); i++)
    {
        pixels[i] = (int **) malloc(camera.get_width()*sizeof(int *));
        for (int j = 0 ; j < camera.get_width(); j++)
        {
            pixels[i][j] = (int *)malloc(3*sizeof(int));
        }

    }
    
    system("clear");
    for(int newLine = 0; newLine < 27; newLine++)
        cout<<"\n";

    struct winsize w;
    ioctl(STDOUT_FILENO, TIOCGWINSZ, &w);
    
    cout<<setw((w.ws_col/2)+19)<<"Beginning the image rendering process\n\n\n";

    int count = 0;
    int affordableThreads = camera.get_height()/64;
    thread t[affordableThreads];
    for(int threadCount = 0; threadCount < affordableThreads; threadCount++)
    {
        t[threadCount] = thread(threading, threadCount, pixels, recursionLevel, object, material, normals, sphere, sphereMaterial, light, ambient, sharedVertex, d, camera, w, startTime, &count, affordableThreads);
    }
    for(int threadCount = 0; threadCount < affordableThreads; threadCount++)
    {
        t[threadCount].join();
    }
    
    time_t writeTime = time(0);
    
    progress_bar(camera.get_height()*camera.get_width(), camera.get_height()*camera.get_width(), w.ws_row, w.ws_col, startTime);
    ofstream output_file;
    output_file.open (output_file_name);
    output_file << "P3\n"<<camera.get_width()<<" "<<camera.get_height()<<" "<<"255\n";
    
    for (int i = 0; i < camera.get_height(); i++)
    {
        output_string = "";
        for (int j = 0 ; j < camera.get_width(); j++)
        {
            stringstream temporary_output_string;
            temporary_output_string << pixels[i][j][0] << " " << pixels[i][j][1] << " " << pixels[i][j][2] << " #"<<i<<"-"<<j<<"\n";
            output_string += temporary_output_string.str();
        }
        output_string += '\n';
        output_file << output_string;
    }
    
    int endTime = time(0)-writeTime*1.0;
    string time = to_string(endTime)+" seconds is the time taken to write the file";
    cout << "\n\n"<<setw((w.ws_col/2)+time.length()/2-2)<<time;
            
    cout << "\n\n"<<setw((w.ws_col/2)+20)<<"Image rendering complete. Opening image\n\n\n";
    for(int newLine = 0; newLine < (w.ws_row/2)-9; newLine++)
        cout<<"\n";
    command2 += argv[2];
    command +=  "\"Image rendering complete. Opening image\"";
    if(strcmp(command.c_str(),"echo"))
        system(command.c_str());
    system(command2.c_str());
}
