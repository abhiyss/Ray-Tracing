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
#include "CImg/CImg.h"
#include "materialObject.hpp"
#include "lightObject.hpp"
#include "displayObject.hpp"
#include "cameraObject.hpp"
#include "normalObject.hpp"
#include "sharedVertexObject.hpp"
#include "sphereObject.hpp"

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

class rayObject;

class modelObject
{
    public:
        double wx, wy, wz, theta, scale, tx, ty, tz; 
        bool sharpFlag;
        string objectFileName;
        //WX, WY, WZ provide the axis along which rotation is to be done.
        //Theta tells the angle to be which the object is to be rotated
        //Scale provides the uniform-scaling factor
        //TX, TY, TZ provide the values across respective basis axes by which translation is to be performed
        //sharpFlag tells whether a particular object should be smoothened or to be left with sharp edges
        //objectFileName provides the name of the file in which the vertices and the faces of the object are stored 
        void set_values(double value1, double value2, double value3, double value4, double value5, double value6, double value7, double value8, string value9, string value10)
        {
            wx = value1;
            wy = value2;
            wz = value3;
            theta = value4;
            scale = value5;
            tx = value6;
            ty = value7;
            tz = value8;
            if(!value9.compare("sharp"))
                sharpFlag = true;
            else
                sharpFlag = false;
            objectFileName = value10;
        }

        Matrix4d translation_Matrix_Generator() //Generates the translation matrix
        {
            Matrix4d translationMatrixTemp;
            translationMatrixTemp << 1, 0, 0, tx,
                                    0, 1, 0, ty,
                                    0, 0, 1, tz,
                                    0, 0, 0, 1;
            return translationMatrixTemp;
        }

        Matrix4d scaling_Matrix_Generator() //Generates the scaling matrix
        {
            Matrix4d scalingMatrixTemp;
            scalingMatrixTemp << scale, 0, 0, 0,
                                0, scale, 0, 0,
                                0, 0, scale, 0,
                                0, 0, 0, 1;
            return scalingMatrixTemp;
        }

        Matrix4d rotation_Matrix_Generator() //Generates the rotation matrix
        {
            RowVector3d w(wx, wy, wz); //Vector for the axis of rotation
            w.normalize();

            int minimumInW = INT_MAX, indexMinimumInW = -1; //Find the the minimum among X, Y, Z to get the vector orgonal to W
            for (int i = 0; i < 3; i++)
            {
                if(minimumInW >= abs(w(i)))
                {
                    minimumInW = w(i);
                    indexMinimumInW = i;
                }
            }

            RowVector3d wOrthogonalMatrix;
            for(int i = 0; i < 3; i++)
            {
                if (i != indexMinimumInW)
                    wOrthogonalMatrix(i) = w(i);
                else
                    wOrthogonalMatrix(i) = 1.0;
                    
            }

            RowVector3d u;
            u = w.cross(wOrthogonalMatrix); //u is perpendixular to w
            u.normalize();

            RowVector3d v;
            v = w.cross(u); //v is perpendicular to both u and w

            Matrix3d rotation2D;
            rotation2D << u, v, w;

            Matrix4d rotation3D;

            rotation3D << rotation2D(0), rotation2D(3), rotation2D(6), 0,
                        rotation2D(1), rotation2D(4), rotation2D(7), 0,
                        rotation2D(2), rotation2D(5), rotation2D(8), 0,
                        0, 0, 0, 1;

            Matrix4d rotation3DTranspose;

            rotation3DTranspose = rotation3D.transpose();
            
            double angle = theta*PI/180;
            Matrix4d rotationTheta;
            rotationTheta << cos(angle), -sin(angle), 0, 0,
                            sin(angle), cos(angle), 0, 0,
                            0, 0, 1, 0,
                            0, 0, 0, 1;

            return rotation3DTranspose * rotationTheta * rotation3D;
        }
};

class rayObject
{
    public:
        RowVector3d L, D, intersectionPoint;
        double best_tVal;
        sphereObject best_sphere;
        materialObject bestMaterial;
        void set_values(RowVector3d value1, RowVector3d value2, double value3)
        {
            L = value1;
            D = value2;
            best_tVal = value3;
        }
        bool sphere_ray(sphereObject sphere)
        {
            RowVector3d T = sphere.get_center() - L;
            double disc = pow(sphere.get_radius(),2) - (T.dot(T)-pow(T.dot(D),2));
            if (disc > 0)
            {
                double tVal = T.dot(D) - sqrt(disc);
                if (tVal > 0.00001 && tVal < best_tVal)
                {
                    best_tVal = tVal;
                    best_sphere = sphere;
                    intersectionPoint = L + best_tVal*D;
                    return true;
                }
            }
            return false;
        }
        bool intersectionDetector(RowVector3d A, RowVector3d B, RowVector3d C, RowVector3d D, RowVector3d pixpt, double *betaO, double *gammaO)
        {
            Matrix3d M_for_intersection;
            M_for_intersection << A-B,A-C,D;
            
            Matrix3d M_for_intersection_transpose;
            M_for_intersection_transpose << M_for_intersection.transpose();

            double determinant = M_for_intersection_transpose.determinant();
            if(!(abs(determinant) <= 0.000001))
            {        
                RowVector3d Y = A-pixpt;
                
                Matrix3d MatrixCopy1;
                MatrixCopy1 << M_for_intersection_transpose;
                for(int l = 0; l < 3; l++)
                    MatrixCopy1(l,0) = Y(l);
                    
                double determinant1 = MatrixCopy1.determinant(), beta = determinant1/determinant;
                if (beta >= 0)
                {
                    Matrix3d MatrixCopy2;
                    MatrixCopy2 << M_for_intersection_transpose;
                    for(int l = 0; l < 3; l++)
                        MatrixCopy2(l,1) = Y(l);
                    
                    double determinant2 = MatrixCopy2.determinant(), gamma = determinant2/determinant;
                    if (gamma >= 0 && (beta + gamma) <= 1)
                    {
                        Matrix3d MatrixCopy3;
                        MatrixCopy3 << M_for_intersection_transpose;
                        for(int l = 0; l < 3; l++)
                            MatrixCopy3(l,2) = Y(l);
                        double determinant3 = MatrixCopy3.determinant(),tVal = determinant3/determinant;
                        if (best_tVal > tVal && tVal > 0.0001)
                        {
                            *betaO = beta;
                            *gammaO = gamma;
                            best_tVal = tVal;
                            intersectionPoint = L + best_tVal * D;
                            return true;
                        }
                    } 
                }
            }
            return false;
        }
        bool ray_trace(double pixels[3], Vector3d refatt, int level, vector <displayObject> object, vector < vector < materialObject > > material, vector < normalObject > normals, vector < sphereObject > sphere, vector < materialObject > sphereMaterial, vector < lightObject > light, RowVector3d ambient, vector < sharedVertexObject > sharedVertex)
        {
            int minFace = -1;
            double beta, gamma;
            RowVector3d surfaceNormal;
            RowVector3d cameraDirection;
            double bestSphere = -1;
            
            for (int iterator = 0; iterator < object.size(); iterator++)
            {        
                for(int k = 0; k < object[iterator].faces.size(); k++)
                {
                    RowVector3d A(object[iterator].vertex[object[iterator].faces[k][0]][0],object[iterator].vertex[object[iterator].faces[k][0]][1],object[iterator].vertex[object[iterator].faces[k][0]][2]);
                    RowVector3d B(object[iterator].vertex[object[iterator].faces[k][1]][0],object[iterator].vertex[object[iterator].faces[k][1]][1],object[iterator].vertex[object[iterator].faces[k][1]][2]);
                    RowVector3d C(object[iterator].vertex[object[iterator].faces[k][2]][0],object[iterator].vertex[object[iterator].faces[k][2]][1],object[iterator].vertex[object[iterator].faces[k][2]][2]);
                    
                    if(this->D.dot(normals[iterator].get_normal(k)) < 0)
                    {
                        if(intersectionDetector(A,B,C,this->D,this->L, &beta, &gamma))
                        {
                            minFace = k;
                            this->bestMaterial = material[iterator][object[iterator].faceMaterial[minFace]];
                            if(object[iterator].sharpFlag)
                                surfaceNormal = normals[iterator].get_normal(minFace);
                            else
                                surfaceNormal = sharedVertex[iterator].normal_generator(beta, gamma, object[iterator].faces[minFace], minFace, normals[iterator]);
                            
                            RowVector2d textureIntersectionPoint;
                            unsigned char *red, *green, *blue;
                            RowVector3d textureColor;
                            if(material[iterator][object[iterator].faceMaterial[minFace]].get_textureMapping() == true)
                            {
                                int imageWidth = material[iterator][object[iterator].faceMaterial[minFace]].get_textureImage().width();
                                int imageHeight = material[iterator][object[iterator].faceMaterial[minFace]].get_textureImage().height();
                                textureIntersectionPoint(0) = object[iterator].textureVertex[object[iterator].textureFaces[minFace][0]][0];
                                textureIntersectionPoint(1) = object[iterator].textureVertex[object[iterator].textureFaces[minFace][0]][1];
                                textureIntersectionPoint(0) += (beta * (object[iterator].textureVertex[object[iterator].textureFaces[minFace][1]][0] - textureIntersectionPoint(0)) + gamma * (object[iterator].textureVertex[object[iterator].textureFaces[minFace][2]][0] - textureIntersectionPoint(0)));
                                textureIntersectionPoint(1) += (beta * (object[iterator].textureVertex[object[iterator].textureFaces[minFace][1]][1] - textureIntersectionPoint(1)) + gamma * (object[iterator].textureVertex[object[iterator].textureFaces[minFace][2]][1] - textureIntersectionPoint(1)));
                                textureIntersectionPoint(0) *= imageWidth;
                                textureIntersectionPoint(1) *= imageHeight;
                                textureIntersectionPoint(1) = imageHeight - textureIntersectionPoint(1);
                                red = material[iterator][object[iterator].faceMaterial[minFace]].get_textureImage().data(textureIntersectionPoint(0),textureIntersectionPoint(1),0,0); 
                                green = material[iterator][object[iterator].faceMaterial[minFace]].get_textureImage().data(textureIntersectionPoint(0),textureIntersectionPoint(1),0,1); 
                                blue = material[iterator][object[iterator].faceMaterial[minFace]].get_textureImage().data(textureIntersectionPoint(0),textureIntersectionPoint(1),0,2); 
                                textureColor(0) = (double)*red/255;
                                textureColor(1) = (double)*green/255;
                                textureColor(2) = (double)*blue/255;
                        
                                this->bestMaterial.set_values_Kd_individual(textureColor(0),0,0);
                                this->bestMaterial.set_values_Kd_individual(textureColor(1),1,1);
                                this->bestMaterial.set_values_Kd_individual(textureColor(2),2,2);
                            }
                        }
                    }
                }
            }
            
            for (int iterator = 0; iterator < sphere.size(); iterator++)
            {
                if(this->sphere_ray(sphere[iterator]))
                {
                    this->bestMaterial = sphereMaterial[iterator];
                    bestSphere = iterator;
                    surfaceNormal = (this->intersectionPoint - this->best_sphere.get_center());
                    surfaceNormal.normalize();
                }
            }

            if(this->best_tVal != DBL_MAX)
            {
                cameraDirection = this->L-this->intersectionPoint;
                cameraDirection.normalize();
                
                RowVector3d finalPixel;
                finalPixel = ambient * this->bestMaterial.get_Ka();
                
                for (int j = 0; j < light.size(); j++)
                {
                    RowVector3d lightDirection(light[j].get_xLoc(),light[j].get_yLoc(),light[j].get_zLoc());
                    RowVector3d LightColor(light[j].get_rVal(),light[j].get_gVal(),light[j].get_bVal());

                    lightDirection = lightDirection-this->intersectionPoint;
                    lightDirection.normalize();
                    
                    bool flag = false;
                    for (int iterator = 0; iterator < object.size(); iterator++)
                    {
                        rayObject tempRay;
                        tempRay.set_values(this->intersectionPoint,lightDirection,DBL_MAX);
                        for(int k = 0; k < object[iterator].faces.size(); k++)
                        {
                            RowVector3d A(object[iterator].vertex[object[iterator].faces[k][0]][0],object[iterator].vertex[object[iterator].faces[k][0]][1],object[iterator].vertex[object[iterator].faces[k][0]][2]);
                            RowVector3d B(object[iterator].vertex[object[iterator].faces[k][1]][0],object[iterator].vertex[object[iterator].faces[k][1]][1],object[iterator].vertex[object[iterator].faces[k][1]][2]);
                            RowVector3d C(object[iterator].vertex[object[iterator].faces[k][2]][0],object[iterator].vertex[object[iterator].faces[k][2]][1],object[iterator].vertex[object[iterator].faces[k][2]][2]);
                            
                            double temp1, temp2;
                            if(tempRay.intersectionDetector(A,B,C,lightDirection,this->intersectionPoint, &temp1, &temp2))
                                flag = true;
                        }
                    }
                    for (int iterator = 0; iterator < sphere.size(); iterator++)
                    {
                        rayObject tempRay;
                        tempRay.set_values(this->intersectionPoint,lightDirection,DBL_MAX);
                        if(tempRay.sphere_ray(sphere[iterator]))
                        {   
                            flag = true;
                        }
                    }
                    if(lightDirection.dot(surfaceNormal) > 0.0 && !flag)
                    {
                        finalPixel += LightColor * this->bestMaterial.get_Kd() * (lightDirection.dot(surfaceNormal));
                        RowVector3d R = (2*(lightDirection.dot(surfaceNormal))*surfaceNormal)-lightDirection;
                        R.normalize();
                        if(cameraDirection.dot(R) > 0)
                            finalPixel += LightColor * this->bestMaterial.get_Ks() * (pow(cameraDirection.dot(R),bestMaterial.get_Ns()));
                    }
                }
                for(int pixelIndex = 0; pixelIndex < 3; pixelIndex++)
                    if(bestSphere == -1)
                        pixels[pixelIndex] = finalPixel(pixelIndex) * refatt(pixelIndex);
                    else
                        pixels[pixelIndex] = finalPixel(pixelIndex) * refatt(pixelIndex) * this->bestMaterial.get_Ko()(pixelIndex,pixelIndex);
                if (level > 0)
                {
                    double flec[3] = {0.0,0.0,0.0};
                    RowVector3d Uinv;
                    Uinv = -1 * this->D;
                    RowVector3d refR;
                    refR = (2 * surfaceNormal.dot(Uinv) * surfaceNormal) - Uinv;
                    refR.normalize();
                    rayObject tempRay;
                    tempRay.set_values(this->intersectionPoint,refR,DBL_MAX);
                    Vector3d newRefatt = this->bestMaterial.get_Kr() * refatt;
                    tempRay.ray_trace(flec, newRefatt, (level - 1), object, material, normals, sphere, sphereMaterial, light, ambient, sharedVertex);
                    for(int i = 0; i < 3; i++)
                    {
                        if(bestSphere == -1)
                            pixels[i] += refatt(i)*flec[i];
                        else
                            pixels[i] += refatt(i)*flec[i]*this->bestMaterial.get_Ko()(i,i);
                    }
                }
                double sumKo = this->bestMaterial.get_Ko()(0,0) + this->bestMaterial.get_Ko()(1,1) + this->bestMaterial.get_Ko()(2,2);
                if (level > 0 && sumKo < 3.0 && bestSphere != -1)
                {
                    double throughPixels[3] = {0.0,0.0,0.0};
                    Vector3d newRefatt = this->bestMaterial.get_Kr() * refatt;
                    RowVector3d exitPoint,T2;
                    if(sphere[bestSphere].refraction_exit(-1 * this->D, this->intersectionPoint, bestMaterial.get_eta(), &exitPoint, &T2))
                    {   
                        rayObject tempRay;
                        tempRay.set_values(exitPoint, T2, DBL_MAX);
                        tempRay.ray_trace(throughPixels, newRefatt, (level - 1), object, material, normals, sphere, sphereMaterial, light, ambient, sharedVertex);
                        for(int i = 0; i < 3; i++)
                            pixels[i] += refatt(i)*throughPixels[i]*(1.0 - this->bestMaterial.get_Ko()(i,i));
                    }
                }
                return true;
            }
            return false;
        }
};

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
        
        ifstream objectfile (model[iterator].objectFileName);
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
                    
                    vector<double> verticesTemp(3);
                    verticesTemp[0] = final_Matrix(0);
                    verticesTemp[1] = final_Matrix(1);
                    verticesTemp[2] = final_Matrix(2);
                    object[iterator].vertex.push_back(verticesTemp);
                }
                else if (typeOfParameter == "vt")
                {
                    string temporary_coordinates[2];
                    vector<double> verticesTemp(2);
                    objectLine >> temporary_coordinates[0] >> temporary_coordinates[1];
                    verticesTemp[0] = stod(temporary_coordinates[0]);
                    verticesTemp[1] = stod(temporary_coordinates[1]);
                    object[iterator].textureVertex.push_back(verticesTemp);
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
                    
                    object[iterator].faces.push_back(facesTemp);
                    object[iterator].textureFaces.push_back(textureFacesTemp);
                    
                    if(materialIndex != -1)
                        object[iterator].faceMaterial.push_back(materialIndex);
                    else
                        object[iterator].faceMaterial.push_back(0);
                }
            }
        }
        object[iterator].sharpFlag = model[iterator].sharpFlag;
        
        sharedVertexObject sharedVertexTemp;
        sharedVertexTemp.set_row_size(object[iterator].vertex.size());
        for(int i = 0; i < object[iterator].faces.size(); i++)
        {
            for( int j = 0; j < 3; j++)
            {
                sharedVertexTemp.push_value_in(object[iterator].faces[i][j],i);
            }
        }
        sharedVertex.push_back(sharedVertexTemp);

        normalObject normalsTemp;
        normalsTemp.set_size_of_normals(object[iterator].faces.size());
        
        normalsTemp.generate_normals(object[iterator]);
        normals.push_back(normalsTemp);

        ifstream materialfile (object[iterator].material_file_name);
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
