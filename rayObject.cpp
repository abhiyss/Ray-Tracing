#include "Eigen/Dense"
#include "materialObject.hpp"
#include "sphereObject.hpp"
#include "displayObject.hpp"
#include "normalObject.hpp"
#include "lightObject.hpp"
#include "sharedVertexObject.hpp"
#include "rayObject.hpp"
#include <vector>

void rayObject::set_values(RowVector3d value1, RowVector3d value2, double value3)
{
    rayObject::L = value1;
    rayObject::D = value2;
    rayObject::best_tVal = value3;
}

bool rayObject::sphere_ray(sphereObject sphere)
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

bool rayObject::intersectionDetector(RowVector3d A, RowVector3d B, RowVector3d C, RowVector3d D, RowVector3d pixpt, double *betaO, double *gammaO)
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

bool rayObject::ray_trace(double pixels[3], Vector3d refatt, int level, vector <displayObject> object, vector < vector < materialObject > > material, vector < normalObject > normals, vector < sphereObject > sphere, vector < materialObject > sphereMaterial, vector < lightObject > light, RowVector3d ambient, vector < sharedVertexObject > sharedVertex)
{
    int minFace = -1;
    double beta, gamma;
    RowVector3d surfaceNormal;
    RowVector3d cameraDirection;
    double bestSphere = -1;
    
    for (int iterator = 0; iterator < object.size(); iterator++)
    {        
        for(int k = 0; k < object[iterator].get_faces_size(); k++)
        {
            RowVector3d A(object[iterator].get_vertex(object[iterator].get_face(k,0),0),object[iterator].get_vertex(object[iterator].get_face(k,0),1),object[iterator].get_vertex(object[iterator].get_face(k,0),2));
            RowVector3d B(object[iterator].get_vertex(object[iterator].get_face(k,1),0),object[iterator].get_vertex(object[iterator].get_face(k,1),1),object[iterator].get_vertex(object[iterator].get_face(k,1),2));
            RowVector3d C(object[iterator].get_vertex(object[iterator].get_face(k,2),0),object[iterator].get_vertex(object[iterator].get_face(k,2),1),object[iterator].get_vertex(object[iterator].get_face(k,2),2));
            
            if(this->D.dot(normals[iterator].get_normal(k)) < 0)
            {
                if(intersectionDetector(A,B,C,this->D,this->L, &beta, &gamma))
                {
                    minFace = k;
                    this->bestMaterial = material[iterator][object[iterator].get_faceMaterial(minFace)];
                    if(object[iterator].get_sharpFlag())
                        surfaceNormal = normals[iterator].get_normal(minFace);
                    else
                        surfaceNormal = sharedVertex[iterator].normal_generator(beta, gamma, object[iterator].get_face_vector(minFace), minFace, normals[iterator]);
                    
                    RowVector2d textureIntersectionPoint;
                    unsigned char *red, *green, *blue;
                    RowVector3d textureColor;
                    if(material[iterator][object[iterator].get_faceMaterial(minFace)].get_textureMapping() == true)
                    {
                        int imageWidth = material[iterator][object[iterator].get_faceMaterial(minFace)].get_textureImage().width();
                        int imageHeight = material[iterator][object[iterator].get_faceMaterial(minFace)].get_textureImage().height();
                        textureIntersectionPoint(0) = object[iterator].get_textureVertex(object[iterator].get_textureFaces(minFace,0),0);
                        textureIntersectionPoint(1) = object[iterator].get_textureVertex(object[iterator].get_textureFaces(minFace,0),1);
                        textureIntersectionPoint(0) += (beta * (object[iterator].get_textureVertex(object[iterator].get_textureFaces(minFace,1),0) - textureIntersectionPoint(0)) + gamma * (object[iterator].get_textureVertex(object[iterator].get_textureFaces(minFace,2),0) - textureIntersectionPoint(0)));
                        textureIntersectionPoint(1) += (beta * (object[iterator].get_textureVertex(object[iterator].get_textureFaces(minFace,1),1) - textureIntersectionPoint(1)) + gamma * (object[iterator].get_textureVertex(object[iterator].get_textureFaces(minFace,2),1) - textureIntersectionPoint(1)));
                        textureIntersectionPoint(0) *= imageWidth;
                        textureIntersectionPoint(1) *= imageHeight;
                        textureIntersectionPoint(1) = imageHeight - textureIntersectionPoint(1);
                        red = material[iterator][object[iterator].get_faceMaterial(minFace)].get_textureImage().data(textureIntersectionPoint(0),textureIntersectionPoint(1),0,0); 
                        green = material[iterator][object[iterator].get_faceMaterial(minFace)].get_textureImage().data(textureIntersectionPoint(0),textureIntersectionPoint(1),0,1); 
                        blue = material[iterator][object[iterator].get_faceMaterial(minFace)].get_textureImage().data(textureIntersectionPoint(0),textureIntersectionPoint(1),0,2); 
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
                for(int k = 0; k < object[iterator].get_faces_size(); k++)
                {
                    RowVector3d A(object[iterator].get_vertex(object[iterator].get_faces(k,0),0),object[iterator].get_vertex(object[iterator].get_faces(k,0),1),object[iterator].get_vertex(object[iterator].get_faces(k,0),2));
                    RowVector3d B(object[iterator].get_vertex(object[iterator].get_faces(k,1),0),object[iterator].get_vertex(object[iterator].get_faces(k,1),1),object[iterator].get_vertex(object[iterator].get_faces(k,1),2));
                    RowVector3d C(object[iterator].get_vertex(object[iterator].get_faces(k,2),0),object[iterator].get_vertex(object[iterator].get_faces(k,2),1),object[iterator].get_vertex(object[iterator].get_faces(k,2),2));
                    
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
