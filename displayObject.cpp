#pragma once

#include "displayObject.hpp"
#include <vector>

void displayObject::set_material_file_name(string value1)
{
    displayObject::material_file_name = value1;
}

void displayObject::vertex_push_back(double value1, double value2, double value3)
{
    vector<double> temp(3);
    temp[0] = value1;
    temp[1] = value2;
    temp[2] = value3;
    displayObject::vertex.push_back(temp);
}

void displayObject::textureVertex_push_back(double value1, double value2)
{
    vector<double> temp(2);
    temp[0] = value1;
    temp[1] = value2;
    displayObject::textureVertex.push_back(temp);
}

void displayObject::textureFaces_push_back(int value1, int value2, int value3)
{
    vector<int> temp(3);
    temp[0] = value1;
    temp[1] = value2;
    temp[2] = value3;
    displayObject::textureFaces.push_back(temp);
}

void displayObject::faces_push_back(int value1, int value2, int value3)
{
    vector<int> temp(3);
    temp[0] = value1;
    temp[1] = value2;
    temp[2] = value3;
    displayObject::faces.push_back(temp);
}

void displayObject::faceMaterial_push_back(int value1)
{
    displayObject::faceMaterial.push_back(value1);
}

void displayObject::set_sharpFlag(bool value1)
{
    displayObject::sharpFlag = value1;
}

int displayObject::get_vertex_size()
{
    return displayObject::vertex.size();
}

int displayObject::get_faces_size()
{
    return displayObject::faces.size();
}

int displayObject::get_face(int value1, int value2)
{
    return displayObject::faces[value1][value2];
}

string displayObject::get_material_file_name()
{
    return displayObject::material_file_name;
}

double displayObject::get_vertex(int value1, int value2)
{
    return displayObject::vertex[value1][value2];
}

int displayObject::get_faces(int value1, int value2)
{
    return displayObject::faces[value1][value2];
}

int displayObject::get_faceMaterial(int value1)
{
    return displayObject::faceMaterial[value1];
}

vector<int> displayObject::get_face_vector(int value1)
{
    return displayObject::faces[value1];
}

double displayObject::get_textureVertex(int value1, int value2)
{
    return displayObject::textureVertex[value1][value2];
}

int displayObject::get_textureFaces(int value1, int value2)
{
    return displayObject::textureFaces[value1][value2];
}

bool displayObject::get_sharpFlag()
{
    return displayObject::sharpFlag;
}