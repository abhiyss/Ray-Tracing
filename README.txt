# Ray-Tracing
/*------------------------------------------------------------------------------*/
/*              Developed By: Sri Sagar Abhishek Yeluri                         */
/*                            Version : 5.2.2                                   */
/* Recursive Raytracer with Reflections, Refractions, Shadows, Texture Mapping  */
/*------------------------------------------------------------------------------*/
This is the solution i implemented for Recursive Raytracer.

The following files can be found in the repository.

1. Raytracer.cpp                   - The coordinator program that takes in inputs, creates necessary objects, and keeps the progress bar on the screen updated.

2. cameraObject.cpp and .hpp       - Program and header file that deals with the camera class and describe the view port of the camera using which the image will be taken.

3. lightObject.cpp and .hpp        - Program and header file that deals with each light source in the given environment.

4. modelObject.cpp and .hpp        - Program and header file that deals with the characterstics of each non-sphere object in the space, such as position, rotation, scaling, etc.

5. sphereObject.cpp and .hpp       - Program and header file that deals with the characteristics of sphere objects in the space. Different from modelObject as this has no vertices or faces, but just a radius and position, thereby increasing the speed of computation.

6. displayObject.cpp and .hpp      - Program and header file that deals with the faces, vertices, which material to use for each, etc. This is used for modelObject objects.

7. materialObject.cpp and .hpp     - Program and header file that deals with each material value such as Ks, Kd, texture mapping, etc.

8. normalObject.cpp and .hpp       - Program and header file that deals with normals from each face and their calculation.

9. sharedVertexObject.cpp and .hpp - Program and header file that deals with faces that share vertices. Used for surface smoothening.

10. rayObject.cpp and .hpp         - Program and header file that deals with rays that come from each pixel in the viewport and how they may or may not strike an object, and be reflected or refracted.


To compile the files use the command:

	$ make

To remove the executable run the command:

	$ make clean

After compiling, to run the program use the ./raytracer followed by driver file name(.txt) and the name of the image to be generated(.ppm)

	Eg. ./raytracer driver00.txt driver00.ppm

Syntax of driver file.

keyword required_parameter1 required_parameter2 required_parameter3 ...

Let us look at individual keywords, their syntax and examples:

1. eye - Describes the eye viewing the image in the space
   
	Syntax : eye X-coordinate Y-coordinate Z-coordinate
   
	Eg. eye 0 10 60

2. look - Describes the direction in which the eye is looking

	Syntax : look X-coordinate Y-coordinate Z-coordinate
   
	Eg. look 0 0 0

3. up - Describes which way the X, Y and Z Coordinates are Oriented. 

	Syntax : up X-coordinate Y-coordinate Z-coordinate
   
	Eg. up 0 1 0

4. d - Distance between eye and camera viewport

	Syntax : d distance   
	Eg. d 50

5. bounds - Bounds of the viewport

	Syntax : bounds Left_Bound Bottom_Bound Right_Bound Top_Bound
   
	Eg. bounds -2 -2 2 2

6. res - Resolution of genereated picture(It has to be a square image)

	Syntax : res pixelcount1 pixelcount2
   
	Eg. res 2048 2048

7. recursionLevel - Number of times a ray can bounce of objects before it disappears.

	Syntax : recursionLevel count
   
	Eg. recursionLevel 3

8. ambient - Ambient lighting in the scene

	Syntax : ambient Red_Light_Level Green_Light_Level Blue_Light_Level
   
	Eg. ambient 0.2 0.2 0.2

9. light - Describes each light source in the scene. If 4th value is 0 the light is at infinite distance in the direction of X, Y and Z coordinates.

	Syntax : light X-Location Y-Location Z-Location At_Infinite_Distance Red_Light_Level Green_Light_Level Blue_Light_Level
   
	Eg. light 10 10 10 1 0.5 0.5 0.5

10. model - Describes a non-sphere object in space.

	Syntax : X-coordinate Y-coordinate Z-coordinate Rotation_angle  Scaling Translation_Along_X-Axis Translation_Along_Y-Axis Translation_Along_Z-Axis Sharp_flag Object_File_Name

	Eg. model 0.0 0.0 1.0 45 1.5 0 0.0 -1.9 sharp checker_colored.obj

11. sphere - Describes a sphere object in space

	Syntax : sphere X-coordinate_Center Y-coordinate_Center Z-coordinate_Center Ka_Red Ka_Green Ka_Blue Kd_Red Kd_Green Kd_Blue Ks_Red Ks_Green Ks_Blue Kr_Red Kr_Green Kr_Blue Ko_Red Ko_Green Ko_Blue eta

	Eg. sphere -1.5 0.0 -0.9 0.3 0.1 0.1 0.1 0.1 0.1 0.1 0.7 0.7 0.7 1 1 1 0.3 0.3 0.3 5.0
