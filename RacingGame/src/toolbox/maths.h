#ifndef MATHS_H
#define MATHS_H

class Vector3f;
class Vector2f;
class Camera;

#include <math.h>
#include <random>
#include <chrono>
#include "matrix.h"

#define M_PI 3.14159265358979323846f

class Maths
{
private:
	static std::mt19937* generatorUniform;
	static std::uniform_real_distribution<float>* distributionUniform;

	static std::default_random_engine* generatorNormal;
	static std::normal_distribution<float>* distributionNormal;

public:
	static float toRadians(float deg);

	static float toDegrees(float rad);

	//Blank Matrix4f should have already been created. This function does not allocate any memory to matrix.
	static void createTransformationMatrix(Matrix4f* result, Vector3f* translation, float rX, float rY, float rZ, float rS, float scale);

	static void createTransformationMatrix(Matrix4f* result, Vector3f* translation, float rX, float rY, float rZ, float rS, float sX, float sY, float sZ);

	//For 2D GUI Images
	static void createTransformationMatrix(Matrix4f* result, Vector2f* translation, float roation, Vector2f* scale);

	//Based on how sadx calculates rotations
	static void createTransformationMatrixSADX(Matrix4f* result, Vector3f* translation, float rX, float rY, float rZ, float scale);

	static void createViewMatrix(Matrix4f* result, Camera* cam);

	//Returns the difference between the two angles
	//ang1 and ang2 should be in degrees
	static float compareTwoAngles(float ang1, float ang2);

	static int sign(float value);

	//calculates the number of base 10 decimal digits in the number. 
	// examples: numDigits(38)    = 2  
	//           numDigits(68932) = 5
	static int numDigits(int number);

	//
	static float approach(float initialValue, float terminalValue, float approachConstant, float timePassed);

	//result needs to be array of 3 doubles
	//theta is in radians
	static void rotatePoint(float result[],
		float a, float b, float c,
		float u, float v, float w,
		float x, float y, float z,
		float theta);

	//angle in radians
	static Vector3f mapInputs3(float angle, float mag, Vector3f* VecC);

	//in radians
	static Vector3f mapCamera(float yaw, float pitch, float mag, Vector3f* VecC);


	static Vector3f bounceVector(Vector3f* initialVelocity, Vector3f* surfaceNormal, float elasticity);

	/**
	* Calculate the x and z speeds relative to a plane based off
	* the previous position you are coming in from
	*
	* @param xspd the x speed that you are going at before collision
	* @param yspd the y speed that you are going at before collision
	* @param zspd the z speed that you are going at before collision
	* @param A the collision point on the triangle
	* @param normal the normal of the triangle
	*/
	static Vector3f calculatePlaneSpeed(float xspd, float yspd, float zspd, Vector3f* normal);

	/** Returns the point on a sphere that has the given angles from the center
	* @param angH in radians
	* @param angV in radians
	* @param radius
	* @return
	*/
	static Vector3f spherePositionFromAngles(float angH, float angV, float radius);

	//Generates a uniformly distributed random position on a sphere of radius 1
	static Vector3f randomPointOnSphere();

	static Vector3f projectOntoPlane(Vector3f* A, Vector3f* normal);

	//returns uniform random float >= 0 and < 1
	static float random();

	//normal distribution mean = 0, std dev = 1
	static float nextGaussian();

	//returns uniform random float >= 0 and < 1
	static float nextUniform();

	//Creates a view matrix that is centered at the eye position, and looking at focus
	static Matrix4f lookAt(Vector3f* eye, Vector3f* focus, Vector3f* up);

	//Given two vectors that are perpencicular to each other, calculate
	// a third vector that is perpendicular to both
	static Vector3f calcThirdAxis(Vector3f* A, Vector3f* B);

	//Given a basis {v1, v2, v3}, computes the coordinate vector
	// of a point a in the new basis
	static Vector3f coordinatesRelativeToBasis(Vector3f* v1, Vector3f* v2, Vector3f* v3, Vector3f* a);
};

#endif