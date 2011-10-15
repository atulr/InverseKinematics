#include <stdio.h>
#include "vector.h"

Vector Vector::cross(Vector other) {
	float x1, y1, z1;
	x1 = (float)(y*other.z - z*other.y);
	y1 = (float)(z*other.x - x*other.z);
	z1 = (float)(x*other.y - y*other.x);
	Vector product(x1, y1, z1);
	return product;
}

Vector Vector::scmult(float scalar) {
	float x1, y1, z1;
	x1 = (float)(scalar * x);
	y1 = (float)(scalar * y);
	z1 = (float)(scalar * z);
	Vector product(x1, y1, z1);
	return product;
}

Vector Vector::add(Vector other){
	float x1,y1,z1;
	x1 = (float)(x + other.x);
	y1 = (float)(y + other.y);
	z1 = (float)(z + other.z);
	Vector vector(x1, y1, z1);
	return vector;
}

Vector Vector::sub(Vector other){
	float x1,y1,z1;
	x1 = (float)(x - other.x);
	y1 = (float)(y - other.y);
	z1 = (float)(z - other.z);
	Vector vector(x1, y1, z1);
	return vector;
}

float Vector::dot(Vector other) {
	float product = 0.0f;
	product += (float)(other.x * x);
	product += (float)(other.y * y);
	product += (float)(other.z * z);
	return product;
}

Vector Vector::normalize() {
	float magnitude = length();
	Vector vector((float)x/magnitude, (float)y/magnitude, (float)z/magnitude);
	return vector;
}

float Vector::length (){
	return ((float)sqrt((float)x*x + (float)y*y + (float)z*z));
}
