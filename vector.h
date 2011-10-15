/*
 * vector.h
 *
 *  Created on: Sep 23, 2011
 *      Author: atulrungta
 */

#ifndef VECTOR_H_
#define VECTOR_H_

class Vector {
	float x,y,z;
public:
	Vector(){
		x = 0.0f;
		y = 0.0f;
		z = 0.0f;
	}

	Vector(float x1, float y1, float z1) {
		x = x1;
		y = y1;
		z = z1;
	}
	float getx() {return x;}
	float gety() {return y;}
	float dot(Vector other);
	Vector normalize();
	Vector cross(Vector other);
	Vector add(Vector other);
	Vector sub(Vector other);
	Vector scmult(float scalar);
	float length();

};

#endif /* VECTOR_H_ */
