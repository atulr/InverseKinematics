/*
 * test.cpp
 *
 *  Created on: Oct 2, 2011
 *      Author: atulrungta
 */

#include <OpenGL/OpenGL.h>
#include <GLUT/glut.h>
#include <math.h>
#include <stdio.h>

#include <gsl/gsl_linalg.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_matrix_double.h>
#include <gsl/gsl_blas.h>

const float PI = 3.14159, length_arm_1 = .5f, length_arm_2 = .3f, beta = 0.01f;
float joint_angle_1 = (float)PI/2.f;
float joint_angle_2 = (float)PI/9.f;

gsl_matrix* jacobian(gsl_matrix* J) {
	gsl_matrix_set(J, 0, 0, length_arm_1 * sin(joint_angle_1) * (-1));
	gsl_matrix_set(J, 0, 1, length_arm_2 * sin(joint_angle_2) * (-1));
	gsl_matrix_set(J, 1, 0, length_arm_1 * cos(joint_angle_1));
	gsl_matrix_set(J, 1, 1, length_arm_2 * cos(joint_angle_2));
	return J;
}

gsl_matrix* inverse(gsl_matrix* J) {
	gsl_matrix* A;
	gsl_matrix* U;
	gsl_matrix* V;
	gsl_matrix* new_S;
	gsl_matrix* new_U;
	gsl_matrix* y;
	gsl_matrix* foo;
	gsl_vector* S;
	gsl_vector* work;

	int M, N;

	M = 2;
	N = 2;

	A = gsl_matrix_alloc(M, M);
	U = gsl_matrix_alloc(M, M);
	V = gsl_matrix_alloc(N, N);
	new_U = gsl_matrix_alloc(N, N);
	new_S = gsl_matrix_alloc(M, N);
	y = gsl_matrix_alloc(M, N);
	foo = gsl_matrix_alloc(M, N);
	S = gsl_vector_alloc(N);
	work = gsl_vector_alloc(N);


	gsl_linalg_SV_decomp(J, V, S, work);
	U = J;

	for(int i = 0 ;i < N; i++){
		if (gsl_vector_get(S, i) != 0.f) {
			gsl_vector_set(S, i, (float)(1/(gsl_vector_get(S, i))));
		}
	}

	gsl_matrix_set(new_S, 0, 0, gsl_vector_get(S, 1));
	gsl_matrix_set(new_S, 0, 1, 0.f);
	gsl_matrix_set(new_S, 1, 0, 0.f);
	gsl_matrix_set(new_S, 1, 1, gsl_vector_get(S, 0));

	gsl_matrix_transpose_memcpy(new_U, U);

	gsl_blas_dgemm( CblasNoTrans, CblasNoTrans, 1.0, V, new_S, 0.0, y );
	gsl_blas_dgemm( CblasNoTrans, CblasNoTrans, 1.0, y, new_U, 0.0, foo );
	return foo;
}

gsl_matrix* end_effector() {
	gsl_matrix* effector;
	effector = gsl_matrix_alloc(2,1);
	gsl_matrix_set(effector, 0, 0, length_arm_2 * cos(joint_angle_2));
	gsl_matrix_set(effector, 1, 0, length_arm_2 * sin(joint_angle_2));
	return effector;
}

gsl_matrix* difference(gsl_matrix* G, gsl_matrix* E) {
	gsl_matrix* new_matrix;
	new_matrix = gsl_matrix_alloc(2, 1);
	gsl_matrix_memcpy(new_matrix, G);
	gsl_matrix_sub(new_matrix, E);
	gsl_matrix_set(new_matrix, 0, 0, (float) beta * gsl_matrix_get(new_matrix, 0, 0));
	gsl_matrix_set(new_matrix, 1, 0, (float) beta * gsl_matrix_get(new_matrix, 1, 0));
	return new_matrix;
}

void mouse(int key, int tmp, int x, int y) {
	float x_norm, y_norm;
	gsl_matrix* E;
	gsl_matrix* G;

	gsl_matrix* J;
	gsl_matrix* del_THETA;
	gsl_matrix* del_E;
	gsl_matrix* J_INV;

	int M, N, P, count;
	 M = 2;
	 N = 2;
	 P = 1;
	 count = 0;

	 E = gsl_matrix_alloc(M, P);
	 G = gsl_matrix_alloc(M, P);
	 J = gsl_matrix_alloc(M, N);
	 del_THETA = gsl_matrix_alloc(M, P);
	 del_E = gsl_matrix_alloc(M, P);
	 J_INV = gsl_matrix_alloc(M, N);


	if (key == GLUT_LEFT_BUTTON) {
		 x_norm = (float)(x - 250)/250;
		 y_norm = (float)(250 - y)/ 250;
		 gsl_matrix_set(G, 0, 0, x_norm);
		 gsl_matrix_set(G, 1, 0, y_norm);
//		 printf("x_norm %f ", x_norm);
//		 printf("y_norm %f \n", y_norm);
		 E = end_effector();
//		 printf("e_x %f ", length_arm_2 * cos(joint_angle_2));
//		 printf("e_y %f \n", length_arm_2 * sin(joint_angle_2));
		 while(true) {
			 J = jacobian(J);
			 J_INV = inverse(J);
			 del_E = difference(G, E); //del x, del y
			 gsl_blas_dgemm( CblasNoTrans, CblasNoTrans, 1.0, J_INV, del_E, 0.0, del_THETA );
//			 printf("%f ", gsl_matrix_get(del_THETA, 0, 0));
//			 printf("%f \n", gsl_matrix_get(del_THETA, 1, 0));

			 joint_angle_1 += gsl_matrix_get(del_THETA, 0, 0);
			 joint_angle_2 += gsl_matrix_get(del_THETA, 1, 0);
//			 printf("ja1 %f \n", joint_angle_1);
//			 printf("ja2 %f \n", joint_angle_2);

			 E = end_effector();
			 if (count++ > 10000){
				 count = 0;
				 break;
			 }
			 glutPostRedisplay();

		 }
	}
}

void display(void) {
	glClear(GL_COLOR_BUFFER_BIT);
    glLineWidth(5.0);
    glColor3f(255,0 , 0);
	glBegin (GL_LINES);
   		glVertex2f (0, 0);
   		glVertex2f (length_arm_1 * cos(joint_angle_1), length_arm_1 * sin(joint_angle_1)); // change
	glEnd();

	glColor3f(255,0 , 8);
	glBegin (GL_LINES);
	   	glVertex2f (length_arm_1 * cos(joint_angle_1), length_arm_1 * sin(joint_angle_1)); //change
		glVertex2f (length_arm_2 * cos(joint_angle_2), length_arm_2 * sin(joint_angle_2));
	glEnd();
	/* flush GL buffers */

	glFlush();

}

int main(int argc, char** argv)
{
	/* Initialize mode and open a window in upper left corner of screen */
	/* Window title is name of program (arg[0]) */

	/* You must call glutInit before any other OpenGL/GLUT calls */
	glutInit(&argc,argv);
	glutInitDisplayMode (GLUT_SINGLE | GLUT_RGB);
	glutInitWindowSize(500,500);
	glutInitWindowPosition(0,0);
	glutCreateWindow("Atul Rungta - Assignment 1");
	glutDisplayFunc(display);
	glutMouseFunc(mouse);
//	init();
	glutMainLoop();

}





