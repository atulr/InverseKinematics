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

float shoulder = 0.f, elbow = 0.f;
//static int shoulder = 0, elbow = 0;
const float PI = 3.14159, length_arm_1 = 1.f, length_arm_2 = 1.f, beta = 0.01f;
//float shoulder = (float)PI/2.f;
//float elbow = (float)PI/9.f;


void init(void)
{
  glClearColor (0.0, 0.0, 0.0, 0.0);
  glShadeModel (GL_FLAT);
}

void display(void)
{
   glClear (GL_COLOR_BUFFER_BIT);
   glPushMatrix();
   glTranslatef (-1.0, 0.0, 0.0);
   glRotatef ((GLfloat) shoulder * 180/PI, 0.0, 0.0, 1.0);
   glTranslatef (1.0, 0.0, 0.0);
   glPushMatrix();
   glScalef (2.0, 0.4, 1.0);
   glutWireCube (length_arm_1);

	/* flush GL buffers */
   glPopMatrix();

   glTranslatef (0.5, 0.0, 0.0);
   glRotatef ((GLfloat) elbow * 180/PI, 0.0, 0.0, 1.0);
   glTranslatef (1.0, 0.0, 0.0);
   glPushMatrix();
   glScalef (2.0, 0.4, 1.0);
   glutWireCube (length_arm_2);
   glPopMatrix();

   glPopMatrix();
   glutSwapBuffers();
   glFlush();
}

void reshape (int w, int h)
{
   glViewport (0, 0, (GLsizei) w, (GLsizei) h);
   glMatrixMode (GL_PROJECTION);
   glLoadIdentity ();
   gluPerspective(65.0, (GLfloat) w/(GLfloat) h, 1.0, 20.0);
   glMatrixMode(GL_MODELVIEW);
   glLoadIdentity();
   glTranslatef (0.0, 0.0, -5.0);
}

gsl_matrix* jacobian(gsl_matrix* J) {
	gsl_matrix_set(J, 0, 0, length_arm_1 * sin(shoulder) * (-1));
	gsl_matrix_set(J, 0, 1, length_arm_2 * sin(elbow) * (-1));
	gsl_matrix_set(J, 1, 0, length_arm_1 * cos(shoulder));
	gsl_matrix_set(J, 1, 1, length_arm_2 * cos(elbow));
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
	gsl_matrix_set(effector, 0, 0, length_arm_1 * cos(shoulder) + length_arm_2 * cos(shoulder - elbow));
	gsl_matrix_set(effector, 1, 0, length_arm_1 * sin(shoulder) + length_arm_2 * sin(shoulder + elbow));
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

void keyboard (unsigned char key, int x, int y)
{
//   switch (key) {
//      case 's':   /*  s key rotates at shoulder  */
//         shoulder = (shoulder + 5) % 360;
//         glutPostRedisplay();
//         break;
//      case 'S':
//         shoulder = (shoulder - 5) % 360;
//         glutPostRedisplay();
//         break;
//      case 'e':  /*  e key rotates at elbow  */
//         elbow = (elbow + 5) % 360;
//         glutPostRedisplay();
//         break;
//      case 'E':
//         elbow = (elbow - 5) % 360;
//         glutPostRedisplay();
//         break;
//      default:
//         break;
//   }
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
		 printf(" x %f ", x_norm);
		 printf(" y %f \n", y_norm);
		 E = end_effector();
		 printf(" ex %f ", length_arm_2 * cos(elbow));
		 printf(" ey %f \n", length_arm_2 * sin(elbow));

		 while(true) {
			 J = jacobian(J);
			 J_INV = inverse(J);
			 del_E = difference(G, E); //del x, del y
			 gsl_blas_dgemm( CblasNoTrans, CblasNoTrans, 1.0, J_INV, del_E, 0.0, del_THETA );

			 shoulder += gsl_matrix_get(del_THETA, 0, 0);
			 elbow += gsl_matrix_get(del_THETA, 1, 0);
			 E = end_effector();
			 if (count++ > 10000){
				 count = 0;
				 break;
			 }
			 glutPostRedisplay();

		 }
	}
}


int main(int argc, char** argv)
{
   glutInit(&argc, argv);
   glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB);
   glutInitWindowSize (500, 500);
   glutInitWindowPosition (100, 100);
   glutCreateWindow (argv[0]);
   init ();
   glutDisplayFunc(display);
   glutReshapeFunc(reshape);
//   glutKeyboardFunc(keyboard);
   glutMouseFunc(mouse);
   glutMainLoop();
   return 0;
}
