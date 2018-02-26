#include <GL/glut.h>
#include <math.h>
#include <vector>
#include <algorithm>

#define M_PI 3.1415926535897932384626433832795

typedef struct point2d { GLfloat x, y; } POINT2D;
typedef struct { GLfloat x, y, z; } POINT3D;
typedef struct { GLfloat x, y, z, w; } POINT3DH;
typedef struct face { std::vector<POINT3DH> pontok; bool teteje; } FACE;


GLsizei winWidth = 800, winHeight = 600;
GLint keyStates[256], selectedpoint = -1, tengely = 0;
GLfloat s = 3.0f, szog = M_PI / 9.0f, fel = 0.25, r = 1.5;
std::vector<face> faces;
POINT3DH ktpontok[16], vtpontok[16],
alappontok[16] = { 
{ -0.60f, -0.60f, -0.40f, 1.0f },
{ -0.20f, -0.60f, 0.20f, 1.0f },
{ 0.20f, -0.60f, 0.20f, 1.0f },
{ 0.60f, -0.60f, -0.40f, 1.0f },
{ -0.60f, -0.20f, 0.20f, 1.0f },
{ -0.20f, -0.20f, 0.20f, 1.0f },
{ 0.20f, -0.20f, 0.20f, 1.0f },
{ 0.60f, -0.20f, 0.20f, 1.0f },
{ -0.60f, 0.20f, 0.20f, 1.0f },
{ -0.20f, 0.20f, 0.20f, 1.0f },
{ 0.20f, 0.20f, 0.20f, 1.0f },
{ 0.60f, 0.20f, 0.20f, 1.0f },
{ -0.60f, 0.60f, -0.40f, 1.0f },
{ -0.20f, 0.60f, 0.20f, 1.0f },
{ 0.20f, 0.60f, 0.20f, 1.0f },
{ 0.60f, 0.60f, -0.40f, 1.0f }};

POINT2D initPoint2D(GLfloat x, GLfloat y) {
	POINT2D P;
	P.x = x;
	P.y = y;
	return P;
}

POINT3D initVector3(GLfloat x, GLfloat y, GLfloat z) {
	POINT3D P;
	P.x = x;
	P.y = y;
	P.z = z;
	return P;
}

POINT3DH initPoint3DH(GLfloat x, GLfloat y, GLfloat z, GLfloat w) {
	POINT3DH P;
	P.x = x;
	P.y = y;
	P.z = z;
	P.w = w;
	return P;
}

POINT3D kulsoSzorzat(POINT3D a, POINT3D b) {
	return initVector3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

GLdouble belsoSzorzat(POINT3D a, POINT3D b) {
	return (a.x * b.x + a.y * b.y + a.z * b.z);
}

POINT3D normalvektor(face f) {
	POINT3DH a = f.pontok[0];
	POINT3DH b = f.pontok[1];
	POINT3DH c = f.pontok[2];
	POINT3D  n = kulsoSzorzat(initVector3(b.x - a.x, b.y - a.y, b.z - a.z), initVector3(c.x - a.x, c.y - a.y, c.z - a.z));
	GLdouble hossz = sqrtf(n.x*n.x + n.y*n.y + n.z*n.z);
	return initVector3(n.x / hossz, n.y / hossz, n.z / hossz);
}

FACE initFace(std::vector<POINT3DH> pontok) {
	face lap;
	lap.pontok.push_back(pontok[0]);
	lap.pontok.push_back(pontok[1]);
	lap.pontok.push_back(pontok[2]);
	lap.pontok.push_back(pontok[3]);
	if (belsoSzorzat(normalvektor(lap), initVector3(-lap.pontok[0].x, -lap.pontok[0].y, s - lap.pontok[0].z)) < 0)
		lap.teteje = false;
	else
		lap.teteje = true;
	return lap;
}

POINT3D up = initVector3(0.0, 0.0, 1.0); //up vektor
POINT3D P = initVector3(0.0, 0.0, 0.0);  // ide néz a kamera

void init(){
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0.0f, winWidth, 0.0f, winHeight, 0.0f, 1.0f);
	glLineWidth(1.25);
	glPointSize(5.0);
}

void mul_matrices(float A[][4], float B[][4], float C[][4])
{
	float sum;
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++) {
			sum = 0;
			for (int k = 0; k < 4; k++)
				sum = sum + A[i][k] * B[k][j];
			C[i][j] = sum;
		}
}

void mulMV(float A[4][4], POINT3DH regi[16], POINT3DH uj[16]) {
	for (int i = 0; i < 16; i++) {
		uj[i].x = A[0][0] * regi[i].x + A[0][1] * regi[i].y + A[0][2] * regi[i].z + A[0][3] * regi[i].w;
		uj[i].y = A[1][0] * regi[i].x + A[1][1] * regi[i].y + A[1][2] * regi[i].z + A[1][3] * regi[i].w;
		uj[i].z = A[2][0] * regi[i].x + A[2][1] * regi[i].y + A[2][2] * regi[i].z + A[2][3] * regi[i].w;
		uj[i].w = A[3][0] * regi[i].x + A[3][1] * regi[i].y + A[3][2] * regi[i].z + A[3][3] * regi[i].w;
	}
}

float N[4][4], M[4][4];
float Vc[4][4] = { { 1, 0, 0 , 0 },{ 0, 1, 0, 0 },{ 0, 0, 0, 0 },{ 0, 0, -1.0f / s, 1 } };
float wl = -0.5f, wr = 0.5f, wb = -0.5f, wt = 0.5f, vl = 300.0f, vr = 500.0f, vb = 200.0f, vt= 400.0f; //200x os nagyítás
float WtV[4][4] = { { (vr - vl) / (wr - wl), 0, 0 ,vl - wl*((vr - vl) / (wr - wl)) },{ 0, (vt - vb) / (wt - wb), 0, vb - wb*((vt - vb) / (wt - wb)) },{ 0, 0, 1 ,0 },{ 0, 0, 0, 1 } };

void pontok_vetitese_kirajzolasa() {
	mulMV(N, ktpontok, vtpontok);
	
	glBegin(GL_POINTS);
	for (int i = 0; i < 16; i++) {
		if (selectedpoint == i)
			glColor3f(0.0f, 1.0f, 0.0f);
		else
			glColor3f(1.0f, 0.0f, 0.0f);
		glVertex2f(vtpontok[i].x / vtpontok[i].w, vtpontok[i].y / vtpontok[i].w);
	}
	glEnd();
	glColor3f(0.0f, 0.0f, 0.0f);
	glBegin(GL_LINES);
	for (int i = 0; i < 16; i += 4) {
		glVertex2f(vtpontok[i].x / vtpontok[i].w, vtpontok[i].y / vtpontok[i].w);
		glVertex2f(vtpontok[i + 1].x / vtpontok[i + 1].w, vtpontok[i + 1].y / vtpontok[i + 1].w);
		glVertex2f(vtpontok[i + 1].x / vtpontok[i + 1].w, vtpontok[i + 1].y / vtpontok[i + 1].w);
		glVertex2f(vtpontok[i + 2].x / vtpontok[i + 2].w, vtpontok[i + 2].y / vtpontok[i + 2].w);
		glVertex2f(vtpontok[i + 2].x / vtpontok[i + 2].w, vtpontok[i + 2].y / vtpontok[i + 2].w);
		glVertex2f(vtpontok[i + 3].x / vtpontok[i + 3].w, vtpontok[i + 3].y / vtpontok[i + 3].w);
	}
	for (int i = 0; i < 4; i++) {
		glVertex2f(vtpontok[i].x / vtpontok[i].w, vtpontok[i].y / vtpontok[i].w);
		glVertex2f(vtpontok[i + 4].x / vtpontok[i + 4].w, vtpontok[i + 4].y / vtpontok[i + 4].w);
		glVertex2f(vtpontok[i + 4].x / vtpontok[i + 4].w, vtpontok[i + 4].y / vtpontok[i + 4].w);
		glVertex2f(vtpontok[i + 8].x / vtpontok[i + 8].w, vtpontok[i + 8].y / vtpontok[i + 8].w);
		glVertex2f(vtpontok[i + 8].x / vtpontok[i + 8].w, vtpontok[i + 8].y / vtpontok[i + 8].w);
		glVertex2f(vtpontok[i + 12].x / vtpontok[i + 12].w, vtpontok[i + 12].y / vtpontok[i + 12].w);
	}
	glEnd();
}

void keyPressed(unsigned char key, int x, int y) {
	keyStates[key] = 1;
}

void keyUp(unsigned char key, int x, int y) {
	keyStates[key] = 0;
}

void keyOperations() {
	if (keyStates['a']) {
		szog -= 0.025;
	}
	if (keyStates['d']) {
		szog += 0.025;
	}
	if (keyStates['w']) {
		fel += 0.05;
	}
	if (keyStates['s']) {
		fel -= 0.05;
	}
	if (keyStates['+']) {
		if (selectedpoint == -1) {
			if (r > 0.25) r -= 0.05;
		}
		else if (tengely == 0) {
			alappontok[selectedpoint].x += 0.01;
		}
		else if (tengely == 1) {
			alappontok[selectedpoint].y += 0.01;
		}
		else if (tengely == 2) {
			alappontok[selectedpoint].z += 0.01;
		}
	}
	if (keyStates['-']) {
		if (selectedpoint == -1) {
			if (r < 5) r += 0.05;
		}
		else if (tengely == 0){
			alappontok[selectedpoint].x -= 0.01;
		}
		else if (tengely == 1) {
			alappontok[selectedpoint].y -= 0.01;
		}
		else if (tengely == 2) {
			alappontok[selectedpoint].z -= 0.01;
		}
	}
	if (keyStates['x']) {
		tengely = 0;
		
	}
	if (keyStates['y']) {
		tengely = 1;
	}
	if (keyStates['z']) {
		tengely = 2;
	}
	glutPostRedisplay();
}

GLdouble bazisfv(double t, int i) {
	if (i==3)
		return t*t*t;
	else if (i==2) 
		return -3 * (t*t*t) + 3 * (t*t);
	else if (i==1)
		return 3 * (t*t*t) - 6 * (t*t) + 3 * t;
	else 
		return -(t*t*t) + 3 * (t*t) - 3 * t + 1;
}

void rendez() {
	std::sort(faces.begin(), faces.end(), [](face a, face b) {

		POINT3D a_kozepe = initVector3((a.pontok[0].x + a.pontok[1].x + a.pontok[2].x + a.pontok[3].x) / 4,   //a lap kozepe
									   (a.pontok[0].y + a.pontok[1].y + a.pontok[2].y + a.pontok[3].y) / 4,
									   (a.pontok[0].z + a.pontok[1].z + a.pontok[2].z + a.pontok[3].z) / 4);
		POINT3D b_kozepe = initVector3((b.pontok[0].x + b.pontok[1].x + b.pontok[2].x + b.pontok[3].x) / 4,   //b lap kozepe
								   	   (b.pontok[0].y + b.pontok[1].y + b.pontok[2].y + b.pontok[3].y) / 4,
			                           (b.pontok[0].z + b.pontok[1].z + b.pontok[2].z + b.pontok[3].z) / 4);

		GLdouble a_tav = sqrt((a_kozepe.x * a_kozepe.x) + (a_kozepe.y * a_kozepe.y) + (a_kozepe.z - s) * (a_kozepe.z - s)); // az a lap kozepenek a centrumtol(0,0,s) valo tavolsaga
		GLdouble b_tav = sqrt((b_kozepe.x * b_kozepe.x) + (b_kozepe.y * b_kozepe.y) + (b_kozepe.z - s) * (b_kozepe.z - s)); // a b lap  kozepenek a centrumtol valo tavolsaga

		return (a_tav > b_tav); //csokkeno rendezes (legtavolabbit rajzoljuk eloszor)
	});
}

void transformal_es_kirajzol(){
	face regi,uj;
	GLdouble xx, yy, zz, ww;
	for (int i = 0; i < faces.size(); i++) {
		regi = faces[i];
		uj.pontok.clear();
		uj.teteje = regi.teteje;
		for (int j = 0; j < 4; j++) {
			xx = N[0][0] * regi.pontok[j].x + N[0][1] * regi.pontok[j].y + N[0][2] * regi.pontok[j].z + N[0][3] * regi.pontok[j].w;
			yy = N[1][0] * regi.pontok[j].x + N[1][1] * regi.pontok[j].y + N[1][2] * regi.pontok[j].z + N[1][3] * regi.pontok[j].w;
			zz = N[2][0] * regi.pontok[j].x + N[2][1] * regi.pontok[j].y + N[2][2] * regi.pontok[j].z + N[2][3] * regi.pontok[j].w;
			ww = N[3][0] * regi.pontok[j].x + N[3][1] * regi.pontok[j].y + N[3][2] * regi.pontok[j].z + N[3][3] * regi.pontok[j].w;
			uj.pontok.push_back(initPoint3DH(xx, yy, zz, ww));
		}
		if (uj.teteje) {
			glColor3f(0.95f, 0.22f, 0.35f);
			glBegin(GL_POLYGON);
				glVertex2f(uj.pontok[0].x / uj.pontok[0].w, uj.pontok[0].y / uj.pontok[0].w);
				glVertex2f(uj.pontok[1].x / uj.pontok[1].w, uj.pontok[1].y / uj.pontok[1].w);
				glVertex2f(uj.pontok[1].x / uj.pontok[1].w, uj.pontok[1].y / uj.pontok[1].w);
				glVertex2f(uj.pontok[2].x / uj.pontok[2].w, uj.pontok[2].y / uj.pontok[2].w);
				glVertex2f(uj.pontok[2].x / uj.pontok[2].w, uj.pontok[2].y / uj.pontok[2].w);
				glVertex2f(uj.pontok[0].x / uj.pontok[0].w, uj.pontok[0].y / uj.pontok[0].w);
			glEnd();
			glColor3f(0.96f, 0.65f, 0.01f);
			glBegin(GL_POLYGON);
				glVertex2f(uj.pontok[0].x / uj.pontok[0].w, uj.pontok[0].y / uj.pontok[0].w);
				glVertex2f(uj.pontok[2].x / uj.pontok[2].w, uj.pontok[2].y / uj.pontok[2].w);
				glVertex2f(uj.pontok[2].x / uj.pontok[2].w, uj.pontok[2].y / uj.pontok[2].w);
				glVertex2f(uj.pontok[3].x / uj.pontok[3].w, uj.pontok[3].y / uj.pontok[3].w);
				glVertex2f(uj.pontok[3].x / uj.pontok[3].w, uj.pontok[3].y / uj.pontok[3].w);
				glVertex2f(uj.pontok[0].x / uj.pontok[0].w, uj.pontok[0].y / uj.pontok[0].w);
			glEnd();
		}else{
			glColor3f(0.16f, 0.56f, 0.27f);
			glBegin(GL_POLYGON);
				glVertex2f(uj.pontok[0].x / uj.pontok[0].w, uj.pontok[0].y / uj.pontok[0].w);
				glVertex2f(uj.pontok[1].x / uj.pontok[1].w, uj.pontok[1].y / uj.pontok[1].w);
				glVertex2f(uj.pontok[1].x / uj.pontok[1].w, uj.pontok[1].y / uj.pontok[1].w);
				glVertex2f(uj.pontok[2].x / uj.pontok[2].w, uj.pontok[2].y / uj.pontok[2].w);
				glVertex2f(uj.pontok[2].x / uj.pontok[2].w, uj.pontok[2].y / uj.pontok[2].w);
				glVertex2f(uj.pontok[0].x / uj.pontok[0].w, uj.pontok[0].y / uj.pontok[0].w);
			glEnd();
			glColor3f(0.12f, 0.36f, 0.17f);
			glBegin(GL_POLYGON);
				glVertex2f(uj.pontok[0].x / uj.pontok[0].w, uj.pontok[0].y / uj.pontok[0].w);
				glVertex2f(uj.pontok[2].x / uj.pontok[2].w, uj.pontok[2].y / uj.pontok[2].w);
				glVertex2f(uj.pontok[2].x / uj.pontok[2].w, uj.pontok[2].y / uj.pontok[2].w);
				glVertex2f(uj.pontok[3].x / uj.pontok[3].w, uj.pontok[3].y / uj.pontok[3].w);
				glVertex2f(uj.pontok[3].x / uj.pontok[3].w, uj.pontok[3].y / uj.pontok[3].w);
				glVertex2f(uj.pontok[0].x / uj.pontok[0].w, uj.pontok[0].y / uj.pontok[0].w);
			glEnd();
		}
	}
}

void TopKek(){
	glClear(GL_COLOR_BUFFER_BIT);
	keyOperations();
	
	POINT3D C = initVector3(r*cos(szog), r*sin(szog), fel);  
	POINT3D w = initVector3(  //a kamera az origoba nez
		C.x / sqrt(C.x * C.x + C.y * C.y + C.z * C.z),
		C.y / sqrt(C.x * C.x + C.y * C.y + C.z * C.z),
		C.z / sqrt(C.x * C.x + C.y * C.y + C.z * C.z)
	);
	POINT3D u1 = kulsoSzorzat(up, w);
	POINT3D u = initVector3(
		u1.x / sqrt(u1.x * u1.x + u1.y * u1.y + u1.z * u1.z),
		u1.y / sqrt(u1.x * u1.x + u1.y * u1.y + u1.z * u1.z),
		u1.z / sqrt(u1.x * u1.x + u1.y * u1.y + u1.z * u1.z)
	);
	POINT3D v1 = kulsoSzorzat(w, u);
	POINT3D v = initVector3(
		v1.x / sqrt(v1.x * v1.x + v1.y * v1.y + v1.z * v1.z),
		v1.y / sqrt(v1.x * v1.x + v1.y * v1.y + v1.z * v1.z),
		v1.z / sqrt(v1.x * v1.x + v1.y * v1.y + v1.z * v1.z)
	);

	float K[4][4] = { { u.x, u.y, u.z ,-belsoSzorzat(C, u) },
					  { v.x, v.y, v.z ,-belsoSzorzat(C, v) },
					  { w.x, w.y, w.z ,-belsoSzorzat(C, w) },
					  { 0,    0,   0,          1           }};

	mul_matrices(WtV, Vc, N); //vetitesi matrix eloallitasa
	mulMV(K, alappontok, ktpontok); // alappontok a kamera koordinátarendszerében

	std::vector<POINT3DH> elozok, kovetkezok;
	faces.clear();
	for (double s = 0; s < 1.01; s += 0.1) {
		for (double t = 0; t < 1.01; t += 0.1) {
			double x = 0, y = 0, z = 0;
			for (int i = 0; i < 4; i++) {
				for (int j = 0; j < 4; j++) {					
					x += ktpontok[i * 4 + j].x * bazisfv(s, i) * bazisfv(t, j);
					y += ktpontok[i * 4 + j].y * bazisfv(s, i) * bazisfv(t, j);
					z += ktpontok[i * 4 + j].z * bazisfv(s, i) * bazisfv(t, j);
				}
			}
			kovetkezok.push_back(initPoint3DH(x, y, z, 1));
		}
		if (s > 0) {
			for (int i = 0; i < kovetkezok.size() - 1; i++) {
				std::vector<POINT3DH> points = { elozok[i], elozok[i + 1], kovetkezok[i + 1], kovetkezok[i] };
				face lap = initFace(points);
				
			faces.push_back(lap);
			}
		}
		elozok = kovetkezok;
		kovetkezok.clear();
	}
	rendez(); //festõ algoritmus - láthatóság szerinti rendezés
	transformal_es_kirajzol(); // a hálót alkotó facek transzformálása, majd kirajzolása
	pontok_vetitese_kirajzolasa(); //végül a pontok és a rács kirajzolása
	glFlush();
	glutSwapBuffers();
}

GLfloat dist(POINT3DH P1, POINT2D P2) {
	GLfloat t1 = (P1.x/P1.w) - P2.x;
	GLfloat t2 = (P1.y / P1.w) - P2.y;
	return t1 * t1 + t2 * t2;
}

GLint getActivePoint(GLint sens, GLint x, GLint y) {
	GLint i, s = sens * sens;
	POINT2D P = initPoint2D(x, y);
	for (i = 0; i < 16; i++)
		if (dist(vtpontok[i], P) < s) 
			return i;
	return -1;
}

void processMouse(GLint button, GLint action, GLint xMouse, GLint yMouse) {
	GLint i;
	if (button == GLUT_LEFT_BUTTON && action == GLUT_DOWN) {
		if ((i = getActivePoint(8, xMouse, winHeight - yMouse)) != -1)
			selectedpoint = i;
		else 
			selectedpoint = -1;
	}
}

int main (int argc, char** argv)
{
    glutInit (&argc, argv);                         // Initialize GLUT.
    glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB);   // Set display mode.
    glutInitWindowPosition (50, 100);   // Set top-left display-window position.
    glutInitWindowSize (800, 600);      // Set display-window width and height.
    glutCreateWindow ("harmadik házi - Bézier-felület"); // Create display window.
    init ( );                            // Execute initialization procedure.
    glutDisplayFunc (TopKek);       // Send graphics to display window.
	glutMouseFunc(processMouse);
	glutKeyboardFunc(keyPressed);
	glutKeyboardUpFunc(keyUp);
    glutMainLoop ( );                    // Display everything and wait.
    return 0;
}