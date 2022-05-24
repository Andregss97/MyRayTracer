 ///////////////////////////////////////////////////////////////////////
//
// P3D Course
// (c) 2021 by Jo�o Madeiras Pereira
//Ray Tracing P3F scenes and drawing points with Modern OpenGL
//
///////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string.h>
#include <stdio.h>
#include <chrono>
#include <conio.h>
#include <time.h>

#include <GL/glew.h>
#include <GL/freeglut.h>
#include <IL/il.h>

#include "scene.h"
#include "rayAccelerator.h"
#include "maths.h"
#include "macros.h"
#include "vector.h"

//Enable OpenGL drawing.  
bool drawModeEnabled = true;

bool P3F_scene = true; //choose between P3F scene or a built-in random scene

#define MAX_DEPTH 4  //number of bounces

#define CAPTION "Whitted Ray-Tracer"
#define VERTEX_COORD_ATTRIB 0
#define COLOR_ATTRIB 1

#define NSAMPLES 4

#define ANTIALIASING true
#define	SOFTSHADOWS true
#define LIGHT_SIDE 0.5
#define DOF true

unsigned int FrameCount = 0;

// Current Camera Position
float camX, camY, camZ;

//Original Camera position;
Vector Eye;

// Mouse Tracking Variables
int startX, startY, tracking = 0;

// Camera Spherical Coordinates
float alpha = 0.0f, beta = 0.0f;
float r = 4.0f;

// Frame counting and FPS computation
long myTime, timebase = 0, frame = 0;
char s[32];


// Points defined by 2 attributes: positions which are stored in vertices array and colors which are stored in colors array
float *colors;
float *vertices;
int size_vertices;
int size_colors;

//Array of Pixels to be stored in a file by using DevIL library
uint8_t *img_Data;

GLfloat m[16];  //projection matrix initialized by ortho function

GLuint VaoId;
GLuint VboId[2];

GLuint VertexShaderId, FragmentShaderId, ProgramId;
GLint UniformId;

Scene* scene = NULL;


Grid* grid_ptr = NULL;
BVH* bvh_ptr = NULL;
accelerator Accel_Struct = NONE;

int RES_X, RES_Y;

int WindowHandle = 0;



/////////////////////////////////////////////////////////////////////// ERRORS

bool isOpenGLError() {
	bool isError = false;
	GLenum errCode;
	const GLubyte *errString;
	while ((errCode = glGetError()) != GL_NO_ERROR) {
		isError = true;
		errString = gluErrorString(errCode);
		std::cerr << "OpenGL ERROR [" << errString << "]." << std::endl;
	}
	return isError;
}

void checkOpenGLError(std::string error)
{
	if(isOpenGLError()) {
		std::cerr << error << std::endl;
		exit(EXIT_FAILURE);
	}
}

/////////////////////////////////////////////////////////////////////// SHADERs

const GLchar* VertexShader =
{
	"#version 430 core\n"

	"in vec2 in_Position;\n"
	"in vec3 in_Color;\n"
	"uniform mat4 Matrix;\n"
	"out vec4 color;\n"

	"void main(void)\n"
	"{\n"
	"	vec4 position = vec4(in_Position, 0.0, 1.0);\n"
	"	color = vec4(in_Color, 1.0);\n"
	"	gl_Position = Matrix * position;\n"

	"}\n"
};

const GLchar* FragmentShader =
{
	"#version 430 core\n"

	"in vec4 color;\n"
	"out vec4 out_Color;\n"

	"void main(void)\n"
	"{\n"
	"	out_Color = color;\n"
	"}\n"
};

void createShaderProgram()
{
	VertexShaderId = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(VertexShaderId, 1, &VertexShader, 0);
	glCompileShader(VertexShaderId);

	FragmentShaderId = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(FragmentShaderId, 1, &FragmentShader, 0);
	glCompileShader(FragmentShaderId);

	ProgramId = glCreateProgram();
	glAttachShader(ProgramId, VertexShaderId);
	glAttachShader(ProgramId, FragmentShaderId);

	glBindAttribLocation(ProgramId, VERTEX_COORD_ATTRIB, "in_Position");
	glBindAttribLocation(ProgramId, COLOR_ATTRIB, "in_Color");
	
	glLinkProgram(ProgramId);
	UniformId = glGetUniformLocation(ProgramId, "Matrix");

	checkOpenGLError("ERROR: Could not create shaders.");
}

void destroyShaderProgram()
{
	glUseProgram(0);
	glDetachShader(ProgramId, VertexShaderId);
	glDetachShader(ProgramId, FragmentShaderId);

	glDeleteShader(FragmentShaderId);
	glDeleteShader(VertexShaderId);
	glDeleteProgram(ProgramId);

	checkOpenGLError("ERROR: Could not destroy shaders.");
}

/////////////////////////////////////////////////////////////////////// VAOs & VBOs

void createBufferObjects()
{
	glGenVertexArrays(1, &VaoId);
	glBindVertexArray(VaoId);
	glGenBuffers(2, VboId);
	glBindBuffer(GL_ARRAY_BUFFER, VboId[0]);

	/* S� se faz a aloca��o dos arrays glBufferData (NULL), e o envio dos pontos para a placa gr�fica
	� feito na drawPoints com GlBufferSubData em tempo de execu��o pois os arrays s�o GL_DYNAMIC_DRAW */
	glBufferData(GL_ARRAY_BUFFER, size_vertices, NULL, GL_DYNAMIC_DRAW);
	glEnableVertexAttribArray(VERTEX_COORD_ATTRIB);
	glVertexAttribPointer(VERTEX_COORD_ATTRIB, 2, GL_FLOAT, 0, 0, 0);
	
	glBindBuffer(GL_ARRAY_BUFFER, VboId[1]);
	glBufferData(GL_ARRAY_BUFFER, size_colors, NULL, GL_DYNAMIC_DRAW);
	glEnableVertexAttribArray(COLOR_ATTRIB);
	glVertexAttribPointer(COLOR_ATTRIB, 3, GL_FLOAT, 0, 0, 0);
	
// unbind the VAO
	glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
//	glDisableVertexAttribArray(VERTEX_COORD_ATTRIB); 
//	glDisableVertexAttribArray(COLOR_ATTRIB);
	checkOpenGLError("ERROR: Could not create VAOs and VBOs.");
}

void destroyBufferObjects()
{
	glDisableVertexAttribArray(VERTEX_COORD_ATTRIB);
	glDisableVertexAttribArray(COLOR_ATTRIB);
	
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);

	glDeleteBuffers(1, VboId);
	glDeleteVertexArrays(1, &VaoId);
	checkOpenGLError("ERROR: Could not destroy VAOs and VBOs.");
}

void drawPoints()
{
	FrameCount++;
	glClear(GL_COLOR_BUFFER_BIT);

	glBindVertexArray(VaoId);
	glUseProgram(ProgramId);

	glBindBuffer(GL_ARRAY_BUFFER, VboId[0]);
	glBufferSubData(GL_ARRAY_BUFFER, 0, size_vertices, vertices);
	glBindBuffer(GL_ARRAY_BUFFER, VboId[1]);
	glBufferSubData(GL_ARRAY_BUFFER, 0, size_colors, colors);

	glUniformMatrix4fv(UniformId, 1, GL_FALSE, m);
	glDrawArrays(GL_POINTS, 0, RES_X*RES_Y);
	glFinish();

	glUseProgram(0);
	glBindVertexArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	checkOpenGLError("ERROR: Could not draw scene.");
}

ILuint saveImgFile(const char *filename) {
	ILuint ImageId;

	ilEnable(IL_FILE_OVERWRITE);
	ilGenImages(1, &ImageId);
	ilBindImage(ImageId);

	ilTexImage(RES_X, RES_Y, 1, 3, IL_RGB, IL_UNSIGNED_BYTE, img_Data /*Texture*/);
	ilSaveImage(filename);

	ilDisable(IL_FILE_OVERWRITE);
	ilDeleteImages(1, &ImageId);
	if (ilGetError() != IL_NO_ERROR)return ilGetError();

	return IL_NO_ERROR;
}

/////////////////////////////////////////////////////////////////////// CALLBACKS

void timer(int value)
{
	std::ostringstream oss;
	oss << CAPTION << ": " << FrameCount << " FPS @ (" << RES_X << "x" << RES_Y << ")";
	std::string s = oss.str();
	glutSetWindow(WindowHandle);
	glutSetWindowTitle(s.c_str());
	FrameCount = 0;
	glutTimerFunc(1000, timer, 0);
}


// Callback function for glutCloseFunc
void cleanup()
{
	destroyShaderProgram();
	destroyBufferObjects();
}

void ortho(float left, float right, float bottom, float top, 
			float nearp, float farp)
{
	m[0 * 4 + 0] = 2 / (right - left);
	m[0 * 4 + 1] = 0.0;
	m[0 * 4 + 2] = 0.0;
	m[0 * 4 + 3] = 0.0;
	m[1 * 4 + 0] = 0.0;
	m[1 * 4 + 1] = 2 / (top - bottom);
	m[1 * 4 + 2] = 0.0;
	m[1 * 4 + 3] = 0.0;
	m[2 * 4 + 0] = 0.0;
	m[2 * 4 + 1] = 0.0;
	m[2 * 4 + 2] = -2 / (farp - nearp);
	m[2 * 4 + 3] = 0.0;
	m[3 * 4 + 0] = -(right + left) / (right - left);
	m[3 * 4 + 1] = -(top + bottom) / (top - bottom);
	m[3 * 4 + 2] = -(farp + nearp) / (farp - nearp);
	m[3 * 4 + 3] = 1.0;
}

void reshape(int w, int h)
{
    glClear(GL_COLOR_BUFFER_BIT);
	glViewport(0, 0, w, h);
	ortho(0, (float)RES_X, 0, (float)RES_Y, -1.0, 1.0);
}

void processKeys(unsigned char key, int xx, int yy)
{
	switch (key) {

		case 27:
			glutLeaveMainLoop();
			break;

		case 'r':
			camX = Eye.x;
			camY = Eye.y;
			camZ = Eye.z;
			r = Eye.length();
			beta = asinf(camY / r) * 180.0f / 3.14f;
			alpha = atanf(camX / camZ) * 180.0f / 3.14f;
			break;

		case 'c':
			printf("Camera Spherical Coordinates (%f, %f, %f)\n", r, beta, alpha);
			printf("Camera Cartesian Coordinates (%f, %f, %f)\n", camX, camY, camZ);
			break;
	}
}


// ------------------------------------------------------------
//
// Mouse Events
//

void processMouseButtons(int button, int state, int xx, int yy)
{
	// start tracking the mouse
	if (state == GLUT_DOWN) {
		startX = xx;
		startY = yy;
		if (button == GLUT_LEFT_BUTTON)
			tracking = 1;
		else if (button == GLUT_RIGHT_BUTTON)
			tracking = 2;
	}

	//stop tracking the mouse
	else if (state == GLUT_UP) {
		if (tracking == 1) {
			alpha -= (xx - startX);
			beta += (yy - startY);
		}
		else if (tracking == 2) {
			r += (yy - startY) * 0.01f;
			if (r < 0.1f)
				r = 0.1f;
		}
		tracking = 0;
	}
}

// Track mouse motion while buttons are pressed

void processMouseMotion(int xx, int yy)
{

	int deltaX, deltaY;
	float alphaAux, betaAux;
	float rAux;

	deltaX = -xx + startX;
	deltaY = yy - startY;

	// left mouse button: move camera
	if (tracking == 1) {


		alphaAux = alpha + deltaX;
		betaAux = beta + deltaY;

		if (betaAux > 85.0f)
			betaAux = 85.0f;
		else if (betaAux < -85.0f)
			betaAux = -85.0f;
		rAux = r;
	}
	// right mouse button: zoom
	else if (tracking == 2) {

		alphaAux = alpha;
		betaAux = beta;
		rAux = r + (deltaY * 0.01f);
		if (rAux < 0.1f)
			rAux = 0.1f;
	}

	camX = rAux * sin(alphaAux * 3.14f / 180.0f) * cos(betaAux * 3.14f / 180.0f);
	camZ = rAux * cos(alphaAux * 3.14f / 180.0f) * cos(betaAux * 3.14f / 180.0f);
	camY = rAux * sin(betaAux * 3.14f / 180.0f);
}

void mouseWheel(int wheel, int direction, int x, int y) {

	r += direction * 0.1f;
	if (r < 0.1f)
		r = 0.1f;

	camX = r * sin(alpha * 3.14f / 180.0f) * cos(beta * 3.14f / 180.0f);
	camZ = r * cos(alpha * 3.14f / 180.0f) * cos(beta * 3.14f / 180.0f);
	camY = r * sin(beta * 3.14f / 180.0f);
}


void setupGLEW() {
	glewExperimental = GL_TRUE;
	GLenum result = glewInit() ; 
	if (result != GLEW_OK) { 
		std::cerr << "ERROR glewInit: " << glewGetString(result) << std::endl;
		exit(EXIT_FAILURE);
	} 
	GLenum err_code = glGetError();
	printf ("Vendor: %s\n", glGetString (GL_VENDOR));
	printf ("Renderer: %s\n", glGetString (GL_RENDERER));
	printf ("Version: %s\n", glGetString (GL_VERSION));
	printf ("GLSL: %s\n", glGetString (GL_SHADING_LANGUAGE_VERSION));
}

void setupGLUT(int argc, char* argv[])
{
	glutInit(&argc, argv);
	
	glutInitContextVersion(4, 3);
	glutInitContextFlags(GLUT_FORWARD_COMPATIBLE);
	glutInitContextProfile(GLUT_CORE_PROFILE);

	glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE,GLUT_ACTION_GLUTMAINLOOP_RETURNS);
	
	glutInitWindowPosition(100, 250);
	glutInitWindowSize(RES_X, RES_Y);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA);
	glDisable(GL_DEPTH_TEST);
	WindowHandle = glutCreateWindow(CAPTION);
	if(WindowHandle < 1) {
		std::cerr << "ERROR: Could not create a new rendering window." << std::endl;
		exit(EXIT_FAILURE);
	}
}


/////////////////////////////////////////////////////YOUR CODE HERE///////////////////////////////////////////////////////////////////////////////////////


// Fresnel equation to calculate the amount of light reflected
// kr -> Ratio of reflected light for a given incident direction (I) and surfance normal (nHit)
void fresnel(Vector &I, Vector &nHit, float &ior, float &kr){

	float cosi = clamp(I * nHit, -1, 1);
	float etai = 1;				// refraction index of the medium the ray is in before entering the second medium
	float etat = ior;			// refraction index of the object

	if (cosi > 0) {
		std::swap(etai, etat);
	}

	float sint = etai / etat * sqrtf(std::max(0.f, 1 - cosi * cosi));

	// Total internal reflection
	if (sint >= 1) {
		kr = 1;
	}

	else {
		float cost = sqrtf(std::max(0.f, 1 - sint * sint));	
		cosi = fabsf(cosi);	// absolute value

		// Fresnel equations
		float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
		float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));

		kr = (Rs * Rs + Rp * Rp) / 2;
	}
}


Color rayTracing(Ray ray, int depth, float ior_1,int offsetX, int offsetY)  //index of refraction of medium 1 where the ray is travelling
{

	Color color;


	Vector pHit; // intersection point
	Vector nHit; // normal in pHit

	Object* object = NULL;
	float minDist = FLT_MAX;
	float t;

	Vector L;
	int num_objects = scene->getNumObjects();
	int num_lights = scene->getNumLights();

	Light* light;

	// search for intersections -> choose closest object
	for (int k = 0; k < num_objects; k++) {
		Object *obj = scene->getObject(k);
		if (obj->intercepts(ray, t)) {

			if (t < minDist) {
				minDist = t;
				object = scene->getObject(k);
			}
		}
	}

	// If there was an object that collided with the ray
	if (object != NULL) {
		pHit = ray.origin + ray.direction * minDist;
		
		float transmitanceFlag = object->GetMaterial()->GetTransmittance();
		float reflectiveFlag = object->GetMaterial()->GetReflection();

		nHit = object->getNormal(pHit);

		for (int j = 0; j < num_lights; j++) {
			light = scene->getLight(j);

			if (SOFTSHADOWS && ANTIALIASING) {
				float offX = offsetX * 1.0;
				float offY = offsetY * 1.0;

				Vector position = Vector(
					light->position.x + LIGHT_SIDE * (offX + rand_float()) / NSAMPLES,
					light->position.y + LIGHT_SIDE * (offY + rand_float()) / NSAMPLES,
					light->position.z);

				L = position - pHit;
			}
			else {
			
				// L vector: from point intersection to light source
				L = light->position - pHit;
			}

			Vector Lnormal = L;
			Lnormal = Lnormal.normalize();

			float distLight;
			bool inShadow = false;

			Vector I = ray.direction * -1;
			float cosI = I * nHit;

			float tNear = INFINITY;
			int index;

			// Ray hits from outside of object
			if (cosI > 0) {

				// avoid acne effect
				Vector shadowRayOrigin = pHit + Lnormal * EPSILON;

				// Secondary Shadow Ray
				Ray shadowRay = Ray(shadowRayOrigin, Lnormal);

				// check if object is in shadow or not
				for (int s = 0; s < num_objects; s++) {

					// Object in shadow
					if (scene->getObject(s)->intercepts(shadowRay, distLight) && (distLight < tNear)) {
						tNear = distLight;	// distance between intersection point and intersected object
						index = s;			// save object that has been intersected
						inShadow = true;
					}
				}

				// Calculate color when not in shadow. Else pixel is not colored
				if (!inShadow){
						Vector H = (Lnormal + I).normalize();

						// heuristic to calculate attenuation index
						float k1 = 1.25;
						float katt = 1 / (k1 * num_lights); // attenuation index

						Color diff = scene->getLight(j)->color * object->GetMaterial()->GetDiffuse() * object->GetMaterial()->GetDiffColor() * max((nHit * Lnormal), 0.0f);
						Color spec = scene->getLight(j)->color * object->GetMaterial()->GetSpecular() * object->GetMaterial()->GetSpecColor() * pow(max((nHit * H), 0.0f), object->GetMaterial()->GetShine());
						color += diff + (spec * katt);
				}
			}
		
		}

		if (depth >= MAX_DEPTH) {
			return scene->GetBackgroundColor();
		}


		Vector V = ray.direction * -1;
		/*
		// object is transparent. Compute REFRACTION ray
		if (transmitanceFlag != 0) {

			Vector refractionDir;
			Vector nAux;

			float cosi = clamp(V * nHit, -1, 1);

			float etai = 1;		// refraction index of the medium the ray is in before entering the second medium
			float etat = ior_1; // refraction index of the object
			
			// Ray is inside object
			if (cosi < 0) {
				cosi = -cosi;
			}
			// Ray is outside object
			else {
				std::swap(etai, etat);
				nAux = nHit * -1;
			}

			float eta = etai / etat; // Snells Law: n_1 / n_2

			// Compute wether incident light is completelly reflected rather than refracted
			float k = 1 - eta * eta * (1 - cosi * cosi);

			// total internal reflection. There is no refraction
			if (k < 0) {
				refractionDir = Vector(0,0,0).normalize();
			}
			else {
				refractionDir = (V*eta + nHit*(eta * cosi - sqrtf(k))).normalize();
			}

			Vector refractionOrigin;

			// avoid acne effect
			if (nAux * refractionDir < 0) {
				refractionOrigin = pHit - nAux * EPSILON;
			}

			else {
				refractionOrigin = pHit + nAux * EPSILON;
			}

			Ray rayRefraction = Ray(refractionOrigin, refractionDir);
			Color refractionColor = rayTracing(rayRefraction, depth + 1, ior_1, offsetX, offsetY);

			float kReflection; // refraction coefficient
			fresnel(V, nHit, ior_1, kReflection);
			color += refractionColor * (1 - kReflection);

		}
		*/

		// object is reflective like. Compute REFLECTION ray
		if (reflectiveFlag > 0) {
			// float kReflection; // reflection coefficient - calculado so na refracao

			Vector reflectionDir = nHit*(V*nHit)*2 - V; // 2(Vn)n - V
			Vector reflectionOrigin;

			// avoid acne efffect
			if (reflectionDir * nHit < 0) {
				reflectionOrigin = pHit - nHit * EPSILON;
			} else {
				reflectionOrigin = pHit + nHit * EPSILON;
			}

			Ray rayReflection = Ray(reflectionOrigin, reflectionDir);
			Color reflectionColor = rayTracing(rayReflection, depth + 1, ior_1, offsetX, offsetY);

			//fresnel(V, nHit, ior_1, kReflection);- so para refracao
			color += reflectionColor * object->GetMaterial()->GetReflection() * object->GetMaterial()->GetSpecColor();
		}
		return color;
	}
	else {
		return scene->GetBackgroundColor();
	}

}

Vector sample_unit_disk() {
	Vector p;
	do {
		p = Vector(rand_float(), rand_float(), 0.0); // z component is 0, pixel has to be in lenses
		p = p * 2 - Vector(1.0, 1.0, 0.0);			 // make sure that pixel is inside disk
	} while (p * p >= 1.0);
	return p;
}

// Render function by primary ray casting from the eye towards the scene's objects

void renderScene()
{
	int index_pos = 0;
	int index_col = 0;
	unsigned int counter = 0;

	if (drawModeEnabled) {
		glClear(GL_COLOR_BUFFER_BIT);
		scene->GetCamera()->SetEye(Vector(camX, camY, camZ));  //Camera motion
	}

	set_rand_seed(time(NULL));

	
	// Soft Shadows without antialiasing
	if (SOFTSHADOWS && !ANTIALIASING) {
		vector<Light*> new_lights;

		int num_lights = scene->getNumLights();
		float step = LIGHT_SIDE / NSAMPLES;
		float start = -LIGHT_SIDE / 2 + step / 2;
		float end = LIGHT_SIDE / 2;

		for (int l = 0; l < num_lights; l++) {
			Light* light = scene->getLight(l);
			Color colorAvg = light->color / pow(NSAMPLES, 2);

			for (float i = start; i < end; i = i + step) {
				for (float k = start; k < end; k = k + step) {
					Vector pos = Vector(light->position.x + i, light->position.y + k, light->position.z);

					Light* newLight = new Light(pos, colorAvg);

					// new scene lights is an area of finite point lights
					new_lights.push_back(newLight);
				}
			}
		}
		scene->setLights(new_lights);
	}
	

	for (int y = 0; y < RES_Y; y++)
	{
		for (int x = 0; x < RES_X; x++)
		{
			Color color = Color();
			Vector pixel;  //viewport coordinates

			
			// multiple primary rays per pixel
			if (ANTIALIASING) {

				// Jittering method
				for (int pi = 0; pi < NSAMPLES; pi++) {
					for (int pj = 0; pj < NSAMPLES; pj++) {
						pixel.x = x + ((pi + rand_float()) / NSAMPLES);
						pixel.y = y + ((pj + rand_float()) / NSAMPLES);

						if (DOF) {
							Vector lens_sample = Vector(
								sample_unit_disk().x * scene->GetCamera()->GetAperture(),
								sample_unit_disk().y * scene->GetCamera()->GetAperture(), 
								0.0f
							);

							Ray ray = scene->GetCamera()->PrimaryRay(lens_sample, pixel);   //function from camera.h
							color = color + rayTracing(ray, 1, 0.5, pi, pj).clamp();
						}
						else {
							Ray ray = scene->GetCamera()->PrimaryRay(pixel);
							color = color + rayTracing(ray, 1, 1.0, pi, pj).clamp();
						}
						
					}
				}
				color = color / (NSAMPLES * NSAMPLES);
			}

			// No antialiasing. One primary ray per pixel
			else {
			
				pixel.x = x + 0.5f;
				pixel.y = y + 0.5f;

				//YOUR 2 FUNTIONS:
				Ray ray = scene->GetCamera()->PrimaryRay(pixel);   //function from camera.h
				color = rayTracing(ray, 1, 1.0, 0, 0).clamp();	   // last two arguments = no offset
			}

			img_Data[counter++] = u8fromfloat((float)color.r());
			img_Data[counter++] = u8fromfloat((float)color.g());
			img_Data[counter++] = u8fromfloat((float)color.b());

			if (drawModeEnabled) {
				vertices[index_pos++] = (float)x;
				vertices[index_pos++] = (float)y;
				colors[index_col++] = (float)color.r();

				colors[index_col++] = (float)color.g();

				colors[index_col++] = (float)color.b();
			}
		}
	}

	if (drawModeEnabled) {
		drawPoints();
		glutSwapBuffers();
	}
	else {
		printf("Terminou o desenho!\n");
		if (saveImgFile("RT_Output.png") != IL_NO_ERROR) {
			printf("Error saving Image file\n");
			exit(0);
		}
		printf("Image file created\n");
	}
}


///////////////////////////////////////////////////////////////////////  SETUP     ///////////////////////////////////////////////////////

void setupCallbacks()
{
	glutKeyboardFunc(processKeys);
	glutCloseFunc(cleanup);
	glutDisplayFunc(renderScene);
	glutReshapeFunc(reshape);
	glutMouseFunc(processMouseButtons);
	glutMotionFunc(processMouseMotion);
	glutMouseWheelFunc(mouseWheel);

	glutIdleFunc(renderScene);
	glutTimerFunc(0, timer, 0);
}
void init(int argc, char* argv[])
{
	// set the initial camera position on its spherical coordinates
	Eye = scene->GetCamera()->GetEye();
	camX = Eye.x;
	camY = Eye.y;
	camZ = Eye.z;
	r = Eye.length();
	beta = asinf(camY / r) * 180.0f / 3.14f;
	alpha = atanf(camX / camZ) * 180.0f / 3.14f;

	setupGLUT(argc, argv);
	setupGLEW();
	std::cerr << "CONTEXT: OpenGL v" << glGetString(GL_VERSION) << std::endl;
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	createShaderProgram();
	createBufferObjects();
	setupCallbacks();
}


void init_scene(void)
{
	char scenes_dir[70] = "P3D_Scenes/";
	char input_user[50];
	char scene_name[70];

	scene = new Scene();

	if (P3F_scene) {  //Loading a P3F scene

		while (true) {
			cout << "Input the Scene Name: ";
			cin >> input_user;
			strcpy_s(scene_name, sizeof(scene_name), scenes_dir);
			strcat_s(scene_name, sizeof(scene_name), input_user);

			ifstream file(scene_name, ios::in);
			if (file.fail()) {
				printf("\nError opening P3F file.\n");
			}
			else
				break;
		}

		scene->load_p3f(scene_name);
		printf("Scene loaded.\n\n");
	}
	else {
		printf("Creating a Random Scene.\n\n");
		scene->create_random_scene();
	}


	RES_X = scene->GetCamera()->GetResX();
	RES_Y = scene->GetCamera()->GetResY();
	printf("\nResolutionX = %d  ResolutionY= %d.\n", RES_X, RES_Y);

	// Pixel buffer to be used in the Save Image function
	img_Data = (uint8_t*)malloc(3 * RES_X*RES_Y * sizeof(uint8_t));
	if (img_Data == NULL) exit(1);

	Accel_Struct = scene->GetAccelStruct();   //Type of acceleration data structure

	if (Accel_Struct == GRID_ACC) {
		grid_ptr = new Grid();
		vector<Object*> objs;
		int num_objects = scene->getNumObjects();

		for (int o = 0; o < num_objects; o++) {
			objs.push_back(scene->getObject(o));
		}
		grid_ptr->Build(objs);
		printf("Grid built.\n\n");
	}
	else if (Accel_Struct == BVH_ACC) {
		vector<Object*> objs;
		int num_objects = scene->getNumObjects();
		bvh_ptr = new BVH();

		for (int o = 0; o < num_objects; o++) {
			objs.push_back(scene->getObject(o));
		}
		bvh_ptr->Build(objs);
		printf("BVH built.\n\n");
	}
	else
		printf("No acceleration data structure.\n\n");

	unsigned int spp = scene->GetSamplesPerPixel();
	if (spp == 0)
		printf("Whitted Ray-Tracing\n");
	else
		printf("Distribution Ray-Tracing\n");

}

int main(int argc, char* argv[])
{
	//Initialization of DevIL 
	if (ilGetInteger(IL_VERSION_NUM) < IL_VERSION)
	{
		printf("wrong DevIL version \n");
		exit(0);
	}
	ilInit();

	int 
		ch;
	if (!drawModeEnabled) {

		do {
			init_scene();

			auto timeStart = std::chrono::high_resolution_clock::now();
			renderScene();  //Just creating an image file
			auto timeEnd = std::chrono::high_resolution_clock::now();
			auto passedTime = std::chrono::duration<double, std::milli>(timeEnd - timeStart).count();
			printf("\nDone: %.2f (sec)\n", passedTime / 1000);
			if (!P3F_scene) break;
			cout << "\nPress 'y' to render another image or another key to terminate!\n";
			delete(scene);
			free(img_Data);
			ch = _getch();
		} while((toupper(ch) == 'Y')) ;
	}

	else {   //Use OpenGL to draw image in the screen
		printf("OPENGL DRAWING MODE\n\n");
		init_scene();
		size_vertices = 2 * RES_X*RES_Y * sizeof(float);
		size_colors = 3 * RES_X*RES_Y * sizeof(float);
		vertices = (float*)malloc(size_vertices);
		if (vertices == NULL) exit(1);
		colors = (float*)malloc(size_colors);
		if (colors == NULL) exit(1);
		memset(colors, 0, size_colors);

		/* Setup GLUT and GLEW */
		init(argc, argv);
		glutMainLoop();
	}

	free(colors);
	free(vertices);
	printf("Program ended normally\n");
	exit(EXIT_SUCCESS);
}
///////////////////////////////////////////////////////////////////////