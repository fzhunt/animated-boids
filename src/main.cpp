/**********************************/
/* Lighting					      
   (C) Bedrich Benes 2021         
   Diffuse and specular per fragment.
   bbenes@purdue.edu               */
/**********************************/

#include <algorithm>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <exception>
#include <stdexcept>
#include <string.h>
#include <iostream>
#define _USE_MATH_DEFINES
#include <math.h>
#include <time.h>
#include <string>
#include <vector>			//Standard template library class
#include <GL/glew.h>
#include <GL/glut.h>
//glm
#include <glm/glm.hpp>
#include <glm/ext.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/half_float.hpp>
#include "shaders.h"    
#include "shapes.h"    
#include "lights.h"    

#pragma warning(disable : 4996)
#pragma comment(lib, "glew32.lib")
#pragma comment(lib, "glut32.lib")

using namespace std;

bool needRedisplay=false;
ShapesC* sphere;
ShapesC* cone;
bool state = true;


/*
* use the view function, they're all supposed to be points, so destination is the point that'll be ending
* make sure that the bird is originally facing positive z, or if that doesn't work do x
* can pull things out of blendr pretty easily, maybe maya would be easier to learn
* view = glm::lookAt(glm::vec3(420.f, 420.f, 420.f),//eye
		glm::vec3(0, 0, 0),  //destination
		glm::vec3(0, 1, 0)); //up
 * if it happens to be 
*/



float max_x = 400.0f;
float max_y = 400.0f;
float max_z = 400.0f;

int margin = 20;

float factorStep = 0.1f;
int otherStep = 10;

float maxFactor = 1.0f;

float turn = 0.5f;
bool ignoreObjects = false;

//shader program ID
GLuint shaderProgram;
GLfloat ftime=0.f;
float dt = 0.7f;
glm::mat4 view=glm::mat4(1.0);
glm::mat4 proj=glm::perspective(80.0f,//fovy
				  		        1.0f,//aspect
						        0.01f,1000.f); //near, far
class ShaderParamsC
{
public:
	GLint modelParameter;		//modeling matrix
	GLint modelViewNParameter;  //modeliview for normals
	GLint viewParameter;		//viewing matrix
	GLint projParameter;		//projection matrix
	//material
	GLint kaParameter;			//ambient material
	GLint kdParameter;			//diffuse material
	GLint ksParameter;			//specular material
	GLint shParameter;			//shinenness material
} params;

class Boid {
public:
	int id;
	glm::vec3 position;
	glm::vec3 velocity;
	bool object;

	Boid() {
		position = glm::vec3(0);
		velocity = glm::vec3(1);
		id = 0;
		object = false;
	}

	Boid(glm::vec3 p, int d) {
		position = p;
		velocity = glm::vec3(0);
		object = true;
	}

	Boid(glm::vec3 p, glm::vec3 v, int d) {
		position = p;
		velocity = v;
		id = d;
		object = false;
	}
};


class Flock {
public:
	vector<Boid*> boidList;
	float radius;
	float angle;

	glm::vec3 sumOfPos = glm::vec3(0);
	glm::vec3 sumOfVelocity = glm::vec3(0);

	float avoidDistance = 10.0f;
	float objectAvoid = 20.0f;

	float separationFactor;
	float cohesionFactor;
	float alignmentFactor;

	float maxspeed = 4.0f;

	Flock() {
		radius = 30.0f;
		angle = 90.0f;
		vector<Boid*> f = {};
		boidList = f;
		separationFactor = 1.0f;
		cohesionFactor = 0.6f;
		alignmentFactor = 0.2f;

	}

	Flock(vector<Boid*> flk, float r, float a) {
		boidList = flk;
		radius = r;
		angle = a;
		separationFactor = 0.5f;
		cohesionFactor = 0.5f;
		alignmentFactor = 0.2f;
	}

	float getAngle(Boid* boid, glm::vec3 dest) {
		glm::vec3 u = glm::normalize(boid->velocity);
		glm::vec3 dir = glm::normalize(dest - boid->position);
		float dot = glm::dot(dir, u);
		float angle = acos(dot) * 180.0 / M_PI;
		return angle;
	}

	void printVals() {
		cout << "-------Flock parameters-------" << endl;
		cout << "Separation (s): " << separationFactor << endl;
		cout << "Cohesion (c): " << cohesionFactor << endl;
		cout << "Alignment (a): " << alignmentFactor << endl;
		cout << "View radius (r): " << radius << endl;
		cout << "View angle (t): " << angle << endl;
		cout << "Avoidance radius (d): " << avoidDistance << endl;
		cout << "Max speed (v): " << maxspeed << endl;
		cout << "Object avoidance " << !ignoreObjects << " (o or O)" << endl;
		cout << "Reseed Boids using q or Q" << endl;
		cout << "Reset Parameters using n or N" << endl;
	}

	glm::vec3 separation(Boid* boid) {
		if (!boid->object) {
			glm::vec3 res(0);
			for (Boid* boid2 : boidList) {
				if (boid2->id != boid->id) {
					if (ignoreObjects && boid2->object) {}
					else if ((boid2->object) && (glm::distance(boid->position, boid2->position) < objectAvoid
						&& getAngle(boid, boid2->position) < angle)) {
						res = res - (boid->position - boid2->position);
					}
					else if (glm::distance(boid->position, boid2->position) < avoidDistance &&
						getAngle(boid, boid2->position) < angle) {
						res = res - (boid2->position - boid->position);
						//cout << "close" << endl;
					}
				}
			}
			return res == glm::vec3(0) ? res : glm::normalize(res);
		}
	}

	glm::vec3 alignment(Boid* boid) {
		if (!boid->object) {
			int neighbors = 0;

			glm::vec3 avgV(0);
			bool found = false;

			if (boidList.size() == 1) return avgV;

			for (Boid* boid2 : boidList) {
				if (boid2->id != boid->id && !boid2->object) {
					if (glm::distance(boid->position, boid2->position) < radius &&
						getAngle(boid, boid2->position) < angle) {
						avgV += boid2->velocity;
						found = true;
						neighbors++;
					}
				}
			}
			if (!found) return glm::vec3(0);

			avgV = avgV * (1.0f / neighbors);
			glm::vec3 force = avgV - boid->velocity;
			return force == glm::vec3(0) ? force : glm::normalize(force);
		}
	}

	glm::vec3 cohesion(Boid* boid) {
		if (!boid->object) {
			bool found = false;
			int neighbors = 0;

			glm::vec3 center(0);
			if (boidList.size() == 1) return center;

			for (Boid* boid2 : boidList) {
				if (boid2->id != boid->id && !boid2->object) {
					if (glm::distance(boid->position, boid2->position) < radius &&
						glm::distance(boid->position, boid2->position) > avoidDistance &&
						getAngle(boid, boid2->position) < angle) {
						center += boid2->position;
						found = true;
						neighbors++;
					}

				}
			}

			if (!found) return glm::vec3(0);

			center = center * (1.0f / neighbors);

			glm::vec3 force = center - boid->position;

			return force == glm::vec3(0) ? force : glm::normalize(force);
		}
	}

	void applyRules() {
		//findSumOfPos();
		//findSumOfVelocity();
		for (Boid* boid : boidList) {
			glm::vec3 sep = separation(boid) * separationFactor;
			glm::vec3 coh = cohesion(boid) * cohesionFactor;
			glm::vec3 align = alignment(boid) * alignmentFactor;

			glm::vec3 totalForce = sep + coh + align;

			//cout << "separation: " << glm::to_string(sep) << endl;
			//cout << "cohesion: " << glm::to_string(coh) << endl;
			//cout << "alignment: " << glm::to_string(align) << endl;

			//cout << "added force: " << glm::to_string(totalForce) << endl;
			if (totalForce != glm::vec3(0)) {
				//totalForce = glm::normalize(totalForce);
			}
	
			boid->velocity += totalForce;

			//cout << "velocity: " << glm::to_string(boid->velocity) << endl;
			
			boid->velocity = (glm::normalize(boid->velocity)) * maxspeed;

			if (boid->object) {
				boid->velocity = glm::vec3(0);
			}
			boid->position += boid->velocity * dt;

			
			if (boid->position.x > max_x - margin) {
				boid->velocity.x -= turn;
			}
			if (boid->position.y > max_y - margin) {
				boid->velocity.y -= turn;
			}
			if (boid->position.z > max_z - margin) {
				boid->velocity.z -= turn;
			}

			if (boid->position.x < 0 + margin) {
				boid->velocity.x += turn;
			}
			if (boid->position.y < 0 + margin) {
				boid->velocity.y += turn;
			}
			if (boid->position.z < 0 + margin) {
				boid->velocity.z += turn;
			}
			
		}
	}

	void setSeparation(float val) {
		separationFactor = val;
	}

	void setCohesion(float val) {
		cohesionFactor = val;
	}

	void setAlignment(float val) {
		alignmentFactor = val;
	}

	void setMaxSpeed(float val) {
		maxspeed = val;
	}

	void setRadius(float r) {
		radius = r;
	}

	void setAngle(float a) {
		angle = a;
	}

	void setAvoidanceDistance(float d) {
		avoidDistance = d;
	}


	//helper function for Cone Rendering Purposes
	glm::vec3 perp(const glm::vec3 v) {
		float min = fabsf(v.x);
		glm::vec3 cardinalAxis(1, 0, 0);

		if (fabsf(v.y) < min) {
			min = fabsf(v.y);
			cardinalAxis = glm::vec3(0, 1, 0);
		}

		if (fabsf(v.z) < min) {
			cardinalAxis = glm::vec3(0, 0, 1);
		}

		return glm::cross (v, cardinalAxis);
	}

	/*
	* Function for drawing a cone
	* D is a normalized vector from the base to the apex point
	* a is the apex point
	* h is the height of the cone
	* rd is the radius of the base
	* n is the number of slices the cone has
	*/

	void drawCone(const glm::vec3 d, const glm::vec3 a,
		const float h, const float rd, const int n) {
		glm::vec3 c = a + (-d * h);
		glm::vec3 e0 = perp(d);
		glm::vec3 e1 = glm::cross(e0, d);
		float angInc = 360.0 / n * (M_PI / 180.0);

		// calculate points around directrix
		std::vector<glm::vec3> pts;
		for (int i = 0; i < n; ++i) {
			float rad = angInc * i;
			glm::vec3 p = c + (((e0 * cos(rad)) + (e1 * sin(rad))) * rd);
			pts.push_back(p);
		}
		glColor3f(0, 0, 1);
		// draw cone top
		glBegin(GL_TRIANGLE_FAN);
		glVertex3f(a.x, a.y, a.z);
		for (int i = 0; i < n; ++i) {
			glVertex3f(pts[i].x, pts[i].y, pts[i].z);
		}
		glEnd();
		glFlush();
		// draw cone bottom
		glBegin(GL_TRIANGLE_FAN);
		glVertex3f(c.x, c.y, c.z);
		for (int i = n - 1; i >= 0; --i) {
			glVertex3f(pts[i].x, pts[i].y, pts[i].z);
		}
		glEnd();
		glFlush();
		glutPostRedisplay();
	}

	void renderFlock() {
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_CLEAR_VALUE);
		//sphere->Render();
		for (Boid* boid : boidList) {
			glutPostRedisplay();
			if (boid->object) {
				//cout << "drawing at " << boid->position.x << ", " << boid->position.y << ", " << boid->position.z << endl;

				glm::mat4 m = glm::translate(glm::mat4(1.0), boid->position);
				//m = glm::scale(m, glm::normalize(boid->velocity) * 2.0f);

				//sphere->SetModel(m);
				cone->SetModel(m);
				glm::mat3 modelViewN = glm::mat3(view * m);
				modelViewN = glm::transpose(glm::inverse(modelViewN));
				//sphere->SetModelViewN(modelViewN);
				//sphere->Render();
				cone->SetModelViewN(modelViewN);
				cone->Render();
			}
			else {
				glm::mat4 m = glm::translate(glm::mat4(1.0), boid->position);
				//m = glm::scale(m, glm::normalize(boid->velocity) * 2.0f);

				//sphere->SetModel(m);
				cone->SetModel(m);
				glm::mat3 modelViewN = glm::mat3(view * m);
				modelViewN = glm::transpose(glm::inverse(modelViewN));
				//sphere->SetModelViewN(modelViewN);
				//sphere->Render();
				cone->SetModelViewN(modelViewN);
				cone->Render();
				
				//drawCone(glm::normalize(boid->velocity), boid->position + boid->velocity, 3, 2, 8);
			}
		}
		
		//for (Boid* boid : boidList) {
		//	glm::mat4 m = glm::translate(glm::mat4(1.0), boid->position);
		//	//m = glm::scale(m, glm::normalize(boid.velocity) * 2.0f);

		//	sphere->SetModel(m);
		//	glm::mat3 modelViewN = glm::mat3(view * m);
		//	modelViewN = glm::transpose(glm::inverse(modelViewN));
		//	sphere->SetModelViewN(modelViewN);
		//	sphere->Render();
		//}

		applyRules();
	}

};

struct delete_ptr {
	template <typename P>
	void operator () (P p) {
		delete p;
	}
};

Flock flock;



LightC light;


//the main window size
GLint wWindow=800;
GLint hWindow=800;

float sh=1;

/*********************************
Some OpenGL-related functions
**********************************/

//called when a window is reshaped
void Reshape(int w, int h)
{
  glViewport(0,0,w, h);       
  glEnable(GL_DEPTH_TEST);
  wWindow=w;
  hWindow=h;
}

void setupFlock() {
	for (int i = 0; i < 1000; i++) {
		float x = ((float)rand() / (float)RAND_MAX) * 2 - 1;
		float y = ((float)rand() / (float)RAND_MAX) * 2 - 1;
		float z = ((float)rand() / (float)RAND_MAX) * 2 - 1;

		float a = ((float)rand() / (float)RAND_MAX);
		float b = ((float)rand() / (float)RAND_MAX);
		float c = ((float)rand() / (float)RAND_MAX);

		//flock.boidList.push_back(new Boid(glm::vec3(i * 20 + 50, k * 20 + 50, j * 20 + 50), glm::vec3(x, y, z), id));
		if (i > 2) {
			flock.boidList.push_back(new Boid(glm::vec3(a * max_x, b * max_y, c * max_z), glm::vec3(x, y, z), i));
		}
		else {
			flock.boidList.push_back(new Boid(glm::vec3(a * max_x, b * max_y, c * max_z), i));
		}
	}
				

}



//the main rendering function
void RenderObjects()
{
	const int range=3;
	glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
	glDisable(GL_CULL_FACE);
	glColor3f(0,0,0);
	glPointSize(2);
	glLineWidth(1);
	//set the projection and view once for the scene
	glUniformMatrix4fv(params.projParameter,1,GL_FALSE,glm::value_ptr(proj));
	//view=glm::lookAt(glm::vec3(25*sin(ftime/40.f),5.f,15*cos(ftime/40.f)),//eye
	//			     glm::vec3(0,0,0),  //destination
	//			     glm::vec3(0,1,0)); //up
	/*view = glm::lookAt(glm::vec3(260.f, 260.f, 260.f),//eye
				     glm::vec3(0,0,0),  //destination
				     glm::vec3(0,1,0)); //u
					 */
	view = glm::lookAt(glm::vec3(420.f, 420.f, 420.f),//eye
		glm::vec3(0, 0, 0),  //destination
		glm::vec3(0, 1, 0)); //up

	glUniformMatrix4fv(params.viewParameter,1,GL_FALSE,glm::value_ptr(view));
//set the light
	static glm::vec4 pos = glm::vec4(400,400,400,1);
	//pos.x=20*sin(ftime/12);pos.y=-10;pos.z=20*cos(ftime/12);pos.w=1;
	light.SetPos(pos);
	light.SetShaders();
	
	flock.renderFlock();

	/*glColor3f(0.3, 0.5, 0.7);
	glBegin(GL_TRIANGLES);
	glVertex3f(0, -50, 0);
	glVertex3f(200, -50, 200);
	glVertex3f(0, -50, 200);
	//glVertex3f(0, -50, 0);
	//glVertex3f(400, -50, 400);
	//glVertex3f(400, -50, 0);
	glEnd();
	glFlush();
	glColor3f(0.7, 0.7, 0.7);
	*/
}
	
void Idle(void)
{
  glClearColor(0.1,0.1,0.1,1);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
  ftime+=0.05;
  glUseProgram(shaderProgram);
  RenderObjects();
  glutSwapBuffers();  
}

void Display(void)
{

}

//keyboard callback
void Kbd(unsigned char a, int x, int y)
{
	switch(a)
	{
 	  case 27 : exit(0);break;
	  case 'c':
		  if (flock.cohesionFactor - factorStep < 0) {
			  flock.setCohesion(0);
			  break;
		  }
		  flock.setCohesion(flock.cohesionFactor - factorStep);
		  cout << "-------Changed Parameter-------" << endl;
		  cout << "Cohesion: " << flock.cohesionFactor << endl;
		  break;
	  case 'C':
		  if (flock.cohesionFactor + factorStep > maxFactor) break;
		  flock.setCohesion(flock.cohesionFactor + factorStep);
		  cout << "-------Changed Parameter-------" << endl;
		  cout << "Cohesion: " << flock.cohesionFactor << endl;
		  break;
	  case 's':
		  if (flock.separationFactor - factorStep < 0) {
			  flock.setSeparation(0);
			  break;
		  }
		  flock.setSeparation(flock.separationFactor - factorStep);
		  cout << "-------Changed Parameter-------" << endl;
		  cout << "Separation: " << flock.separationFactor << endl;
		  break;
	  case 'S':
		  if (flock.separationFactor + factorStep > maxFactor) break;
		  flock.setSeparation(flock.separationFactor + factorStep);
		  cout << "-------Changed Parameter-------" << endl;
		  cout << "Separation: " << flock.separationFactor << endl;
		  break;
	  case 'a':
		  if (flock.alignmentFactor - factorStep < 0) {
			  flock.setAlignment(0);
			  break;
		  }
		  flock.setAlignment(flock.alignmentFactor - factorStep);
		  cout << "-------Changed Parameter-------" << endl;
		  cout << "Alignment: " << flock.alignmentFactor << endl;
		  break;
	  case 'A':
		  if (flock.alignmentFactor + factorStep > maxFactor) break;
		  flock.setAlignment(flock.alignmentFactor + factorStep);
		  cout << "-------Changed Parameter-------" << endl;
		  cout << "Alignment: " << flock.alignmentFactor << endl;
		  break;
	  case 'r':
		  if (flock.radius - otherStep < 0) break;
		  flock.setRadius(flock.radius - otherStep);
		  cout << "-------Changed Parameter-------" << endl;
		  cout << "View radius: " << flock.radius << endl;
		  break;
	  case 'R':
		  if (flock.radius + otherStep > 100) break;
		  flock.setRadius(flock.radius + otherStep);
		  cout << "-------Changed Parameter-------" << endl;
		  cout << "View radius: " << flock.radius << endl;
		  break;
	  case 't':
		  if (flock.angle - otherStep < 0) break;
		  flock.setAngle(flock.angle - otherStep);
		  cout << "-------Changed Parameter-------" << endl;
		  cout << "View angle: " << flock.angle << endl;
		  break;
	  case 'T':
		  if (flock.angle + otherStep > 360) break;
		  flock.setAngle(flock.angle + otherStep);
		  cout << "-------Changed Parameter-------" << endl;
		  cout << "View angle: " << flock.angle << endl;
		  break;
	  case 'd':
		  if (flock.avoidDistance - otherStep < 0) break;
		  flock.setAvoidanceDistance(flock.avoidDistance - otherStep);
		  cout << "-------Changed Parameter-------" << endl;
		  cout << "Avoidance radius: " << flock.avoidDistance << endl;
		  break;
	  case 'D':
		  if (flock.avoidDistance + otherStep > flock.radius) break;
		  flock.setAvoidanceDistance(flock.avoidDistance + otherStep);
		  cout << "-------Changed Parameter-------" << endl;
		  cout << "Avoidance radius: " << flock.avoidDistance << endl;
		  break;
	  case 'v':
		  if (flock.maxspeed - factorStep < 0) break;
		  flock.setMaxSpeed(flock.maxspeed - factorStep);
		  cout << "-------Changed Parameter-------" << endl;
		  cout << "Max speed: " << flock.maxspeed << endl;
		  break;
	  case 'V':
		  if (flock.maxspeed + factorStep > 10.0f) break;
		  flock.setMaxSpeed(flock.maxspeed + factorStep);
		  cout << "-------Changed Parameter-------" << endl;
		  cout << "Max speed: " << flock.maxspeed << endl;
		  break;
	  case 'q':
	  case 'Q':
		  for_each(flock.boidList.begin(), flock.boidList.end(), delete_ptr());
		  flock.boidList.clear();
		  setupFlock();
		  break;
	  case 'n':
	  case 'N': {
		  flock.setRadius(30.0f);
		  flock.setAngle(90.0f);
		  flock.setSeparation(1.0f);
		  flock.setCohesion(0.6f);
		  flock.setAlignment(0.2f);
		  flock.setAvoidanceDistance(10);
		  flock.setMaxSpeed(4);
		  cout << "Reset the parameters!" << endl;
		  break;
	  }
	  case 'o': 
	  case 'O': {ignoreObjects = !ignoreObjects; break; }
	  case 'g': 
	  case 'G': {cone->SetKd(glm::vec3(0,1,0));break;} // these were all sphere before cone was added
	  case 'b': 
	  case 'B': {cone->SetKd(glm::vec3(0,0,1));break;}
	  case 'w': 
	  case 'W': {cone->SetKd(glm::vec3(0.7,0.7,0.7)); state = !state; break;}
	  case '+': {cone->SetSh(sh+=1);break;}
	  case '-': {cone->SetSh(sh-=1);if (sh<1) sh=1;break;}
	}
	flock.printVals();
	//cout << "shineness="<<sh<<endl;
	glutPostRedisplay();
}


//special keyboard callback
void SpecKbdPress(int a, int x, int y)
{
   	switch(a)
	{
 	  case GLUT_KEY_LEFT  : 
		  {
			  break;
		  }
	  case GLUT_KEY_RIGHT : 
		  {
			break;
		  }
 	  case GLUT_KEY_DOWN    : 
		  {
			break;
		  }
	  case GLUT_KEY_UP  :
		  {
			break;
		  }

	}
	glutPostRedisplay();
}

//called when a special key is released
void SpecKbdRelease(int a, int x, int y)
{
	switch(a)
	{
 	  case GLUT_KEY_LEFT  : 
		  {
			  break;
		  }
	  case GLUT_KEY_RIGHT : 
		  {
			  break;
		  }
 	  case GLUT_KEY_DOWN  : 
		  {
			break;
		  }
	  case GLUT_KEY_UP  :
		  {
			break;
		  }
	}
	glutPostRedisplay();
}


void Mouse(int button,int state,int x,int y)
{
	cout << "Location is " << "[" << x << "'" << y << "]" << endl;
}


void InitializeProgram(GLuint *program)
{
	std::vector<GLuint> shaderList;

//load and compile shaders 	
	shaderList.push_back(CreateShader(GL_VERTEX_SHADER,   LoadShader("shaders/phong.vert")));
	shaderList.push_back(CreateShader(GL_FRAGMENT_SHADER, LoadShader("shaders/phong.frag")));

//create the shader program and attach the shaders to it
	*program = CreateProgram(shaderList);

//delete shaders (they are on the GPU now)
	std::for_each(shaderList.begin(), shaderList.end(), glDeleteShader);

	params.modelParameter=glGetUniformLocation(*program,"model");
	params.modelViewNParameter=glGetUniformLocation(*program,"modelViewN");
	params.viewParameter =glGetUniformLocation(*program,"view");
	params.projParameter =glGetUniformLocation(*program,"proj");
	//now the material properties
	params.kaParameter=glGetUniformLocation(*program,"mat.ka");
	params.kdParameter=glGetUniformLocation(*program,"mat.kd");
	params.ksParameter=glGetUniformLocation(*program,"mat.ks");
	params.shParameter=glGetUniformLocation(*program,"mat.sh");
	//now the light properties
	light.SetLaToShader(glGetUniformLocation(*program,"light.la"));
	light.SetLdToShader(glGetUniformLocation(*program,"light.ld"));
	light.SetLsToShader(glGetUniformLocation(*program,"light.ls"));
	light.SetLposToShader(glGetUniformLocation(*program,"light.lPos"));
}

void InitShapes(ShaderParamsC *params)
{
//create one unit sphere in the origin
	/*sphere = new SphereC(50, 50, 2.0f);
	sphere->SetKa(glm::vec3(0.1,0.1,0.1));
	sphere->SetKs(glm::vec3(0,0,1));
	sphere->SetKd(glm::vec3(0.7,0.7,0.7));
	sphere->SetSh(200);
	sphere->SetModel(glm::mat4(1.0));
	sphere->SetModelMatrixParamToShader(params->modelParameter);
	sphere->SetModelViewNMatrixParamToShader(params->modelViewNParameter);
	sphere->SetKaToShader(params->kaParameter);
	sphere->SetKdToShader(params->kdParameter);
	sphere->SetKsToShader(params->ksParameter);
	sphere->SetShToShader(params->shParameter);
	*/
	cone = new Cone();
	cone->SetKa(glm::vec3(0.1, 0.1, 0.1));
	cone->SetKs(glm::vec3(0, 0, 1));
	cone->SetKd(glm::vec3(0.7, 0.7, 0.7));
	cone->SetSh(200);
	cone->SetModel(glm::mat4(1.0));
	cone->SetModelMatrixParamToShader(params->modelParameter);
	cone->SetModelViewNMatrixParamToShader(params->modelViewNParameter);
	cone->SetKaToShader(params->kaParameter);
	cone->SetKdToShader(params->kdParameter);
	cone->SetKsToShader(params->ksParameter);
	cone->SetShToShader(params->shParameter);
}



int main(int argc, char **argv)
{ 
	setupFlock();

	/*flock.boidList.push_back(new Boid(glm::vec3(55, 50, 50), glm::vec3(0, 1, 0), 0));
	flock.boidList.push_back(new Boid(glm::vec3(50, 50, 55), glm::vec3(0, 1, 0), 1));
	flock.boidList.push_back(new Boid(glm::vec3(55, 45, 50), glm::vec3(0, 1, 0), 2));
	flock.boidList.push_back(new Boid(glm::vec3(50, 45, 55), glm::vec3(0, 1, 0), 3));*/
	



  glutInitDisplayString("stencil>=2 rgb double depth samples");
  glutInit(&argc, argv);
  glutInitWindowSize(wWindow,hWindow);
  glutInitWindowPosition(500,100);
  glutCreateWindow("Model View Projection GLSL");
  GLenum err = glewInit();
  if (GLEW_OK != err){
   fprintf(stderr, "Error: %s\n", glewGetErrorString(err));
  }
  glutDisplayFunc(Display);
  glutIdleFunc(Idle);
  glutMouseFunc(Mouse);
  glutReshapeFunc(Reshape);
  glutKeyboardFunc(Kbd); //+ and -
  glutSpecialUpFunc(SpecKbdRelease); //smooth motion
  glutSpecialFunc(SpecKbdPress);
  InitializeProgram(&shaderProgram);
  InitShapes(&params);
  flock.printVals();
  glutMainLoop();
  return 0;        
}
	