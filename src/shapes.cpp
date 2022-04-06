#include <stdio.h>
#include <iostream>
#include <string.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <string>
#include <vector>
#include <GL/glew.h>
#include <GL/glut.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include "shapes.h"

using namespace std;

//help procedure that send values from glm::vec3 to a STL vector of float
//used for creating VBOs
inline void AddVertex(vector <GLfloat> *a, const glm::vec3 *v)
{
	a->push_back(v->x);
	a->push_back(v->y);
	a->push_back(v->z);
}

void ShapesC::Render()
{
	cout << "Base class cannot render" << "\n";
}

void ShapesC::SetModel(glm::mat4 tmp)
{
	model=tmp;
}

void ShapesC::SetModelViewN(glm::mat3 tmp)
{
	modelViewN=tmp;
}


void ShapesC::SetModelMatrixParamToShader(GLuint uniform)
{
  modelParameter=uniform;
}

void ShapesC::SetModelViewNMatrixParamToShader(GLuint uniform)
{
  modelViewNParameter=uniform;
}


void ShapesC::SetColor(GLubyte r,GLubyte b,GLubyte g)
{
	color[0]=r;
	color[1]=g;
	color[2]=b;
}

void Cone::Render() {
	glBindVertexArray(vaID);
	//	glBindBuffer(GL_ARRAY_BUFFER, buffer);
	//	glEnableVertexAttribArray(0);
		//material properties
	glUniform3fv(kaParameter, 1, glm::value_ptr(ka));
	glUniform3fv(kdParameter, 1, glm::value_ptr(kd));
	glUniform3fv(ksParameter, 1, glm::value_ptr(ks));
	glUniform1fv(shParameter, 1, &sh);
	//model matrix
	glUniformMatrix4fv(modelParameter, 1, GL_FALSE, glm::value_ptr(model));
	//model for normals
	glUniformMatrix3fv(modelViewNParameter, 1, GL_FALSE, glm::value_ptr(modelViewN));
	glDrawArrays(GL_TRIANGLES, 0, 3 * points);
}

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

	return glm::cross(v, cardinalAxis);
}

void Cone::Generate(glm::vec3 d, glm::vec3 a, float h, float rd, int n)
{

	glm::vec3 c = a + (-d * h);
	glm::vec3 e0 = perp(d);
	glm::vec3 e1 = glm::cross(e0, d);
	float angInc = 360.0 / n * (M_PI / 180.0);

	// calculate points around directrix
	
	std::vector<glm::vec3> pts;
	for (int i = 0; i < n - 1; i+=2) {
		float rad = angInc * i;
		glm::vec3 p = c + (((e0 * cos(rad)) + (e1 * sin(rad))) * rd);
		pts.push_back(p);
	}

	for (int i = 0; i < pts.size(); ++i) {
		
		// draw top (funnel shaped part)	
		glm::vec3 nextPoint = glm::vec3(0);
		if (i + 1 == pts.size()) {
			nextPoint = pts[0];
		}
		else {
			nextPoint = pts[i + 1];
		}
		glm::vec3 normalv = glm::normalize(glm::cross(pts[i] - a, pts[i] - nextPoint));
		AddVertex(&vertex, &a);
		AddVertex(&normal, &normalv);
		AddVertex(&vertex, &pts[i]);
		AddVertex(&normal, &normalv);
		AddVertex(&vertex, &nextPoint);
		AddVertex(&normal, &normalv);
		
		// draw cone bottom		
		normalv = glm::normalize(-d);
		AddVertex(&vertex, &c);
		AddVertex(&normal, &normalv);
		AddVertex(&vertex, &pts[i]);
		AddVertex(&normal, &normalv);
		AddVertex(&vertex, &nextPoint);
		AddVertex(&normal, &normalv);
	}
}

void Cone::InitArrays() {
	points = vertex.size();
	normals = normal.size();

	//get the vertex array handle and bind it
	glGenVertexArrays(1, &vaID);
	glBindVertexArray(vaID);

	//the vertex array will have two vbos, vertices and normals
	glGenBuffers(2, vboHandles);
	GLuint verticesID = vboHandles[0];
	GLuint normalsID = vboHandles[1];

	//send vertices
	glBindBuffer(GL_ARRAY_BUFFER, verticesID);
	glBufferData(GL_ARRAY_BUFFER, points * sizeof(GLfloat), &vertex[0], GL_STATIC_DRAW);
	glVertexAttribPointer((GLuint)0, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(0);
	vertex.clear(); //no need for the vertex data, it is on the GPU now

//send normals
	glBindBuffer(GL_ARRAY_BUFFER, normalsID);
	glBufferData(GL_ARRAY_BUFFER, normals * sizeof(GLfloat), &normal[0], GL_STATIC_DRAW);
	glVertexAttribPointer((GLuint)1, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(1);
	normal.clear(); //no need for the normal data, it is on the GPU now
}

Cone::Cone() {
	Generate(glm::vec3(1, 0, 0), glm::vec3(0), 3, 2, 8);
	InitArrays();
}



void SphereC::Render()
{
	glBindVertexArray(vaID);
//	glBindBuffer(GL_ARRAY_BUFFER, buffer);
//	glEnableVertexAttribArray(0);
	//material properties
	glUniform3fv(kaParameter,1,glm::value_ptr(ka));
	glUniform3fv(kdParameter,1,glm::value_ptr(kd));
	glUniform3fv(ksParameter,1,glm::value_ptr(ks));
	glUniform1fv(shParameter,1,&sh);
	//model matrix
	glUniformMatrix4fv(modelParameter,1,GL_FALSE,glm::value_ptr(model));
	//model for normals
    glUniformMatrix3fv(modelViewNParameter,1,GL_FALSE,glm::value_ptr(modelViewN));
	glDrawArrays(GL_TRIANGLES, 0, 3*points);
}

void SphereC::Generate(int stacks, int slices, GLfloat r)
{
	glm::vec3 v;

	GLfloat deltaTheta=2*M_PI/(GLfloat)slices;
	GLfloat deltaPhi  =  M_PI/(GLfloat)stacks;

	for (GLint i=0;i<stacks;i++)
	{
		GLfloat phi=i*deltaPhi;
		for (GLint j=0;j<slices;j++)
		{
			GLfloat theta=j*deltaTheta;
//the first triangle
			v=glm::vec3(r*cos(theta)*sin(phi),
				        r*sin(theta)*sin(phi),
				        r*cos(phi));
			AddVertex(&vertex,&v); //add the vertex
			glm::normalize(v);     //normalize it 
			AddVertex(&normal,&v); //and add the normal vector
			v=glm::vec3(r*cos(theta+deltaTheta)*sin(phi),
				        r*sin(theta+deltaTheta)*sin(phi),
                        r*cos(phi));
			AddVertex(&vertex,&v); //add the vertex
			glm::normalize(v);     //normalize it 
			AddVertex(&normal,&v); //and add the normal vector
			v=glm::vec3(r*cos(theta)*sin(phi+deltaPhi),
				        r*sin(theta)*sin(phi+deltaPhi),
				        r*cos(phi+deltaPhi));
			AddVertex(&vertex,&v); //add the vertex
			glm::normalize(v);     //normalize it 
			AddVertex(&normal,&v); //and add the normal vector
//the second triangle
			v=glm::vec3(r*cos(theta+deltaTheta)*sin(phi),
				             r*sin(theta+deltaTheta)*sin(phi),
				             r*cos(phi));
			AddVertex(&vertex,&v); //add the vertex
			glm::normalize(v);     //normalize it 
			AddVertex(&normal,&v); //and add the normal vector
			v=glm::vec3(r*cos(theta)*sin(phi+deltaPhi),
				        r*sin(theta)*sin(phi+deltaPhi),
				        r*cos(phi+deltaPhi));
			AddVertex(&vertex,&v); //add the vertex
			glm::normalize(v);     //normalize it 
			AddVertex(&normal,&v); //and add the normal vector
			v=glm::vec3(r*cos(theta+deltaTheta)*sin(phi+deltaPhi),
				        r*sin(theta+deltaTheta)*sin(phi+deltaPhi),
				        r*cos(phi+deltaPhi));
			AddVertex(&vertex,&v); //add the vertex
			glm::normalize(v);     //normalize it 
			AddVertex(&normal,&v); //and add the normal vector
		}
	}
}

void SphereC::InitArrays()
{
	points=vertex.size();
	normals=normal.size();

//get the vertex array handle and bind it
	glGenVertexArrays(1,&vaID);
	glBindVertexArray(vaID);

//the vertex array will have two vbos, vertices and normals
	glGenBuffers(2, vboHandles);
	GLuint verticesID=vboHandles[0];
	GLuint normalsID= vboHandles[1];

//send vertices
	glBindBuffer(GL_ARRAY_BUFFER, verticesID);
	glBufferData(GL_ARRAY_BUFFER, points*sizeof(GLfloat), &vertex[0], GL_STATIC_DRAW);
	glVertexAttribPointer((GLuint)0, 3, GL_FLOAT, GL_FALSE, 0, 0); 
	glEnableVertexAttribArray(0);
	vertex.clear(); //no need for the vertex data, it is on the GPU now

//send normals
	glBindBuffer(GL_ARRAY_BUFFER, normalsID);
	glBufferData(GL_ARRAY_BUFFER, normals*sizeof(GLfloat), &normal[0], GL_STATIC_DRAW);
	glVertexAttribPointer((GLuint)1, 3, GL_FLOAT, GL_FALSE, 0, 0); 
	glEnableVertexAttribArray(1);
	normal.clear(); //no need for the normal data, it is on the GPU now
}

SphereC::SphereC()
{
	Generate(55,55,1.f);	
	InitArrays();
}

SphereC::SphereC(int stacks, int slices, GLfloat r)
{
	this->stacks=stacks;
	this->slices=slices;
	this->r=r;
	Generate(stacks,slices,r);	
	InitArrays();
}

void CubeC::InitArrays()
{
	glGenVertexArrays(1,&vaID);
	glBindVertexArray(vaID);
	glGenBuffers(1, &buffer);
	glBindBuffer(GL_ARRAY_BUFFER, buffer);
	points=vertex.size();
	glBufferData(GL_ARRAY_BUFFER, points*sizeof(GLfloat), &vertex[0], GL_STATIC_DRAW);
	glVertexAttribPointer((GLuint)0, 3, GL_FLOAT, GL_FALSE, 0, 0); 
	glEnableVertexAttribArray(0);
	vertex.clear(); //no need for the vertex data, it is on the GPU now
}


CubeC::CubeC()
{
	Generate();	
	InitArrays();
}

void CubeC::Generate()
{
	const glm::vec3 A=glm::vec3(-0.5f,-0.5f,-0.5f);
	const glm::vec3 B=glm::vec3(+0.5f,-0.5f,-0.5f);
	const glm::vec3 C=glm::vec3(+0.5f,+0.5f,-0.5f);
	const glm::vec3 D=glm::vec3(-0.5f,+0.5f,-0.5f);
	const glm::vec3 E=glm::vec3(-0.5f,-0.5f,+0.5f);
	const glm::vec3 F=glm::vec3(+0.5f,-0.5f,+0.5f);
	const glm::vec3 G=glm::vec3(+0.5f,+0.5f,+0.5f);
	const glm::vec3 H=glm::vec3(-0.5f,+0.5f,+0.5f);
	AddVertex(&vertex,&A);AddVertex(&vertex,&B);AddVertex(&vertex,&C);
	AddVertex(&vertex,&A);AddVertex(&vertex,&C);AddVertex(&vertex,&D);
	AddVertex(&vertex,&A);AddVertex(&vertex,&E);AddVertex(&vertex,&F);
	AddVertex(&vertex,&A);AddVertex(&vertex,&F);AddVertex(&vertex,&B);
	AddVertex(&vertex,&B);AddVertex(&vertex,&F);AddVertex(&vertex,&G);
	AddVertex(&vertex,&B);AddVertex(&vertex,&G);AddVertex(&vertex,&C);
	AddVertex(&vertex,&C);AddVertex(&vertex,&G);AddVertex(&vertex,&H);
	AddVertex(&vertex,&C);AddVertex(&vertex,&H);AddVertex(&vertex,&D);
	AddVertex(&vertex,&D);AddVertex(&vertex,&H);AddVertex(&vertex,&E);
	AddVertex(&vertex,&D);AddVertex(&vertex,&E);AddVertex(&vertex,&A);
	AddVertex(&vertex,&E);AddVertex(&vertex,&F);AddVertex(&vertex,&G);
	AddVertex(&vertex,&E);AddVertex(&vertex,&G);AddVertex(&vertex,&H);
}

void CubeC::Render()
{
	glBindVertexArray(vaID);
	glBindBuffer(GL_ARRAY_BUFFER, buffer);
	glEnableVertexAttribArray(0);
    glUniformMatrix4fv(modelParameter,1,GL_FALSE,glm::value_ptr(model));
	glDrawArrays(GL_TRIANGLES, 0, 3*points);
}

