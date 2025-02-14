/** \file dab_vis_body_shape.h

Compound Shape functionality not yet implemented
*/

#pragma once

#include <iostream>
#include <vector>
#include "of3dPrimitives.h"
#include "ofMesh.h"
#include "ofMaterial.h"
#include "ofShader.h"
#include "dab_exception.h"
#include "dab_vis_body_transform.h"
#include "dab_vis_material.h"

namespace dab
{

namespace vis
{

#pragma mark BodyShape definition

class BodyShape
{
public:
	BodyShape(const std::string& pName);
	~BodyShape();

	int id() const;
	const std::string& name() const;
	BodyTransform& transform();
	const BodyTransform& transform() const;

	Material& material();

	virtual void drawFaces(const glm::mat4& pPreTransform, const ofShader& pShader);
	virtual void drawWireframe(const glm::mat4& pPreTransform, const ofShader& pShader);

protected:
	static int sId;

	int mId;
	std::string mName;

	BodyTransform mTransform;
	Material mMaterial;
	std::shared_ptr<of3dPrimitive> mVisPrimitive;

	virtual void initVisuals() = 0;
};

#pragma mark BodyPlaneShape definition

class BodyPlaneShape : public BodyShape
{
public:
	BodyPlaneShape(const std::string& pName);
	BodyPlaneShape(const std::string& pName, const glm::vec3& pNormal);
	~BodyPlaneShape();

	static void setSize(float pSize);
	static void setResolution(int pResolution);
	static void setNormal(const glm::vec3& pNormal);

protected:
	static float sSize;
	static int sResolution;
	static glm::vec3 sNormal;

	float mSize;
	int mResolution;
	glm::vec3 mNormal;

	void initVisuals();
};

#pragma mark BodySphereShape definition

class BodySphereShape : public BodyShape
{
public:
	BodySphereShape(const std::string& pName);
	BodySphereShape(const std::string& pName, float pRadius);
	~BodySphereShape();

	static void setRadius(float pRadius);
	static void setResolution(int pResolution);

protected:
	static float sRadius;
	static int sResolution;

	float mRadius;
	int mResolution;

	void initVisuals();
};

#pragma mark BodyBoxShape definition

class BodyBoxShape : public BodyShape
{
public:
	BodyBoxShape(const std::string& pName);
	BodyBoxShape(const std::string& pName, const std::array<float, 3>& pSize);
	~BodyBoxShape();

	static void setSize(const std::array<float, 3>& pSize);
	static void setResolution(const std::array<int, 3>& pResolution);

protected:
	static std::array<float, 3> sSize;
	static std::array<int, 3> sResolution;

	std::array<float, 3> mSize;
	std::array<int, 3> mResolution;

	void initVisuals();
};

#pragma mark BodyCylinderShape definition

class BodyCylinderShape : public BodyShape
{
public:
	BodyCylinderShape(const std::string& pName);
	BodyCylinderShape(const std::string& pName, float pRadius, float pHeight);
	~BodyCylinderShape();

	static void setRadius(float pRadius);
	static void setHeight(float pHeight);
	static void setResolution(const std::array<int, 2>& pResolution);

protected:
	static float sRadius;
	static float sHeight;
	static std::array<int, 2> sResolution;

	float mRadius;
	float mHeight;
	std::array<int, 2> mResolution;

	void initVisuals();
};

#pragma mark BodyCapsuleShape definition

//Warning: at the moment, this class is a copy of BodyCylinderShape
//TODO: create a custom capsule mesh instead of a cylinder primite

class BodyCapsuleShape : public BodyShape
{
public:
	BodyCapsuleShape(const std::string& pName);
	BodyCapsuleShape(const std::string& pName, float pRadius, float pHeight);
	~BodyCapsuleShape();

	static void setRadius(float pRadius);
	static void setHeight(float pHeight);
	static void setResolution(const std::array<int, 2>& pResolution);

protected:
	static float sRadius;
	static float sHeight;
	static std::array<int, 2> sResolution;

	float mRadius;
	float mHeight;
	std::array<int, 2> mResolution;

	void initVisuals();
};

#pragma mark BodyConeShape definition

class BodyConeShape : public BodyShape
{
public:
	BodyConeShape(const std::string& pName);
	BodyConeShape(const std::string& pName, float pRadius, float pHeight);
	~BodyConeShape();

	static void setRadius(float pRadius);
	static void setHeight(float pHeight);
	static void setResolution(const std::array<int, 3>& pResolution);

protected:
	static float sRadius;
	static float sHeight;
	static std::array<int, 3> sResolution;

	float mRadius;
	float mHeight;
	std::array<int, 3> mResolution;

	void initVisuals();
};

#pragma mark BodyMeshShape definition

class BodyMeshShape : public BodyShape
{
public:
	BodyMeshShape(const std::string& pName, const std::string& pMeshName, std::shared_ptr<ofMesh> pMesh);
	~BodyMeshShape();

	const std::string& meshName() const;

protected:

	std::string mMeshName;
	std::shared_ptr<ofMesh> mMesh;

	void initVisuals();
};

#pragma mark BodyCompoundShape definition

class BodyCompoundShape : public BodyShape
{
public:
	BodyCompoundShape(const std::string& pName);
	~BodyCompoundShape();

	int childCount() const;
	const std::vector< BodyTransform >& childTransforms() const;
	const std::vector< std::shared_ptr<BodyShape> >& childShapes() const;

	void addChildShape(std::shared_ptr<BodyShape> pShape, const glm::vec3& pPosition, const glm::quat& pOrientation);
	void addChildShape(std::shared_ptr<BodyShape> pShape, const BodyTransform& pTransform);

	void drawFaces(const glm::mat4& pPreTransform, const ofShader& pShader);
	void drawWireframe(const glm::mat4& pPreTransform, const ofShader& pShader);

protected:
	std::vector< BodyTransform > mChildTransforms;
	std::vector< std::shared_ptr<BodyShape> > mChildShapes;

	void initVisuals();
};


};

};