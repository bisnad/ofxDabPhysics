/** \file dab_vis_material.cpp
*/

#include "dab_vis_material.h"

using namespace dab;
using namespace dab::vis;

float Material::sTransparency = 0.0;
float Material::sAmbientScale = 0.5;
float Material::sDiffuseScale = 0.5;
float Material::sSpecularScale = 0.5;
float Material::sSpecularPow = 10.0;
glm::vec3 Material::sAmbientColor = glm::vec3(1.0, 1.0, 1.0);
glm::vec3 Material::sDiffuseColor = glm::vec3(1.0, 1.0, 1.0);

Material::Material()
	: mTransparency(sTransparency)
	, mAmbientScale(sAmbientScale)
	, mDiffuseScale(sDiffuseScale)
	, mSpecularScale(sSpecularScale)
	, mSpecularPow(sSpecularPow)
	, mAmbientColor(sAmbientColor)
	, mDiffuseColor(sDiffuseColor)
	, mTextureName("")
{}

Material::Material(const Material& pMaterial)
	: mTransparency(pMaterial.mTransparency)
	, mAmbientScale(pMaterial.mAmbientScale)
	, mDiffuseScale(pMaterial.mDiffuseScale)
	, mSpecularScale(pMaterial.mSpecularScale)
	, mSpecularPow(pMaterial.mSpecularPow)
	, mAmbientColor(pMaterial.mAmbientColor)
	, mDiffuseColor(pMaterial.mDiffuseColor)
	, mTextureName(pMaterial.mTextureName)
{}

Material::~Material()
{}


Material& 
Material::operator=(const Material& pMaterial)
{
	mTransparency = pMaterial.mTransparency;
	mAmbientScale = pMaterial.mAmbientScale;
	mDiffuseScale = pMaterial.mDiffuseScale;
	mSpecularScale = pMaterial.mSpecularScale;
	mSpecularPow = pMaterial.mSpecularPow;
	mAmbientColor = pMaterial.mAmbientColor;
	mDiffuseColor = pMaterial.mDiffuseColor;
	mTextureName = pMaterial.mTextureName;

	return *this;
}

float 
Material::transparency() const
{
	return mTransparency;
}

float 
Material::ambientScale() const
{
	return mAmbientScale;
}

float 
Material::diffuseScale() const
{
	return mDiffuseScale;
}

float 
Material::specularScale() const
{
	return mSpecularScale;
}

float 
Material::specularPow() const
{
	return mSpecularPow;
}

const glm::vec3& 
Material::ambientColor() const
{
	return mAmbientColor;
}

const glm::vec3& 
Material::diffuseColor() const
{
	return mDiffuseColor;
}

const std::string&
Material::textureName() const
{
	return mTextureName;
}

void 
Material::setTransparency(float pTransparency)
{
	mTransparency = pTransparency;
}

void 
Material::setAmbientScale(float pAmbientScale)
{
	mAmbientScale = pAmbientScale;
}

void 
Material::setDiffuseScale(float pDiffuseScale)
{
	mDiffuseScale = pDiffuseScale;
}

void 
Material::setSpecularScale(float pSpecularScale)
{
	mSpecularScale = pSpecularScale;
}

void 
Material::setSpecularPow(float pSpecularPow)
{
	mSpecularPow = pSpecularPow;
}

void 
Material::setAmbientColor(const glm::vec3& pAmbientColor)
{
	mAmbientColor = pAmbientColor;
}

void 
Material::setDiffuseColor(const glm::vec3& pDiffuseColor)
{
	mDiffuseColor = pDiffuseColor;
}

void
Material::setTextureName(const std::string& pTextureName)
{
	mTextureName = pTextureName;
}