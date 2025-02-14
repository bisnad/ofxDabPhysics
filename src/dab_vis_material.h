/** \file dab_vis_material.h

// TODO: add texture
*/

#pragma once

#include "ofVectorMath.h"

namespace dab
{

namespace vis
{

class Material
{
public:
	Material();
	Material(const Material& pMaterial);
	~Material();

	Material& operator=(const Material& pMaterial);

	float transparency() const;
	float ambientScale() const;
	float diffuseScale() const;
	float specularScale() const;
	float specularPow() const;
	const glm::vec3& ambientColor() const;
	const glm::vec3& diffuseColor() const;
	const std::string& textureName() const;

	void setTransparency(float pTransparency);
	void setAmbientScale(float pAmbientScale);
	void setDiffuseScale(float pDiffuseScale);
	void setSpecularScale(float pSpecularScale);
	void setSpecularPow(float pSpecularPow);
	void setAmbientColor(const glm::vec3& pAmbientColor);
	void setDiffuseColor(const glm::vec3& pDiffuseColor);
	void setTextureName(const std::string& pTextureName);

protected:
	static float sTransparency;
	static float sAmbientScale;
	static float sDiffuseScale;
	static float sSpecularScale;
	static float sSpecularPow;
	static glm::vec3 sAmbientColor;
	static glm::vec3 sDiffuseColor;

	float mTransparency;
	float mAmbientScale;
	float mDiffuseScale;
	float mSpecularScale;
	float mSpecularPow;
	glm::vec3 mAmbientColor;
	glm::vec3 mDiffuseColor;
	std::string mTextureName;
};

};

};