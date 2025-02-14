/** \file dab_geom_mesh_manager.h
*/

#pragma once

#include "ofVectorMath.h"
#include "ofMesh.h"
#include "dab_singleton.h"
#include "dab_index_map.h"
#include "dab_exception.h"
#include "ofxAssimpModelLoader.h"

namespace dab
{

namespace geom
{

class MeshManager : public Singleton<MeshManager>
{
public:
	MeshManager();
	~MeshManager();

	//std::pair<std::shared_ptr<ofMesh>, std::string> loadMesh(const std::string& pMeshFile, unsigned int pMeshIndex = 0) throw (dab::Exception);
	//std::shared_ptr<ofMesh> loadMesh(const std::string& pMeshName, std::string& pMeshFile, unsigned int pMeshIndex = 0) throw (dab::Exception);
	std::shared_ptr<ofMesh> loadMesh(std::string& pMeshName, std::string& pMeshFile, unsigned int pMeshIndex = 0) throw (dab::Exception);

	bool meshExists(const std::string& pMeshName, unsigned int pMeshIndex = 0) const;
	std::shared_ptr<ofMesh> mesh(const std::string& pMeshName) throw (dab::Exception);
	const std::vector<glm::vec3>& vertices(const std::string& pMeshName) throw (dab::Exception);

protected:
	IndexMap<std::string, std::shared_ptr<ofMesh>> mMeshes;

	std::string meshFile2MeshName(const std::string& pMeshFile, unsigned int pMeshCount, unsigned int pMeshIndex);
};

};

};