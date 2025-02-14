/** \file dab_geom_mesh_manager.h
*/

#include "dab_geom_mesh_manager.h"
#include "dab_tokenizer.h"
#include "ofxAssimpModelLoader.h"

using namespace dab;
using namespace dab::geom;

MeshManager::MeshManager()
{}

MeshManager::~MeshManager()
{}

bool 
MeshManager::meshExists(const std::string& pMeshName, unsigned int pMeshIndex) const
{
	if (mMeshes.contains(pMeshName) == true) return true;
	if (mMeshes.contains(pMeshName + "_" + std::to_string(pMeshIndex)) == true) return true;
	return false;
}

//std::pair<std::shared_ptr<ofMesh>, std::string> 
//MeshManager::loadMesh(const std::string& pMeshFile, unsigned int pMeshIndex) throw (dab::Exception)
//{
//	// load mesh from file and automatically generate mesh name from mesh file
//
//	// load mesh
//	ofxAssimpModelLoader _modelLoader;
//	bool success = _modelLoader.loadModel(pMeshFile, true);
//	if (success == false) throw dab::Exception("GEOM ERROR: failed to load mesh file " + pMeshFile, __FILE__, __FUNCTION__, __LINE__);
//
//	// check number of meshes stored in mesh file
//	unsigned int meshCount = _modelLoader.getMeshCount();
//	if (meshCount < pMeshIndex) throw dab::Exception("VIS ERROR: mesh index " + std::to_string(pMeshIndex) + " exceeds number of meshes available " + std::to_string(meshCount), __FILE__, __FUNCTION__, __LINE__);
//
//	// construct mesh name 
//
//	// split mesh file path if necessary
//	Tokenizer& tokenizer = Tokenizer::get();
//	std::string _meshFile = pMeshFile;
//	std::vector<std::string> splitMeshFileName;
//
//	// make sure all the directory delimiters are the same
//	#ifdef _WIN32
//	std::replace(_meshFile.begin(), _meshFile.end(), '/', '\\');
//	#else
//	std::replace(_meshFile.begin(), _meshFile.end(), '\\', '/');
//	#endif
//
//	// split file name at directory delimiters
//	#ifdef _WIN32
//	if (_meshFile.find('\\') != std::string::npos) tokenizer.split(_meshFile, splitMeshFileName, '\\');
//	#else
//	if (_meshFile.find('/') != std::string::npos) tokenizer.split(_meshFile, splitMeshFileName, '/');
//	#endif
//	else  splitMeshFileName.push_back(_meshFile);
//
//	if (splitMeshFileName.size() == 0) throw dab::Exception("Geom Error: failed to load mesh file " + pMeshFile, __FILE__, __FUNCTION__, __LINE__);
//
//	std::string fileName = splitMeshFileName[splitMeshFileName.size() - 1];
//
//	// get rid of file ending
//	if (fileName.find('.') == std::string::npos) throw dab::Exception("Geom Error: no suffix found in mesh file " + pMeshFile, __FILE__, __FUNCTION__, __LINE__);
//	
//	std::string meshName = fileName.substr(0, fileName.find_last_of('.'));
//
//	// add mesh index to mesh name in case there are several meshes stored in mesh file
//	if (meshCount > 1) meshName += "_" + std::to_string(pMeshIndex);
//
//	//std::cout << "pMeshFile " << pMeshFile << " fileName " << fileName << " meshName " << meshName << "\n";
//
//	try
//	{
//		std::shared_ptr<ofMesh> _mesh = loadMesh(meshName, _meshFile, pMeshIndex);
//		return std::pair<std::shared_ptr<ofMesh>, std::string>(_mesh, meshName);
//	}
//	catch (dab::Exception& e)
//	{
//		throw e;
//	}
//}

//std::shared_ptr<ofMesh>
//MeshManager::loadMesh(const std::string& pMeshName, std::string& pMeshFile, unsigned int pMeshIndex) throw (dab::Exception)
//{
//	std::cout << "load Mesh " << pMeshFile << "\n";
//
//	if (mMeshes.contains(pMeshName)) throw dab::Exception("Geom Error: mesh " + pMeshName + " already exists", __FILE__, __FUNCTION__, __LINE__);
//
//	ofxAssimpModelLoader _modelLoader;
//	bool success = _modelLoader.loadModel(pMeshFile, true);
//
//	if (success == false) throw dab::Exception("GEOM ERROR: failed to load mesh file " + pMeshFile, __FILE__, __FUNCTION__, __LINE__);
//
//	unsigned int meshCount = _modelLoader.getMeshCount();
//
//	if (meshCount < pMeshIndex) throw dab::Exception("VIS ERROR: mesh index " + std::to_string(pMeshIndex) + " exceeds number of meshes available " + std::to_string(meshCount), __FILE__, __FUNCTION__, __LINE__);
//
//	std::shared_ptr<ofMesh> _mesh = std::shared_ptr<ofMesh>(new ofMesh());
//
//	*_mesh = _modelLoader.getMesh(pMeshIndex);
//
//	mMeshes.add(pMeshName, _mesh);
//
//	return _mesh;
//}

std::shared_ptr<ofMesh> 
MeshManager::loadMesh(std::string& pMeshName, std::string& pMeshFile, unsigned int pMeshIndex) throw (dab::Exception)
{
	std::string _meshFile = pMeshFile;

	// load mesh
	ofxAssimpModelLoader _modelLoader;
	bool success = _modelLoader.loadModel(pMeshFile, true);
	if (success == false) throw dab::Exception("GEOM ERROR: failed to load mesh file " + pMeshFile, __FILE__, __FUNCTION__, __LINE__);

	// check number of meshes stored in mesh file
	unsigned int meshCount = _modelLoader.getMeshCount();
	if (meshCount < pMeshIndex) throw dab::Exception("VIS ERROR: mesh index " + std::to_string(pMeshIndex) + " exceeds number of meshes available " + std::to_string(meshCount), __FILE__, __FUNCTION__, __LINE__);

	if (pMeshName == "")
	{
		pMeshName = meshFile2MeshName(pMeshFile, meshCount, pMeshIndex);
	}

	if (mMeshes.contains(pMeshName)) throw dab::Exception("Geom Error: mesh " + pMeshName + " already exists", __FILE__, __FUNCTION__, __LINE__);

	std::shared_ptr<ofMesh> _mesh = std::shared_ptr<ofMesh>(new ofMesh());

	*_mesh = _modelLoader.getMesh(pMeshIndex);

	mMeshes.add(pMeshName, _mesh);

	return _mesh;
}

std::string 
MeshManager::meshFile2MeshName(const std::string& pMeshFile, unsigned int pMeshCount, unsigned int pMeshIndex)
{
	// construct mesh name from mesh file
	std::string _meshFile = pMeshFile;

	// split mesh file path if necessary
	Tokenizer& tokenizer = Tokenizer::get();
	std::vector<std::string> splitMeshFileName;

	// make sure all the directory delimiters are the same
	#ifdef _WIN32
	std::replace(_meshFile.begin(), _meshFile.end(), '/', '\\');
	#else
	std::replace(_meshFile.begin(), _meshFile.end(), '\\', '/');
	#endif

	// split file name at directory delimiters
	#ifdef _WIN32
	if (_meshFile.find('\\') != std::string::npos) tokenizer.split(_meshFile, splitMeshFileName, '\\');
	#else
	if (_meshFile.find('/') != std::string::npos) tokenizer.split(_meshFile, splitMeshFileName, '/');
	#endif
	else  splitMeshFileName.push_back(_meshFile);

	if (splitMeshFileName.size() == 0) throw dab::Exception("Geom Error: failed to load mesh file " + pMeshFile, __FILE__, __FUNCTION__, __LINE__);

	std::string fileName = splitMeshFileName[splitMeshFileName.size() - 1];

	// get rid of file ending
	if (fileName.find('.') == std::string::npos) throw dab::Exception("Geom Error: no suffix found in mesh file " + pMeshFile, __FILE__, __FUNCTION__, __LINE__);

	std::string meshName = fileName.substr(0, fileName.find_last_of('.'));

	// add mesh index to mesh name in case there are several meshes stored in mesh file
	if (pMeshCount > 1) meshName += "_" + std::to_string(pMeshIndex);

	//std::cout << "pMeshFile " << pMeshFile << " fileName " << fileName << " meshName " << meshName << "\n";

	return meshName;
}

//std::shared_ptr<ofMesh> 
//MeshManager::loadMesh(const std::string& pMeshName, std::string& pMeshFile, unsigned int pMeshIndex) throw (dab::Exception)
//{
//	if (mMeshes.contains(pMeshName)) throw dab::Exception("GEOM ERROR: mesh " + pMeshName + " already exists", __FILE__, __FUNCTION__, __LINE__);
//
//	std::shared_ptr<ofxAssimpModelLoader> _modelLoader = std::shared_ptr<ofxAssimpModelLoader>(new ofxAssimpModelLoader());
//	bool success = _modelLoader->loadModel(pMeshFile, true);
//
//	if (success == false) throw dab::Exception("GEOM ERROR: failed to load mesh file " + pMeshFile, __FILE__, __FUNCTION__, __LINE__);
//
//	unsigned int meshCount = _modelLoader->getMeshCount();
//
//	if (meshCount < pMeshIndex) throw dab::Exception("VIS ERROR: mesh index " + std::to_string(pMeshIndex) + " exceeds number of meshes available " + std::to_string(meshCount), __FILE__, __FUNCTION__, __LINE__);
//
//	mModelLoaders.push_back(_modelLoader);
//
//	//std::shared_ptr<ofMesh> _mesh = std::shared_ptr<ofMesh>( &(_modelLoader->getMesh(pMeshIndex)) );
//
//	std::shared_ptr<ofMesh> _mesh = std::shared_ptr<ofMesh>( new ofMesh() );
//
//	*_mesh = _modelLoader->getMesh(pMeshIndex);
//
//	mMeshes.add(pMeshName, _mesh);
//
//	return _mesh;
//}

std::shared_ptr<ofMesh> 
MeshManager::mesh(const std::string& pMeshName) throw (dab::Exception)
{
	if (mMeshes.contains(pMeshName) == false) throw dab::Exception("GEOM ERROR: mesh " + pMeshName + " not found", __FILE__, __FUNCTION__, __LINE__);

	return mMeshes[pMeshName];
}

const std::vector<glm::vec3>& 
MeshManager::vertices(const std::string& pMeshName) throw (dab::Exception)
{
	if (mMeshes.contains(pMeshName) == false) throw dab::Exception("GEOM ERROR: mesh " + pMeshName + " not found", __FILE__, __FUNCTION__, __LINE__);

	return mMeshes[pMeshName]->getVertices();
}
