/** \file dab_urdf_parser.h

// TODO: this shouldn't be a singleton class since it stores internal information
*/

#pragma once

#include "dab_singleton.h"
#include "dab_tokenizer.h"
#include "dab_exception.h"
#include "tinyxml2.h"
#include <btBulletDynamicsCommon.h>

namespace dab
{
	enum UrdfJointTypes
	{
		URDFRevoluteJoint = 1,
		URDFPrismaticJoint,
		URDFContinuousJoint,
		URDFFloatingJoint,
		URDFPlanarJoint,
		URDFFixedJoint,
		URDFSphericalJoint,
	};

	enum URDF_LinkContactFlags
	{
		URDF_CONTACT_HAS_LATERAL_FRICTION = 1,
		URDF_CONTACT_HAS_INERTIA_SCALING = 2,
		URDF_CONTACT_HAS_CONTACT_CFM = 4,
		URDF_CONTACT_HAS_CONTACT_ERP = 8,
		URDF_CONTACT_HAS_STIFFNESS_DAMPING = 16,
		URDF_CONTACT_HAS_ROLLING_FRICTION = 32,
		URDF_CONTACT_HAS_SPINNING_FRICTION = 64,
		URDF_CONTACT_HAS_RESTITUTION = 128,
		URDF_CONTACT_HAS_FRICTION_ANCHOR = 256,

	};

	struct URDFLinkContactInfo
	{
		btScalar m_lateralFriction;
		btScalar m_rollingFriction;
		btScalar m_spinningFriction;
		btScalar m_restitution;
		btScalar m_inertiaScaling;
		btScalar m_contactCfm;
		btScalar m_contactErp;
		btScalar m_contactStiffness;
		btScalar m_contactDamping;

		int m_flags;

		URDFLinkContactInfo()
			: m_lateralFriction(0.5),
			m_rollingFriction(0),
			m_spinningFriction(0),
			m_restitution(0),
			m_inertiaScaling(1),
			m_contactCfm(0),
			m_contactErp(0),
			m_contactStiffness(1e4),
			m_contactDamping(1)
		{
			m_flags = URDF_CONTACT_HAS_LATERAL_FRICTION;
		}
	};

	enum UrdfCollisionFlags
	{
		URDF_FORCE_CONCAVE_TRIMESH = 1,
		URDF_HAS_COLLISION_GROUP = 2,
		URDF_HAS_COLLISION_MASK = 4,
	};

	struct UrdfMaterialColor
	{
		btVector4 m_rgbaColor;
		btVector3 m_specularColor;
		UrdfMaterialColor()
			: m_rgbaColor(0.8, 0.8, 0.8, 1),
			m_specularColor(0.4, 0.4, 0.4)
		{
		}
	};

	//manually sync with eURDF_Flags in SharedMemoryPublic.h!
	enum ConvertURDFFlags
	{
		CUF_USE_SDF = 1,
		// Use inertia values in URDF instead of recomputing them from collision shape.
		CUF_USE_URDF_INERTIA = 2,
		CUF_USE_MJCF = 4,
		CUF_USE_SELF_COLLISION = 8,
		CUF_USE_SELF_COLLISION_EXCLUDE_PARENT = 16,
		CUF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS = 32,
		CUF_RESERVED = 64,
		CUF_USE_IMPLICIT_CYLINDER = 128,
		CUF_GLOBAL_VELOCITIES_MB = 256,
		CUF_MJCF_COLORS_FROM_FILE = 512,
		CUF_ENABLE_CACHED_GRAPHICS_SHAPES = 1024,
		CUF_ENABLE_SLEEPING = 2048,
		CUF_INITIALIZE_SAT_FEATURES = 4096,
		CUF_USE_SELF_COLLISION_INCLUDE_PARENT = 8192,
		CUF_PARSE_SENSORS = 16384,
		CUF_USE_MATERIAL_COLORS_FROM_MTL = 32768,
		CUF_USE_MATERIAL_TRANSPARANCY_FROM_MTL = 65536,
		CUF_MAINTAIN_LINK_ORDER = 131072,
		CUF_ENABLE_WAKEUP = 1 << 18,
		CUF_MERGE_FIXED_LINKS = 1 << 19,
		CUF_IGNORE_VISUAL_SHAPES = 1 << 20,
		CUF_IGNORE_COLLISION_SHAPES = 1 << 21,
		CUF_PRINT_URDF_INFO = 1 << 22,
		CUF_GOOGLEY_UNDEFINED_COLORS = 1 << 23,

	};

	#define btArray btAlignedObjectArray

	struct UrdfErrorLogger
	{
		virtual ~UrdfErrorLogger() 
		{}

		virtual void reportError(const char* error)
		{ 
			std::cout << "URDF Error: " << error << "\n"; 
		};

		virtual void reportWarning(const char* warning)
		{ 
			std::cout << "URDF Warning: " << warning << "\n"; 
		};

		virtual void printMessage(const char* msg)
		{ 
			std::cout << "URDF Message: " << msg << "\n"; 
		};
	};

	struct UrdfMaterial
	{
		std::string m_name;
		std::string m_textureFilename;
		UrdfMaterialColor m_matColor;

		UrdfMaterial()
		{
		}
	};

	struct UrdfInertia
	{
		btTransform m_linkLocalFrame;
		bool m_hasLinkLocalFrame;

		double m_mass;
		double m_ixx, m_ixy, m_ixz, m_iyy, m_iyz, m_izz;

		UrdfInertia()
		{
			m_hasLinkLocalFrame = false;
			m_linkLocalFrame.setIdentity();
			m_mass = 0.f;
			m_ixx = m_ixy = m_ixz = m_iyy = m_iyz = m_izz = 0.f;
		}
	};

	enum UrdfGeomTypes
	{
		URDF_GEOM_SPHERE = 2,
		URDF_GEOM_BOX,
		URDF_GEOM_CYLINDER,
		URDF_GEOM_MESH,
		URDF_GEOM_PLANE,
		URDF_GEOM_CAPSULE,  //non-standard URDF
		URDF_GEOM_CDF,      //signed-distance-field, non-standard URDF
		URDF_GEOM_HEIGHTFIELD,   //heightfield, non-standard URDF
		URDF_GEOM_UNKNOWN,
	};

	struct UrdfGeometry
	{
		UrdfGeomTypes m_type;

		double m_sphereRadius;

		btVector3 m_boxSize;

		double m_capsuleRadius;
		double m_capsuleHeight;
		int m_hasFromTo;
		btVector3 m_capsuleFrom;
		btVector3 m_capsuleTo;

		btVector3 m_planeNormal;

		enum
		{
			FILE_STL = 1,
			FILE_COLLADA = 2,
			FILE_OBJ = 3,
			FILE_CDF = 4,
			MEMORY_VERTICES = 5,
			FILE_VTK = 6,
		};
		int m_meshFileType;
		std::string m_meshFileName;
		btVector3 m_meshScale;

		btArray<btVector3> m_vertices;
		btArray<btVector3> m_uvs;
		btArray<btVector3> m_normals;
		btArray<int> m_indices;


		UrdfMaterial m_localMaterial;
		bool m_hasLocalMaterial;

		UrdfGeometry()
			: m_type(URDF_GEOM_UNKNOWN),
			m_sphereRadius(1),
			m_boxSize(1, 1, 1),
			m_capsuleRadius(1),
			m_capsuleHeight(1),
			m_hasFromTo(0),
			m_capsuleFrom(0, 1, 0),
			m_capsuleTo(1, 0, 0),
			m_planeNormal(0, 0, 1),
			m_meshFileType(0),
			m_meshScale(1, 1, 1),
			m_hasLocalMaterial(false)
		{
		}
	};


	struct UrdfShape
	{
		std::string m_sourceFileLocation;
		btTransform m_linkLocalFrame;
		UrdfGeometry m_geometry;
		std::string m_name;
	};

	struct UrdfVisual : UrdfShape
	{
		std::string m_materialName;
		// Maps user data keys to user data values.
		btHashMap<btHashString, std::string> m_userData;
	};

	struct UrdfCollision : UrdfShape
	{
		int m_flags;
		int m_collisionGroup;
		int m_collisionMask;
		UrdfCollision()
			: m_flags(0)
		{
		}
	};

	struct UrdfJoint;

	struct UrdfLink
	{
		std::string m_name;
		UrdfInertia m_inertia;
		btTransform m_linkTransformInWorld;
		btArray<UrdfVisual> m_visualArray;
		btArray<UrdfCollision> m_collisionArray;
		UrdfLink* m_parentLink;
		UrdfJoint* m_parentJoint;

		btArray<UrdfJoint*> m_childJoints;
		btArray<UrdfLink*> m_childLinks;

		int m_linkIndex;

		URDFLinkContactInfo m_contactInfo;

		// Maps user data keys to user data values.
		btHashMap<btHashString, std::string> m_userData;

		UrdfLink()
			: m_parentLink(0),
			m_parentJoint(0),
			m_linkIndex(-2)
		{
		}
	};

	struct UrdfJoint
	{
		std::string m_name;
		UrdfJointTypes m_type;
		btTransform m_parentLinkToJointTransform;
		std::string m_parentLinkName;
		std::string m_childLinkName;
		btVector3 m_localJointAxis;

		double m_lowerLimit;
		double m_upperLimit;

		double m_effortLimit;
		double m_velocityLimit;

		double m_jointDamping;
		double m_jointFriction;
		UrdfJoint()
			: m_lowerLimit(0),
			m_upperLimit(-1),
			m_effortLimit(0),
			m_velocityLimit(0),
			m_jointDamping(0),
			m_jointFriction(0)
		{
		}
	};

	struct SpringCoeffcients
	{
		double elastic_stiffness;
		double damping_stiffness;
		double bending_stiffness;
		int damp_all_directions;
		int bending_stride;
		SpringCoeffcients() : elastic_stiffness(0.),
			damping_stiffness(0.),
			bending_stiffness(0.),
			damp_all_directions(0),
			bending_stride(2) {}
	};

	struct LameCoefficients
	{
		double mu;
		double lambda;
		double damping;
		LameCoefficients() : mu(0.), lambda(0.), damping(0.) {}
	};

	struct UrdfDeformable
	{
		std::string m_name;
		double m_mass;
		double m_collisionMargin;
		double m_friction;
		double m_repulsionStiffness;
		double m_gravFactor;
		bool m_cache_barycenter;

		SpringCoeffcients m_springCoefficients;
		LameCoefficients m_corotatedCoefficients;
		LameCoefficients m_neohookeanCoefficients;

		std::string m_visualFileName;
		std::string m_simFileName;
		btHashMap<btHashString, std::string> m_userData;

		UrdfDeformable() : m_mass(1.), m_collisionMargin(0.02), m_friction(1.), m_repulsionStiffness(0.5), m_gravFactor(1.), m_cache_barycenter(false), m_visualFileName(""), m_simFileName("")
		{
		}
	};

	struct UrdfModel
	{
		std::string m_name;
		std::string m_sourceFile;
		btTransform m_rootTransformInWorld;
		btHashMap<btHashString, UrdfMaterial*> m_materials;
		btHashMap<btHashString, UrdfLink*> m_links;
		btHashMap<btHashString, UrdfJoint*> m_joints;
		UrdfDeformable m_deformable;
		// Maps user data keys to user data values.
		btHashMap<btHashString, std::string> m_userData;

		btArray<UrdfLink*> m_rootLinks;
		bool m_overrideFixedBase;

		UrdfModel()
			: m_overrideFixedBase(false)
		{
			m_rootTransformInWorld.setIdentity();
		}

		~UrdfModel()
		{
			for (int i = 0; i < m_materials.size(); i++)
			{
				UrdfMaterial** ptr = m_materials.getAtIndex(i);
				if (ptr)
				{
					UrdfMaterial* t = *ptr;
					delete t;
				}
			}
			for (int i = 0; i < m_links.size(); i++)
			{
				UrdfLink** ptr = m_links.getAtIndex(i);
				if (ptr)
				{
					UrdfLink* t = *ptr;
					delete t;
				}
			}
			for (int i = 0; i < m_joints.size(); i++)
			{
				UrdfJoint** ptr = m_joints.getAtIndex(i);
				if (ptr)
				{
					UrdfJoint* t = *ptr;
					delete t;
				}
			}
		}
	};


class UrdfParser : public Singleton<UrdfParser>
{
public:
	UrdfParser();
	~UrdfParser();

	void setParseSDF(bool useSDF)
	{
		m_parseSDF = useSDF;
	}

	bool getParseSDF() const
	{
		return m_parseSDF;
	}
	void setGlobalScaling(btScalar scaling)
	{
		m_urdfScaling = scaling;
	}

	bool loadUrdf(const char* urdfText, bool forceFixedBase, bool parseSensors);

	bool loadUrdf(const char* urdfText, bool forceFixedBase)
	{
		return loadUrdf(urdfText, forceFixedBase, false);
	}

	bool loadSDF(const char* sdfText);

	int getNumModels() const
	{
		//user should have loaded an SDF when calling this method
		if (m_parseSDF)
		{
			return m_sdfModels.size();
		}
		return 1;
	}

	void activateModel(int modelIndex);

	UrdfModel& getModelByIndex(int index)
	{
		//user should have loaded an SDF when calling this method
		btAssert(m_parseSDF);

		return *m_sdfModels[index];
	}

	const UrdfModel& getModelByIndex(int index) const
	{
		//user should have loaded an SDF when calling this method
		btAssert(m_parseSDF);

		return *m_sdfModels[index];
	}

	const UrdfModel& getModel() const
	{
		if (m_parseSDF)
		{
			return *m_sdfModels[m_activeSdfModel];
		}

		return m_urdf2Model;
	}

	UrdfModel& getModel()
	{
		if (m_parseSDF)
		{
			return *m_sdfModels[m_activeSdfModel];
		}
		return m_urdf2Model;
	}

	const UrdfDeformable& getDeformable() const
	{
		return m_urdf2Model.m_deformable;
	}

	bool mergeFixedLinks(UrdfModel& model, UrdfLink* link, bool forceFixedBase, int level);
	bool printTree(UrdfLink* link, int level);
	bool recreateModel(UrdfModel& model, UrdfLink* link);

	std::string sourceFileLocation(tinyxml2::XMLElement* e);

	void setSourceFile(const std::string& sourceFile)
	{
		m_urdf2Model.m_sourceFile = sourceFile;
	}

protected:
	UrdfModel m_urdf2Model;
	btAlignedObjectArray<UrdfModel*> m_sdfModels;
	btAlignedObjectArray<UrdfModel*> m_tmpModels;

	bool m_parseSDF;
	int m_activeSdfModel;

	btScalar m_urdfScaling;

	struct UrdfErrorLogger* m_logger;

	//struct CommonFileIOInterface* m_fileIO;

	bool parseTransform(btTransform& tr, tinyxml2::XMLElement* xml, bool parseSDF = false);
	bool parseInertia(UrdfInertia& inertia, tinyxml2::XMLElement* config);
	bool parseGeometry(UrdfGeometry& geom, tinyxml2::XMLElement* g);
	bool parseVisual(UrdfModel& model, UrdfVisual& visual, tinyxml2::XMLElement* config);
	bool parseCollision(UrdfCollision& collision, tinyxml2::XMLElement* config);
	bool initTreeAndRoot(UrdfModel& model);
	bool parseMaterial(UrdfMaterial& material, tinyxml2::XMLElement* config);
	bool parseJointLimits(UrdfJoint& joint, tinyxml2::XMLElement* config);
	bool parseJointDynamics(UrdfJoint& joint, tinyxml2::XMLElement* config);
	bool parseJoint(UrdfJoint& joint, tinyxml2::XMLElement* config);
	bool parseLink(UrdfModel& model, UrdfLink& link, tinyxml2::XMLElement* config);
	bool parseSensor(UrdfModel& model, UrdfLink& link, UrdfJoint& joint, tinyxml2::XMLElement* config);
	bool parseLameCoefficients(LameCoefficients& lameCoefficients, tinyxml2::XMLElement* config);
	bool parseDeformable(UrdfModel& model, tinyxml2::XMLElement* config);

};

}