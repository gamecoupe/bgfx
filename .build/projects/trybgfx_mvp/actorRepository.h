#ifndef TRYBGFX_ACTOR_REPOSITORY_H_PHYICS_GUARD
#define TRYBGFX_ACTOR_REPOSITORY_H_PHYICS_GUARD

#include <Jolt/Jolt.h>
#include <Jolt/Physics/PhysicsSystem.h>

#include "tinystl/allocator.h"
#include "tinystl/vector.h"
#include "tinystl/unordered_map.h"
#include "tinystl/string.h"

#include "bx/math.h"

#include "gltfloader.h"
#include "Physics.h"

namespace trybgfx
{
	class TStaticActor
	{
	public:
		TStaticActor(TPhysics* _physics, tinystl::string _name, bool _isVisible, bool _applyPhysic)
			:m_physics(_physics), m_name(_name), m_isVisible(_isVisible), m_applyPhysic(_applyPhysic), m_unmanagedMesh(false)
		{
		}

		void load(const bx::FilePath& _filePath)
		{
			m_mesh = trybgfx::loadGltf(_filePath);

			if (m_applyPhysic)
			{
				//m_physics->addVolumeShape(m_mesh->m_aabb, m_mesh->m_originPosition, m_mesh->m_originRotation);

				m_physics->addMeshShape(m_mesh);
			}
		}

		void load(TMesh* _mesh)
		{
			m_mesh = _mesh;
			m_unmanagedMesh = true;

			if (m_applyPhysic)
			{
				m_physics->addVolumeShape(m_mesh->m_aabb, m_mesh->m_originPosition, m_mesh->m_originRotation);
			}
		}

		void unload()
		{
			if (!m_unmanagedMesh)
			{
				m_mesh->unload();
				delete m_mesh;
			}
		}

		void submit(bgfx::ProgramHandle _pg)
		{
			if (!m_isVisible) return;

			float rotateMtx[16];
			bx::mtxRotateXYZ(rotateMtx, m_mesh->m_originRotation.x, m_mesh->m_originRotation.y, m_mesh->m_originRotation.z);
			float scaleMtx[16];
			bx::mtxScale(scaleMtx, 1.0f);
			float translateMtx[16];
			bx::mtxTranslate(translateMtx, m_mesh->m_originPosition.x, m_mesh->m_originPosition.y, m_mesh->m_originPosition.z);

			float scaleRotateMtx[16];
			bx::mtxMul(scaleRotateMtx, scaleMtx, rotateMtx);
			float finalMtx[16];
			bx::mtxMul(finalMtx, scaleRotateMtx, translateMtx);

			m_mesh->submit(0, _pg, finalMtx, BGFX_STATE_MASK);
		}
	private:
		TMesh* m_mesh = nullptr;

		JPH::BodyID m_bodyId;

		tinystl::string m_name;

		TPhysics* m_physics = nullptr;

		bool m_isVisible = false;
		bool m_applyPhysic = false;

		bool m_unmanagedMesh = false;
	};


	typedef tinystl::vector<TStaticActor*> TStaticActorRepository;
	typedef tinystl::unordered_map<tinystl::string, TStaticActor*> TStaticActorDic;
}

#endif
