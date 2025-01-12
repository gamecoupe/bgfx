#include "Physics.h"

#include <cstdarg>

#include "bx/debug.h"

namespace trybgfx
{
	static void traceImpl(const char* _input, ...)
	{
		va_list list;
		va_start(list, _input);
		char buffer[1024];
		vsnprintf(buffer, sizeof(buffer), _input, list);
		va_end(list);

		bx::debugPrintf(buffer);
	}

	static bool assertFailedImpl(const char* inExpression, const char* inMessage,
		const char* inFile, JPH::uint inLine)
	{
		std::cout << inFile << ":" << inLine
			<< ": (" << inExpression << ") "
			<< (inMessage != nullptr ? inMessage : "")
			<< std::endl;

		// Breakpoint
		return true;
	}

	void TPhysics::init()
	{
		// Register allocation hook
		JPH::RegisterDefaultAllocator();

		// Install callbacks
		JPH::Trace = traceImpl;
		JPH_IF_ENABLE_ASSERTS(JPH::AssertFailed = assertFailedImpl;);

		// Create a factory
		JPH::Factory::sInstance = new JPH::Factory();

		// Register all Jolt physics types
		JPH::RegisterTypes();

		// We need a temp allocator for temporary allocations during the physics update. We're
		// pre-allocating 10 MB to avoid having to do allocations during the physics update.
		// B.t.w. 10 MB is way too much for this example but it is a typical value you can use.
		// If you don't want to pre-allocate you can also use TempAllocatorMalloc to fall back to
		// malloc / free.
		m_tempAllocator = new JPH::TempAllocatorImpl(10 * 1024 * 1024);

		// We need a job system that will execute physics jobs on multiple threads. Typically
		// you would implement the JobSystem interface yourself and let Jolt Physics run on top
		// of your own job scheduler. JobSystemThreadPool is an example implementation.
		m_jobSystem = new JPH::JobSystemThreadPool(JPH::cMaxPhysicsJobs, JPH::cMaxPhysicsBarriers,
			JPH::thread::hardware_concurrency() - 1);

		// This is the max amount of rigid bodies that you can add to the physics system.
		// If you try to add more you'll get an error.
		// Note: This value is low because this is a simple test.
		// For a real project use something in the order of 65536.
		const unsigned int cMaxBodies = 1024;

		// This determines how many mutexes to allocate to protect rigid bodies from concurrent access.
		// Set it to 0 for the default settings.
		const unsigned int cNumBodyMutexes = 0;

		// This is the max amount of body pairs that can be queued at any time
		// (the broad phase will detect overlapping body pairs based on their bounding boxes and
		// will insert them into a queue for the narrowphase).
		// If you make this buffer too small the queue will fill up and the broad phase jobs will start to
		// do narrow phase work. This is slightly less efficient.
		// Note: This value is low because this is a simple test.
		// For a real project use something in the order of 65536.
		const unsigned int cMaxBodyPairs = 1024;

		// This is the maximum size of the contact constraint buffer.
		// If more contacts (collisions between bodies) are detected than this
		// number then these contacts will be ignored and
		// bodies will start interpenetrating / fall through the world.
		// Note: This value is low because this is a simple test.
		// For a real project use something in the order of 10240.
		const unsigned int cMaxContactConstraints = 1024;

		// ---------------

		// Now we can create the actual physics system.
		m_physicsSystem = new JPH::PhysicsSystem();
		m_physicsSystem->Init(cMaxBodies, cNumBodyMutexes, cMaxBodyPairs, cMaxContactConstraints,
			m_broadPhaseLayerInterface, m_objectVsBroadphaseLayerFilter, m_objectVsObjectLayerFilter);

		//s_physicsSystem->SetBodyActivationListener(&body_activation_listener);
		m_physicsSystem->SetBodyActivationListener(&m_bodyActivationListener);

		//s_physicsSystem->SetContactListener(&contact_listener);
		m_physicsSystem->SetContactListener(&m_contactListener);

		//m_physicsSystem->SetGravity(JPH::Vec3(0, -1000.0f, 0));
		m_physicsSystem->SetGravity(JPH::Vec3(0, -50.0f, 0));
	}

	void TPhysics::destroy()
	{
	}

	void TPhysics::optimizeBroadPhase()
	{
		// Optional step: Before starting the physics simulation you can optimize the broad phase.
		// This improves collision detection performance (it's pointless here because we only have 2 bodies).
		// You should definitely not call this every frame or when e.g. streaming in a new level section
		// as it is an expensive operation.
		// Instead insert all new objects in batches instead of 1 at a time to keep the broad phase efficient.
		m_physicsSystem->OptimizeBroadPhase();
	}

	void TPhysics::addVolumeShape(bx::Sphere& _sphere)
	{
		//// Now create a dynamic body to bounce on the floor
		//// Note that this uses the shorthand version of creating and adding a body to the world
		//JPH::SphereShape* sphereShape = new JPH::SphereShape(_sphere.radius);
		////JPH::BodyCreationSettings sphere_settings(sphereShape, JPH::RVec3(_sphere.center.x, _sphere.center.y, _sphere.center.z),
		////	JPH::Quat::sIdentity(), JPH::EMotionType::Dynamic, Layers::MOVING);
		//JPH::BodyCreationSettings sphere_settings(sphereShape, JPH::RVec3(_sphere.center.x, _sphere.center.y, _sphere.center.z),
		//	JPH::Quat::sIdentity(), JPH::EMotionType::Dynamic, Layers::MOVING);
		////sphere_settings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
		////sphere_settings.mMassPropertiesOverride.mMass = 1.0f;
		//sphere_settings.mFriction = 1.0f;

		////auto aa = sphere_settings.GetMassProperties().mMass;

		//// The main way to interact with the bodies in the physics system is through the body interface. There is a locking and a non-locking
		//// variant of this. We're going to use the locking version (even though we're not planning to access bodies from multiple threads)
		//JPH::BodyInterface& bodyInterface = m_physicsSystem->GetBodyInterface();
		//m_sphereId = bodyInterface.CreateAndAddBody(sphere_settings, JPH::EActivation::Activate);

		////bodyInterface->SetMotionType(bodyID, EMotionType::Static);


		JPH::RefConst<JPH::Shape> sphereShape = new JPH::SphereShape(_sphere.radius);
		JPH::RVec3 position = JPH::RVec3(_sphere.center.x, _sphere.center.y, _sphere.center.z);
		//JPH::RefConst<JPH::Shape> charactorShape = JPH::RotatedTranslatedShapeSettings(position, JPH::Quat::sIdentity(), sphereShape).Create().Get();

		// Create 'player' character
		JPH::Ref<JPH::CharacterSettings> settings = new JPH::CharacterSettings();
		settings->mMaxSlopeAngle = JPH::DegreesToRadians(45.0f);
		settings->mLayer = Layers::MOVING;
		settings->mShape = sphereShape;
		settings->mFriction = 1.0f;
		//settings->mSupportingVolume = JPH::Plane(JPH::Vec3::sAxisY(), 0.0f); // Accept contacts that touch the lower sphere of the capsule
		m_character = new JPH::Character(settings, position, JPH::Quat::sIdentity(), 0, m_physicsSystem);
		m_character->AddToPhysicsSystem(JPH::EActivation::Activate);
	}

	void TPhysics::addMeshShape(trybgfx::TMesh* _mesh)
	{
		JPH::VertexList vertices;
		JPH::IndexedTriangleList triangles;

		JPH::BodyInterface& bodyInterface = m_physicsSystem->GetBodyInterface();
		for (TGroupArray::iterator it = _mesh->m_groups.begin(), itEnd = _mesh->m_groups.end(); it != itEnd; ++it)
		{
			TGroup& group = *it;

			vertices.clear();
			for (uint32_t i = 0; i < group.m_numVertices; i++)
			{
				float x = group.m_vertices[i].m_x;
				float y = group.m_vertices[i].m_y;
				float z = group.m_vertices[i].m_z;
				vertices.push_back(JPH::Float3(x, y, z));
			}

			triangles.clear();
			for (uint32_t i = 0; i < group.m_numIndices; i += 3)
			{
				float x = group.m_indices[i];
				float y = group.m_indices[i + 1];
				float z = group.m_indices[i + 2];
				triangles.push_back(JPH::IndexedTriangle(x, y, z));
			}

			JPH::MeshShapeSettings mesh(std::move(vertices), std::move(triangles));
			mesh.SetEmbedded();

			JPH::BodyCreationSettings mesh_cylinder(&mesh,
				JPH::RVec3(_mesh->m_originPosition.x, _mesh->m_originPosition.y, _mesh->m_originPosition.z),
				JPH::Quat::sIdentity(), JPH::EMotionType::Static, Layers::NON_MOVING);

			//mBodyInterface->CreateAndAddBody(mesh_cylinder, EActivation::DontActivate);

			bodyInterface.CreateAndAddBody(mesh_cylinder, JPH::EActivation::Activate);
		}


	}

	void TPhysics::addVolumeShape(bx::Capsule& _capsule)
	{
		// Now create a dynamic body to bounce on the floor
		// Note that this uses the shorthand version of creating and adding a body to the world
		JPH::CapsuleShape* capsule = new JPH::CapsuleShape((_capsule.end.y - _capsule.pos.y)/2, _capsule.radius);
		JPH::BodyCreationSettings sphere_settings(capsule, JPH::RVec3(_capsule.pos.x, _capsule.pos.y + (_capsule.end.y - _capsule.pos.y) / 2, _capsule.pos.z),
			JPH::Quat::sIdentity(), JPH::EMotionType::Dynamic, Layers::MOVING);

		// The main way to interact with the bodies in the physics system is through the body interface. There is a locking and a non-locking
		// variant of this. We're going to use the locking version (even though we're not planning to access bodies from multiple threads)
		JPH::BodyInterface& bodyInterface = m_physicsSystem->GetBodyInterface();
		m_sphereId = bodyInterface.CreateAndAddBody(sphere_settings, JPH::EActivation::Activate);
	}

	void TPhysics::addVolumeShape(bx::Vec3& _halfExtent)
	{
		JPH::Vec3 halfExtent = { _halfExtent.x, _halfExtent.y, _halfExtent.z };
		JPH::BoxShape* boxShape = new JPH::BoxShape(halfExtent);

		// Create the settings for the body itself. Note that here you can also set other properties like the restitution / friction.
		JPH::BodyCreationSettings floor_settings(boxShape, JPH::RVec3(0.0f, -1.0f, 0.0f),
			JPH::Quat::Quat::sIdentity(), JPH::EMotionType::Static, Layers::NON_MOVING);

		JPH::BodyInterface& bodyInterface = m_physicsSystem->GetBodyInterface();
		bodyInterface.CreateAndAddBody(floor_settings, JPH::EActivation::DontActivate);
	}

	void TPhysics::addVolumeShape(bx::Aabb& _aabb, bx::Vec3& _position, bx::Vec3& _rotation, bool _isDynamic)
	{
		JPH::Vec3 halfExtent = { (_aabb.max.x - _aabb.min.x)/2.0f, (_aabb.max.y - _aabb.min.y)/2.0f, (_aabb.max.z - _aabb.min.z)/2.0f };
		JPH::BoxShape* boxShape = new JPH::BoxShape(halfExtent);

		// Create the settings for the body itself. Note that here you can also set other properties like the restitution / friction.
		JPH::Quat rotation = JPH::Quat::sEulerAngles(JPH::Vec3(-_rotation.x, -_rotation.y, -_rotation.z));
		JPH::BodyCreationSettings floor_settings(boxShape, JPH::RVec3(_position.x, _position.y - halfExtent.GetY(), _position.z),
			rotation, _isDynamic ? JPH::EMotionType::Dynamic : JPH::EMotionType::Static, _isDynamic ? Layers::MOVING :Layers::NON_MOVING);
		floor_settings.mFriction = 1.0f;

		auto aa = floor_settings.GetMassProperties().mMass;

		JPH::BodyInterface& bodyInterface = m_physicsSystem->GetBodyInterface();
		if (_isDynamic)
		{
			m_sphereId = bodyInterface.CreateAndAddBody(floor_settings, JPH::EActivation::Activate);
		}
		else
		{
			bodyInterface.CreateAndAddBody(floor_settings, JPH::EActivation::DontActivate);
		}
	}

	void TPhysics::setLinearVelocity(bx::Vec3& velocity)
	{
		JPH::Vec3 currentVelocity = m_character->GetLinearVelocity();
		JPH::Vec3 newVelocity(velocity.x, currentVelocity.GetY(), velocity.z);

		//JPH::Character::EGroundState groundState = m_character->GetGroundState();
		//if (m_character->IsSupported())
		//{
		//	newVelocity.SetY(0.0f);
		//}

		//JPH::BodyInterface& bodyInterface = m_physicsSystem->GetBodyInterface();
		//bodyInterface.SetLinearVelocity(m_sphereId, new_velocity);

		m_character->SetLinearVelocity(newVelocity);
	}

	void TPhysics::UpdatePhysics(const float& _deltaTime, bx::Vec3& _position, bx::Vec3& _velocity, bx::Vec3& _angular)
	{
		// If you take larger steps than 1 / 60th of a second you need to do multiple collision steps in order to keep the simulation stable. Do 1 collision step per 1 / 60th of a second (round up).
		const int cCollisionSteps = 1;

		// Step the world
		m_physicsSystem->Update(_deltaTime, cCollisionSteps, m_tempAllocator, m_jobSystem);

		//JPH::BodyInterface& bodyInterface = m_physicsSystem->GetBodyInterface();


		//// Output current position and velocity of the sphere
		//JPH::RVec3 position = bodyInterface.GetPosition(m_sphereId);
		//JPH::Vec3 velocity = bodyInterface.GetLinearVelocity(m_sphereId);

		////std::cout << "Position = (" << position.GetX() << ", " << position.GetY() <<
		////	", " << position.GetZ() << "), Velocity = (" << velocity.GetX() <<
		////	", " << velocity.GetY() << ", " << velocity.GetZ() << ")" << std::endl;

		//_position.x = position.GetX();
		//_position.y = position.GetY();
		//_position.z = position.GetZ();

		//_velocity.x = velocity.GetX();
		//_velocity.y = velocity.GetY();
		//_velocity.z = velocity.GetZ();


		//JPH::RMat44 new_space_ship_transform = bodyInterface.GetCenterOfMassTransform(m_sphereId);
		//JPH::Quat rotation = new_space_ship_transform.GetRotation().GetQuaternion();
		//JPH::Vec3 vRotation = rotation.GetEulerAngles();

		//_angular.x = vRotation.GetX();
		//_angular.y = vRotation.GetY();
		//_angular.z = vRotation.GetZ();





		JPH::RVec3 position = m_character->GetPosition();
		_position.x = position.GetX();
		_position.y = position.GetY();
		_position.z = position.GetZ();

		JPH::Vec3 velocity = m_character->GetLinearVelocity();

		std::cout << "Position = (" << position.GetX() << ", " << position.GetY() <<
			", " << position.GetZ() << "), Velocity = (" << velocity.GetX() <<
			", " << velocity.GetY() << ", " << velocity.GetZ() << ")" << std::endl;


		m_character->PostSimulation(0.05f);
	}
}
