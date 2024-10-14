#ifndef TRYBGFX_GLTFLOADER_H_HEADER_GUARD
#define TRYBGFX_GLTFLOADER_H_HEADER_GUARD

#include "bgfx/bgfx.h"
#include "bx/string.h"
#include "bx/readerwriter.h"
#include "bx/bounds.h"

#include "bimg/bimg.h"

#include "tinystl/allocator.h"
#include "tinystl/vector.h"
#include "tinystl/unordered_map.h"
#include "tinystl/string.h"

#include "DebugDraw.h"

namespace trybgfx
{
	struct Vertex
	{
		// position
		float m_x;
		float m_y;
		float m_z;

		// texcoord
		float m_u;
		float m_v;

		// indices
		uint8_t m_idx0;
		uint8_t m_idx1;
		uint8_t m_idx2;
		uint8_t m_idx3;

		// weight
		float m_w0;
		float m_w1;
		float m_w2;
		float m_w3;
	};

	struct TGroup
	{
		TGroup();
		void destroy();

		bgfx::VertexLayout m_layout;

		bgfx::VertexBufferHandle m_vbh;
		bgfx::IndexBufferHandle m_ibh;

		uint16_t m_numVertices;
		Vertex*  m_vertices;

		uint32_t m_numIndices;
		uint16_t* m_indices;

		tinystl::string m_material;

		bx::Aabb m_aabb;

		bool m_isStatic;
	};

	typedef tinystl::vector<TGroup> TGroupArray;
	typedef tinystl::unordered_map<tinystl::string, bgfx::TextureHandle> TextureMap;


	struct TJoint
	{
		uint32_t m_parentIdx;
		uint32_t has_tranlation;
		uint32_t has_scale;
		uint32_t has_rotate;

		float m_tranlation[4];
		float m_scale[4];
		float m_rotate[4];
	};
	typedef tinystl::vector<TJoint> TJointArray;

	struct TSkinSkeleton
	{
		TSkinSkeleton();

		void destroy();
		void update();

		float* m_inverseBindMatrix;
		float* m_globalTransformMatrix;

		uint16_t m_numJoint;
		TJointArray m_joints;

		bgfx::UniformHandle m_inverseBindMtxUh;
		bgfx::UniformHandle m_globalTransformMtxUh;
	private:
		bool m_isInverseBindMtxUhSet;
	};

	struct TChannel
	{
		enum Type
		{
			Translation,
			Scale,
			Rotation,
		};

		uint32_t m_startKey;
		uint32_t m_numKeys;

		uint32_t m_startValue;
		uint32_t m_numValues;

		uint32_t m_strike;

		uint32_t m_type;

		uint32_t m_targetJointIdx;
	};
	typedef tinystl::vector<TChannel> TChannelArray;
	typedef tinystl::vector<float> FloatArray;

	struct TAnimation
	{
		float m_duration;

		FloatArray m_keyframes;
		FloatArray m_values;

		uint32_t m_numChannel;
		TChannelArray m_channels;
	};

	typedef tinystl::vector<TAnimation> TAnimationArray;
	struct TMesh
	{
		TMesh();

		void load(const bx::FilePath& _filePath);
		void unload();

		void submit(bgfx::ViewId _id, bgfx::ProgramHandle _pg, const float* _mtx,
			uint64_t _state, DebugDrawEncoder* _dde=NULL);

		// mesh
		TGroupArray m_groups;

		// texture
		TextureMap m_textureMap;
		bgfx::UniformHandle m_sampler;

		// skeleton animtion
		TSkinSkeleton m_skinSkeleton;
		TAnimationArray m_animations;
	};

	struct TAnimator
	{
		TAnimator() = delete;
		TAnimator(TMesh* _mesh);

		void update(float _dt);
		void destroy();

		void play(int32_t _idx);
		void playNext();
		void playPre();
		void stop();

		void step(float _dt);

	private:
		float m_currentTime;
		bool m_playing;

		TMesh* m_mesh;
		TAnimation* m_currentAnimation;

		int32_t m_currentIdx;

		bgfx::UniformHandle m_isPlayingUh;
	};

	TMesh* loadGltf(const bx::FilePath& _filePath);
}

#endif
