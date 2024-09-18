#ifndef TRYBGFX_UTILIES_H_HEADER_GUARD
#define TRYBGFX_UTILIES_H_HEADER_GUARD

#include "bgfx/bgfx.h"
#include "bx/string.h"
#include "bx/readerwriter.h"
#include "bx/bounds.h"

#include "bimg/bimg.h"

#include "tinystl/allocator.h"
#include "tinystl/vector.h"
#include "tinystl/unordered_map.h"
#include "tinystl/string.h"

#define DBG_STRINGIZE(_x) DBG_STRINGIZE_(_x)
#define DBG_STRINGIZE_(_x) #_x
#define DBG_FILE_LINE_LITERAL "" __FILE__ "(" DBG_STRINGIZE(__LINE__) "): "
#define DBG(_format, ...) bx::debugPrintf(DBG_FILE_LINE_LITERAL "" _format "\n", ##__VA_ARGS__)


#define MAX_JOINTS	96

struct Primitive
{
	uint32_t m_startIndex;
	uint32_t m_numIndices;

	uint32_t m_startVertex;
	uint32_t m_numVertices;

	bx::Sphere m_sphere;
	bx::Aabb   m_aabb;
	bx::Obb    m_obb;
};

typedef tinystl::vector<Primitive> PrimitiveArray;

struct Group
{
	Group();
	void reset();

	bgfx::VertexBufferHandle m_vbh;
	bgfx::IndexBufferHandle m_ibh;

	uint16_t m_numVertices;
	uint8_t* m_vertices;

	uint32_t m_numIndices;
	uint16_t* m_indices;

	bx::Sphere m_sphere;
	bx::Aabb m_aabb;
	bx::Obb m_obb;

	PrimitiveArray m_prims;

	tinystl::string m_material;
};

typedef tinystl::vector<Group> GroupArray;

typedef tinystl::unordered_map<tinystl::string, bgfx::TextureHandle> TextureMap;

struct Mtx4
{
	float m_m[16];
};

struct Joint
{
	uint32_t m_parentIdx;
	uint32_t has_tranlation;
	uint32_t has_scale;
	uint32_t has_rotate;

	float m_tranlation[4];
	float m_scale[4];
	float m_rotate[4];
};

struct Channel
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

struct Animation
{
	Animation();
	void destroy();

	float m_duration;

	uint32_t m_numKeyframe;
	float* m_keyframes;

	uint32_t m_numValue;
	float* m_values;

	uint32_t m_numChannel;
	Channel* m_channels;
};


struct SkinSkeleton
{
	SkinSkeleton();
	void destroy();

	void update();

	float* m_inverseBindMatrix;
	float* m_globalTransformMatrix;

	uint16_t m_numJoint;
	Joint* m_joints;

	bgfx::UniformHandle m_inverseBindMtxUh;
	bool m_isInverseBindMtxUhSet;

	bgfx::UniformHandle m_globalTransformMtxUh;
};

struct Animator
{
	Animator(Animation* _animation, SkinSkeleton* _skin);

	float m_currentTime;
	bool m_playing;

	Animation* m_currentAnimation;
	SkinSkeleton* m_currentSkinSkeleton;

	void update(float _dt);
	void play();
	void stop();

	void step(float _dt);

private:
	void calcGlobalTransformMtx(float* _translateArray, float* _rotateArray, float* _scaleArray, bool* _changedArray,
		Joint* _joints, uint32_t idx, float *result);
	void calcGlobalTransformMtx2(float* _translateArray, float* _rotateArray, float* _scaleArray, bool* _changedArray,
		Joint* _joints, uint32_t idx, float* result);
};

struct Mesh
{
	Mesh();

	void load(bx::ReaderSeekerI* _reader, bool _ramcopy);
	void unload();

	void update(float _dt);
	void submit(bgfx::ViewId _id, bgfx::ProgramHandle _pg, const float* _mtx, uint64_t _state);

	void printVertexLayout();

	bgfx::VertexLayout m_layout;
	GroupArray m_groups;

	tinystl::vector<bgfx::TextureHandle*> m_meshTextures;

	TextureMap m_textureMap;

	bgfx::UniformHandle m_sampler;

	SkinSkeleton m_skinSkeleton;

	Animation m_animation;

	Animator m_animator;
};


bgfx::ProgramHandle loadShaderProgram(const bx::StringView& _vsName, const bx::StringView& _fsName);

Mesh* loadMesh(const bx::FilePath& _filePath, bool _ramcopy = false);

void unloadMesh(Mesh* _mesh);

bgfx::TextureHandle loadTexture(const bx::FilePath& _filePath,
	uint64_t _flags = BGFX_TEXTURE_NONE | BGFX_SAMPLER_NONE,
	uint8_t _skip = 0, bgfx::TextureInfo* _info = NULL,
	bimg::Orientation::Enum* _orientation = NULL);


#endif // !TRYBGFX_UTILIES
