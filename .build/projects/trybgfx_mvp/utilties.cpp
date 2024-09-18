#include "utilties.h"

#include "bx/debug.h"
#include "bx/file.h"
#include "bx/string.h"

#include "bimg/decode.h"

#include "meshoptimizer/src/meshoptimizer.h"

#include "tinystl/allocator.h"

#include <math.h>

static bx::DefaultAllocator s_defaultAllocator;
static bx::AllocatorI* s_allocator = &s_defaultAllocator;
typedef bx::StringT<&s_allocator> String;

static float imat[16] = {
	1.0f, 0.0f, 0.0f, 0.0f,
	0.0f, 1.0f, 0.0f, 0.0f,
	0.0f, 0.0f, 1.0f, 0.0f,
	0.0f, 0.0f, 0.0f, 1.0f,
};

static String s_currentDir;

class FileReader : public  bx::FileReader
{
	typedef bx::FileReader super;

public:
	virtual bool open(const bx::FilePath& _filePath, bx::Error* _err) override
	{
		String filePath(s_currentDir);
		filePath.append(_filePath);
		return super::open(filePath, _err);
	}
};

const bgfx::Memory* loadMem(bx::FileReaderI* _reader, const bx::FilePath& _filePath)
{
	if (bx::open(_reader, _filePath))
	{
		uint32_t size = (uint32_t)bx::getSize(_reader);
		const bgfx::Memory* mem = bgfx::alloc(size + 1);
		bx::read(_reader, mem->data, size, bx::ErrorAssert{});
		bx::close(_reader);

		mem->data[mem->size - 1] = '\0';
		return mem;
	}

	DBG("Failed to load %s", _filePath);
	return NULL;
}

bgfx::ShaderHandle loadShader(bx::FileReaderI* _reader,
	const bx::StringView& _name)
{
	bx::FilePath filePath("assets/shaders/");

	char fileName[512];
	bx::strCopy(fileName, BX_COUNTOF(fileName), _name);
	bx::strCat(fileName, BX_COUNTOF(fileName), ".bin");

	filePath.join(fileName);

	bgfx::ShaderHandle handle = bgfx::createShader(loadMem(_reader, filePath));
	bgfx::setName(handle, _name.getPtr(), _name.getLength());

	return handle;
}

bgfx::ProgramHandle loadShaderProgram(bx::FileReaderI* _reader,
	const bx::StringView& _vsName, const bx::StringView& _fsName)
{
	bgfx::ShaderHandle vsh = loadShader(_reader, _vsName);
	bgfx::ShaderHandle fsh = BGFX_INVALID_HANDLE;
	if (!_fsName.isEmpty())
	{
		fsh = loadShader(_reader, _fsName);
	}

	return bgfx::createProgram(vsh, fsh, true);
}

bgfx::ProgramHandle loadShaderProgram(const bx::StringView& _vsName, const bx::StringView& _fsName) 
{
	bx::FileReaderI* reader = BX_NEW(s_allocator, FileReader);
	return loadShaderProgram(reader, _vsName, _fsName);
}

Group::Group()
{
	reset();
}

void Group::reset()
{
	m_vbh.idx = bgfx::kInvalidHandle;
	m_ibh.idx = bgfx::kInvalidHandle;
	m_numVertices = 0;
	m_vertices = NULL;
	m_numIndices = 0;
	m_indices = NULL;
	m_prims.clear();
}

namespace bgfx
{
	int32_t read(bx::ReaderI* _reader, bgfx::VertexLayout& _layout, bx::Error* _err);
}

SkinSkeleton::SkinSkeleton()
{
	m_inverseBindMtxUh.idx = bgfx::kInvalidHandle;
	m_isInverseBindMtxUhSet = false;

	m_globalTransformMtxUh.idx = bgfx::kInvalidHandle;

	m_joints = NULL;
}

void SkinSkeleton::destroy()
{
	if (bgfx::isValid(m_inverseBindMtxUh))
	{
		bgfx::destroy(m_inverseBindMtxUh);
	}

	if (bgfx::isValid(m_globalTransformMtxUh))
	{
		bgfx::destroy(m_globalTransformMtxUh);
	}

	if (NULL != m_joints)
	{
		bx::free(s_allocator, m_joints);
	}

	if (NULL != m_inverseBindMatrix)
	{
		bx::free(s_allocator, m_inverseBindMatrix);
	}

	if (NULL != m_globalTransformMatrix)
	{
		bx::free(s_allocator, m_globalTransformMatrix);
	}
}

void SkinSkeleton::update()
{
	if (bgfx::isValid(m_inverseBindMtxUh) && !m_isInverseBindMtxUhSet)
	{
		m_isInverseBindMtxUhSet = true;
		bgfx::setUniform(m_inverseBindMtxUh, m_inverseBindMatrix, m_numJoint);
	}

	if (bgfx::isValid(m_globalTransformMtxUh))
	{
		bgfx::setUniform(m_globalTransformMtxUh, m_globalTransformMatrix, m_numJoint);
	}
}

Animation::Animation()
{
	m_channels = NULL;
	m_keyframes = NULL;
	m_values = NULL;
}

void Animation::destroy()
{
	if (NULL != m_channels)
	{
		bx::free(s_allocator, m_channels);
	}

	if (NULL != m_keyframes)
	{
		bx::free(s_allocator, m_keyframes);
	}

	if (NULL != m_values)
	{
		bx::free(s_allocator, m_values);
	}
}

Animator::Animator(Animation* _animation, SkinSkeleton* _skin)
{
	m_currentAnimation = _animation;
	m_currentSkinSkeleton = _skin;
	m_currentTime = 0.0f;
	m_playing = false;
}

void Animator::step(float _dt)
{
	m_currentTime += _dt;
	float animationTime = fmod(m_currentTime, m_currentAnimation->m_duration);

	//BX_TRACE("%f", animationTime);

	float translateArray[MAX_JOINTS * 3];
	float rotateArray[MAX_JOINTS * 4];
	float scaleArray[MAX_JOINTS * 3];
	bool changedArray[MAX_JOINTS * 3];	// 0: translate 1: scale 2: rotate

	bx::memSet(changedArray, false, sizeof(changedArray));

	for (size_t i = 0; i < m_currentAnimation->m_numChannel; i++)
	{
		const Channel& channel = m_currentAnimation->m_channels[i];

		float* keyframes = m_currentAnimation->m_keyframes + channel.m_startKey;
		float* values = m_currentAnimation->m_values + channel.m_startValue;

		for (size_t j = 0; j < channel.m_numKeys - 1; j++)
		{
			float previousTime = keyframes[j];
			float nextTime = keyframes[j + 1];

			if (animationTime >= previousTime && animationTime <= nextTime)
			{
				float interpolationFactor = (animationTime - previousTime) / (nextTime - previousTime);

				float* previousValue = values + j * channel.m_strike;
				float* nextValue = values + (j + 1) * channel.m_strike;

				if (channel.m_type == (uint32_t)Channel::Type::Translation)
				{
					changedArray[channel.m_targetJointIdx * 3] = true;

					bx::Vec3 previouV = {
						*previousValue ,
						*(previousValue + 1) ,
						*(previousValue + 2)
					};

					bx::Vec3 nextV = {
						*nextValue ,
						*(nextValue + 1) ,
						*(nextValue + 2)
					};

					bx::Vec3 result = bx::lerp(previouV, nextV, interpolationFactor);
					translateArray[channel.m_targetJointIdx * channel.m_strike] = result.x;
					translateArray[channel.m_targetJointIdx * channel.m_strike + 1] = result.y;
					translateArray[channel.m_targetJointIdx * channel.m_strike + 2] = result.z;
				}
				else if (channel.m_type == (uint32_t)Channel::Type::Scale)
				{
					changedArray[channel.m_targetJointIdx * 3 + 1] = true;

					bx::Vec3 previouV = {
						*previousValue ,
						*(previousValue + 1) ,
						*(previousValue + 2)
					};

					bx::Vec3 nextV = {
						*nextValue ,
						*(nextValue + 1) ,
						*(nextValue + 2)
					};

					bx::Vec3 result = bx::lerp(previouV, nextV, interpolationFactor);
					scaleArray[channel.m_targetJointIdx * channel.m_strike] = result.x;
					scaleArray[channel.m_targetJointIdx * channel.m_strike + 1] = result.y;
					scaleArray[channel.m_targetJointIdx * channel.m_strike + 2] = result.z;
				}
				else if (channel.m_type == (uint32_t)Channel::Type::Rotation)
				{
					changedArray[channel.m_targetJointIdx * 3 + 2] = true;

					bx::Quaternion previouQ = {
						*previousValue ,
						*(previousValue + 1) ,
						*(previousValue + 2) ,
						*(previousValue + 3)
					};

					bx::Quaternion nextQ = {
						*nextValue ,
						*(nextValue + 1) ,
						*(nextValue + 2) ,
						*(nextValue + 3)
					};

					bx::Quaternion result = bx::lerp(previouQ, nextQ, interpolationFactor);
					rotateArray[channel.m_targetJointIdx * channel.m_strike] = result.x;
					rotateArray[channel.m_targetJointIdx * channel.m_strike + 1] = result.y;
					rotateArray[channel.m_targetJointIdx * channel.m_strike + 2] = result.z;
					rotateArray[channel.m_targetJointIdx * channel.m_strike + 3] = result.w;
				}
				else
				{
					DBG("Invalid channel.m_type");
				}

				break;
			}
		}
	}

	for (size_t i = 0; i < m_currentSkinSkeleton->m_numJoint; i++)
	{
		float* globalTransformMtx = m_currentSkinSkeleton->m_globalTransformMatrix + i * 16;
		calcGlobalTransformMtx(translateArray, rotateArray, scaleArray, changedArray, m_currentSkinSkeleton->m_joints, i,
			globalTransformMtx);
	}
}

void Animator::update(float _dt)
{
	if (!m_playing)
	{
		return;
	}

	step(_dt);
}


void Animator::calcGlobalTransformMtx(float* _translateArray, float* _rotateArray, float* _scaleArray, bool* _changedArray,
	Joint* _joints, uint32_t idx, float* result)
{
	Joint* currentJoint = _joints + idx;

	bool tranlationChanged = _changedArray[idx * 3];
	bool scaleChanged = _changedArray[idx * 3 + 1];
	bool rotateChanged = _changedArray[idx * 3 + 2];

	float localTranformMtx[16];

	float scale[16];
	if (scaleChanged)
	{
		// tranlation
		float f0 = _scaleArray[idx * 3];
		float f1 = _scaleArray[idx * 3 + 1];
		float f2 = _scaleArray[idx * 3 + 2];
		bx::mtxScale(scale, f0, f1, f2);
	}
	else
	if (currentJoint->has_scale)
	{
		bx::mtxScale(scale,
			currentJoint->m_scale[0],
			currentJoint->m_scale[1],
			currentJoint->m_scale[2]);
	}
	else
	{
		for (size_t j = 0; j < 16; j++)
		{
			scale[j] = imat[j];
		}
	}


	float rotate[16];
	if (rotateChanged)
	{
		// tranlation
		bx::Quaternion q = {
			_rotateArray[idx * 4],
			_rotateArray[idx * 4 + 1],
			_rotateArray[idx * 4 + 2],
			-_rotateArray[idx * 4 + 3],
		};

		bx::Quaternion normalizedQ = bx::normalize(q);
		bx::mtxFromQuaternion(rotate, q);
	}
	else
	if (currentJoint->has_rotate)
	{
		bx::Quaternion q = {
			currentJoint->m_rotate[0],
			currentJoint->m_rotate[1],
			currentJoint->m_rotate[2],
			-currentJoint->m_rotate[3],
		};
		bx::mtxFromQuaternion(rotate, q);
	}
	else
	{
		for (size_t j = 0; j < 16; j++)
		{
			rotate[j] = imat[j];
		}
	}

	float trans[16];
	if (tranlationChanged)
	{
		// tranlation
		float mtx[16];
		float f0 = _translateArray[idx * 3];
		float f1 = _translateArray[idx * 3 + 1];
		float f2 = _translateArray[idx * 3 + 2];
		bx::mtxTranslate(trans, f0, f1, f2);
	}
	else
	if (currentJoint->has_tranlation)
	{
		bx::mtxTranslate(trans,
			currentJoint->m_tranlation[0],
			currentJoint->m_tranlation[1],
			currentJoint->m_tranlation[2]);
	}
	else
	{
		for (size_t j = 0; j < 16; j++)
		{
			trans[j] = imat[j];
		}
	}


	float scaleRotate[16];
	bx::mtxMul(scaleRotate, scale, rotate);

	bx::mtxMul(localTranformMtx, scaleRotate, trans);

	if (currentJoint->m_parentIdx != UINT8_MAX)
	{
		float parentTranformMtx[16];
		calcGlobalTransformMtx(_translateArray, _rotateArray, _scaleArray, _changedArray, _joints,
			currentJoint->m_parentIdx, parentTranformMtx);
		bx::mtxMul(result, localTranformMtx, parentTranformMtx);
	}
	else
	{
		for (size_t i = 0; i < 16; i++)
		{
			result[i] = localTranformMtx[i];
		}
	}

	int aa = 0;
}

void Animator::play()
{
	m_currentTime = 0.0f;
	m_playing = true;
}

void Animator::stop()
{
	m_playing = false;
}


Mesh::Mesh():
	m_animator(Animator(&m_animation, &m_skinSkeleton))
{
}

void Mesh::load(bx::ReaderSeekerI* _reader, bool _ramcopy)
{
	constexpr uint32_t kChunkVertexBuffer = BX_MAKEFOURCC('V', 'B', ' ', 0x1);
	constexpr uint32_t kChunkVertexBufferCompressed = BX_MAKEFOURCC('V', 'B', 'C', 0x0);
	constexpr uint32_t kChunkIndexBuffer = BX_MAKEFOURCC('I', 'B', ' ', 0x0);
	constexpr uint32_t kChunkIndexBufferCompressed = BX_MAKEFOURCC('I', 'B', 'C', 0x1);
	constexpr uint32_t kChunkPrimitive = BX_MAKEFOURCC('P', 'R', 'I', 0x0);

	constexpr uint32_t kChunkJointBuffer = BX_MAKEFOURCC('J', 'B', ' ', 0x0);
	constexpr uint32_t kChunkAnimationBuffer = BX_MAKEFOURCC('A', 'B', ' ', 0x0);

	Group group;

	uint32_t chunk;
	bx::Error err;

	while (4 == bx::read(_reader, chunk, &err) && err.isOk())
	{
		switch (chunk)
		{
			case kChunkVertexBuffer:
			{
				read(_reader, group.m_sphere, &err);
				read(_reader, group.m_aabb, &err);
				read(_reader, group.m_obb, &err);

				read(_reader, m_layout, &err);

				uint16_t stride = m_layout.getStride();

				read(_reader, group.m_numVertices, &err);

				const bgfx::Memory* mem = bgfx::alloc(group.m_numVertices * stride);

				read(_reader, mem->data, mem->size, &err);

				if (_ramcopy)
				{
					group.m_vertices = (uint8_t*)bx::alloc(s_allocator, group.m_numVertices * stride);
					bx::memCopy(group.m_vertices, mem->data, mem->size);
				}

				group.m_vbh = bgfx::createVertexBuffer(mem, m_layout);
			}
			break;

			case kChunkVertexBufferCompressed:
			{
				read(_reader, group.m_sphere, &err);
				read(_reader, group.m_aabb, &err);
				read(_reader, group.m_obb, &err);

				read(_reader, m_layout, &err);

				uint16_t stride = m_layout.getStride();

				read(_reader, group.m_numVertices, &err);

				const bgfx::Memory* mem = bgfx::alloc(group.m_numVertices * stride);

				uint32_t compressedSize;
				bx::read(_reader, compressedSize, &err);

				void* compressedVertices = bx::alloc(s_allocator, compressedSize);
				bx::read(_reader, compressedVertices, compressedSize, &err);

				meshopt_decodeVertexBuffer(mem->data, group.m_numVertices, stride, (uint8_t*)compressedVertices, compressedSize);

				bx::free(s_allocator, compressedVertices);

				if (_ramcopy)
				{
					group.m_vertices = (uint8_t*)bx::alloc(s_allocator, group.m_numVertices * stride);
					bx::memCopy(group.m_vertices, mem->data, mem->size);
				}

				group.m_vbh = bgfx::createVertexBuffer(mem, m_layout);
			}
			break;

			case kChunkIndexBuffer:
			{
				read(_reader, group.m_numIndices, &err);

				const bgfx::Memory* mem = bgfx::alloc(group.m_numIndices * 2);
				read(_reader, mem->data, mem->size, &err);

				if (_ramcopy)
				{
					group.m_indices = (uint16_t*)bx::alloc(s_allocator, group.m_numIndices * 2);
					bx::memCopy(group.m_indices, mem->data, mem->size);
				}

				group.m_ibh = bgfx::createIndexBuffer(mem);
			}
			break;

			case kChunkIndexBufferCompressed:
			{
				bx::read(_reader, group.m_numIndices, &err);

				const bgfx::Memory* mem = bgfx::alloc(group.m_numIndices * 2);

				uint32_t compressedSize;
				bx::read(_reader, compressedSize, &err);

				void* compressedIndices = bx::alloc(s_allocator, compressedSize);

				bx::read(_reader, compressedIndices, compressedSize, &err);

				meshopt_decodeIndexBuffer(mem->data, group.m_numIndices, 2, (uint8_t*)compressedIndices, compressedSize);

				bx::free(s_allocator, compressedIndices);

				if (_ramcopy)
				{
					group.m_indices = (uint16_t*)bx::alloc(s_allocator, group.m_numIndices * 2);
					bx::memCopy(group.m_indices, mem->data, mem->size);
				}

				group.m_ibh = bgfx::createIndexBuffer(mem);
			}
			break;

			case kChunkPrimitive:
			{
				uint16_t len;
				read(_reader, len, &err);

				tinystl::string material;
				material.resize(len);
				read(_reader, const_cast<char*>(material.c_str()), len, &err);

				if (len > 0)
				{
					group.m_material = material;

					TextureMap::iterator it = m_textureMap.find(group.m_material);
					if (it == m_textureMap.end())
					{
						tinystl::string texturePath;
						texturePath.append("assets/");
						texturePath.append(group.m_material.c_str());
						texturePath.append(".dds");
						bgfx::TextureHandle texture = loadTexture(texturePath.c_str());

						m_textureMap.insert(tinystl::make_pair(group.m_material, texture));
					}
				}

				uint16_t num;
				read(_reader, num, &err);

				for (uint32_t ii = 0; ii < num; ++ii)
				{
					read(_reader, len, &err);

					tinystl::string name;
					name.resize(len);
					read(_reader, const_cast<char*>(name.c_str()), len, &err);

					Primitive prim;
					read(_reader, prim.m_startIndex, &err);
					read(_reader, prim.m_numIndices, &err);
					read(_reader, prim.m_startVertex, &err);
					read(_reader, prim.m_numVertices, &err);
					read(_reader, prim.m_sphere, &err);
					read(_reader, prim.m_aabb, &err);
					read(_reader, prim.m_obb, &err);

					group.m_prims.push_back(prim);
				}

				m_groups.push_back(group);
				group.reset();
			}
			break;

			case kChunkJointBuffer:
			{
				read(_reader, m_skinSkeleton.m_numJoint, &err);

				m_skinSkeleton.m_inverseBindMatrix = (float*)bx::alloc(s_allocator, m_skinSkeleton.m_numJoint * 16 * sizeof(float));
				bx::read(_reader, m_skinSkeleton.m_inverseBindMatrix, m_skinSkeleton.m_numJoint * 16 * sizeof(float), &err);

				//Joint debug[59];
				m_skinSkeleton.m_joints = (Joint*)bx::alloc(s_allocator, m_skinSkeleton.m_numJoint * sizeof(Joint));
				bx::read(_reader, m_skinSkeleton.m_joints, m_skinSkeleton.m_numJoint * sizeof(Joint), &err);

				m_skinSkeleton.m_inverseBindMtxUh = bgfx::createUniform("u_inverseBindMatrix", bgfx::UniformType::Mat4,
					m_skinSkeleton.m_numJoint);

				m_skinSkeleton.m_globalTransformMtxUh = bgfx::createUniform("u_globalTransformMatrix", bgfx::UniformType::Mat4,
					m_skinSkeleton.m_numJoint);

				m_skinSkeleton.m_globalTransformMatrix = (float*)bx::alloc(s_allocator, m_skinSkeleton.m_numJoint * 16 * sizeof(float));
			}
			break;

			case kChunkAnimationBuffer:
			{
				// KeyFrames
				read(_reader, m_animation.m_numKeyframe, &err);

				bx::read(_reader, &m_animation.m_duration, sizeof(float), &err);

				//float keys[1060];
				m_animation.m_keyframes = (float*)bx::alloc(s_allocator, m_animation.m_numKeyframe * sizeof(float));
				bx::read(_reader, m_animation.m_keyframes, m_animation.m_numKeyframe * sizeof(float), &err);
				//bx::read(_reader, keys, m_animation.m_numKeyframe * sizeof(float), &err);

				// Values
				read(_reader, m_animation.m_numValue, &err);

				//float values[3824];
				m_animation.m_values = (float*)bx::alloc(s_allocator, m_animation.m_numValue * sizeof(float));
				bx::read(_reader, m_animation.m_values, m_animation.m_numValue * sizeof(float), &err);
				//bx::read(_reader, values, m_animation.m_numValue * sizeof(float), &err);

				// Channels
				read(_reader, m_animation.m_numChannel, &err);

				m_animation.m_channels = (Channel*)bx::alloc(s_allocator, m_animation.m_numChannel * sizeof(Channel));
				bx::read(_reader, m_animation.m_channels, m_animation.m_numChannel * sizeof(Channel), &err);
			}
			break;

			default:
				DBG("%08x at %d", chunk, bx::skip(_reader, 0));
				break;
		}
	}
}

void Mesh::unload()
{
	for (GroupArray::const_iterator it = m_groups.begin(),itEnd = m_groups.end(); it != itEnd; ++it)
	{
		const Group& group = *it;
		bgfx::destroy(group.m_vbh);

		if (bgfx::isValid(group.m_ibh))
		{
			bgfx::destroy(group.m_ibh);
		}

		if (NULL != group.m_vertices)
		{
			bx::free(s_allocator, group.m_vertices);
		}

		if (NULL != group.m_indices)
		{
			bx::free(s_allocator, group.m_indices);
		}
	}
	m_groups.clear();

	for (TextureMap::iterator it = m_textureMap.begin(), itEnd = m_textureMap.end(); it != itEnd; ++it)
	{
		bgfx::TextureHandle txh = it->second;
		if (txh.idx != bgfx::kInvalidHandle)
		{
			bgfx::destroy(txh);
		}
	}
	m_textureMap.clear();

	if (bgfx::isValid(m_sampler))
	{
		bgfx::destroy(m_sampler);
	}

	m_animation.destroy();

	m_skinSkeleton.destroy();
}

void Mesh::update(float _dt)
{
	m_animator.update(_dt);
	m_skinSkeleton.update();
}

void Mesh::submit(bgfx::ViewId _id, bgfx::ProgramHandle _pg, const float* _mtx, uint64_t _state)
{
	if (BGFX_STATE_MASK == _state)
	{
		_state = 0
			| BGFX_STATE_WRITE_RGB
			| BGFX_STATE_WRITE_A
			| BGFX_STATE_WRITE_Z
			| BGFX_STATE_DEPTH_TEST_LESS
			| BGFX_STATE_CULL_CCW
			| BGFX_STATE_MSAA
			;
	}

	bgfx::setTransform(_mtx);
	bgfx::setState(_state);

	int idx = 0;
	for (GroupArray::const_iterator it = m_groups.begin(), itEnd = m_groups.end();
		it != itEnd; ++it)
	{
		const Group& group = *it;
		bgfx::setIndexBuffer(group.m_ibh);
		bgfx::setVertexBuffer(0, group.m_vbh);

		if(m_meshTextures.size() > 0)
		{
			bgfx::TextureHandle txh = *m_meshTextures[idx];
			bgfx::setTexture(0, m_sampler, txh);
		}
		else
		{
			TextureMap::const_iterator it = m_textureMap.find(group.m_material);
			if (it != m_textureMap.end())
			{
				bgfx::TextureHandle txh = it->second;
				bgfx::setTexture(0, m_sampler, txh);
			}
		}

		bgfx::submit(_id, _pg, 0, BGFX_DISCARD_INDEX_BUFFER | BGFX_DISCARD_VERTEX_STREAMS);
		idx++;
	}

	bgfx::discard();
}

void Mesh::printVertexLayout()
{
	for (uint32_t ii = 0; ii < bgfx::Attrib::Count; ++ii)
	{
		bgfx::Attrib::Enum attr = (bgfx::Attrib::Enum)ii;

		if (m_layout.has(attr))
		{
			uint16_t offset = m_layout.getOffset(attr);

			uint8_t num;
			bgfx::AttribType::Enum type;
			bool normalized;
			bool asInt;
			m_layout.decode(attr, num, type, normalized, asInt);

			DBG("attr:%d offset:%d, num:%d, type:%d, normalized:%d",
				ii, offset, num, type, normalized);
		}
	}
}

Mesh* loadMesh(const bx::FilePath& _filePath, bool _ramcopy)
{
	bx::FileReaderI* reader = BX_NEW(s_allocator, FileReader);
	if (bx::open(reader, _filePath))
	{
		Mesh* mesh = new Mesh;
		mesh->load(reader, _ramcopy);
		bx::close(reader);
		return mesh;
	}

	return NULL;
}


void unloadMesh(Mesh* _mesh)
{
	_mesh->unload();
	delete _mesh;
}

void* load(bx::FileReaderI* _reader, bx::AllocatorI* _allocator,
	const bx::FilePath& _filePath, uint32_t* _size)
{
	if (bx::open(_reader, _filePath))
	{
		uint32_t size = (uint32_t)bx::getSize(_reader);
		void* data = bx::alloc(_allocator, size);
		bx::read(_reader, data, size, bx::ErrorAssert{});
		bx::close(_reader);
		if (NULL != _size)
		{
			*_size = size;
		}
		return data;
	}
	else
	{
		DBG("Failed to open: %s.", _filePath);
	}

	if (NULL != _size)
	{
		*_size = 0;
	}

	return NULL;
}

void unload(void* _ptr)
{
	bx::free(s_allocator, _ptr);
}

static void imageReleaseCb(void* _ptr, void* _userData)
{
	BX_UNUSED(_ptr);
	bimg::ImageContainer* imageContainer = (bimg::ImageContainer*)_userData;
	bimg::imageFree(imageContainer);
}

bgfx::TextureHandle loadTexture(bx::FileReaderI* _reader, const bx::FilePath& _filePath,
	uint64_t _flags, uint8_t _skip, bgfx::TextureInfo* _info,
	bimg::Orientation::Enum* _orientation)
{
	BX_UNUSED(_skip);
	bgfx::TextureHandle handle = BGFX_INVALID_HANDLE;

	uint32_t size;
	void* data = load(_reader, s_allocator, _filePath, &size);
	if (NULL != data)
	{
		bimg::ImageContainer* imageContainer = bimg::imageParse(s_allocator, data, size);

		if (NULL != imageContainer)
		{
			if (NULL != _orientation)
			{
				*_orientation = imageContainer->m_orientation;
			}

			const bgfx::Memory* mem = bgfx::makeRef(
				imageContainer->m_data
				, imageContainer->m_size
				, imageReleaseCb
				, imageContainer
			);
			unload(data);

			if (imageContainer->m_cubeMap)
			{
				handle = bgfx::createTextureCube(
					uint16_t(imageContainer->m_width)
					, 1 < imageContainer->m_numMips
					, imageContainer->m_numLayers
					, bgfx::TextureFormat::Enum(imageContainer->m_format)
					, _flags
					, mem
				);
			}
			else if (1 < imageContainer->m_depth)
			{
				handle = bgfx::createTexture3D(
					uint16_t(imageContainer->m_width)
					, uint16_t(imageContainer->m_height)
					, uint16_t(imageContainer->m_depth)
					, 1 < imageContainer->m_numMips
					, bgfx::TextureFormat::Enum(imageContainer->m_format)
					, _flags
					, mem
				);
			}
			else if (bgfx::isTextureValid(0, false, imageContainer->m_numLayers, bgfx::TextureFormat::Enum(imageContainer->m_format), _flags))
			{
				handle = bgfx::createTexture2D(
					uint16_t(imageContainer->m_width)
					, uint16_t(imageContainer->m_height)
					, 1 < imageContainer->m_numMips
					, imageContainer->m_numLayers
					, bgfx::TextureFormat::Enum(imageContainer->m_format)
					, _flags
					, mem
				);
			}

			if (bgfx::isValid(handle))
			{
				const bx::StringView name(_filePath);
				bgfx::setName(handle, name.getPtr(), name.getLength());
			}

			if (NULL != _info)
			{
				bgfx::calcTextureSize(
					*_info
					, uint16_t(imageContainer->m_width)
					, uint16_t(imageContainer->m_height)
					, uint16_t(imageContainer->m_depth)
					, imageContainer->m_cubeMap
					, 1 < imageContainer->m_numMips
					, imageContainer->m_numLayers
					, bgfx::TextureFormat::Enum(imageContainer->m_format)
				);
			}
		}
	}

	return handle;
}

bgfx::TextureHandle loadTexture(const bx::FilePath& _filePath,
	uint64_t _flags, uint8_t _skip, bgfx::TextureInfo* _info,
	bimg::Orientation::Enum* _orientation)
{
	bx::FileReaderI* reader = BX_NEW(s_allocator, FileReader);
	return loadTexture(reader, _filePath, _flags, _skip, _info, _orientation);
}
