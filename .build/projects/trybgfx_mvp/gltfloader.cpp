#include "gltfloader.h"

#include "bx/debug.h"
#include "bx/file.h"
#include "bx/string.h"
#include "bx/bounds.h"

#include "utilties.h"

#define CGLTF_VALIDATE_ENABLE_ASSERTS BX_CONFIG_DEBUG
#define CGLTF_IMPLEMENTATION
#include <cgltf/cgltf.h>

#include <math.h>


namespace trybgfx
{
	typedef tinystl::vector<bx::Vec3> Vec3Array;

	struct Vec2
	{
		float x, y;
	};
	typedef tinystl::vector<Vec2> Vec2Array;

	struct IVec4
	{
		uint8_t x;
		uint8_t y;
		uint8_t z;
		uint8_t w;
	};
	typedef tinystl::vector<IVec4> IVec4Array;

	struct Vec4
	{
		float x;
		float y;
		float z;
		float w;
	};
	typedef tinystl::vector<Vec4> Vec4Array;

	typedef tinystl::unordered_map<uint32_t, uint32_t> NodeJointIdxMap;

	static float imat[16] = {
		1.0f, 0.0f, 0.0f, 0.0f,
		0.0f, 1.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 1.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f,
	};

	static void gltfReadFloat(const float* _accessorData, cgltf_size _accessorNumComponents, cgltf_size _index, cgltf_float* _out, cgltf_size _outElementSize)
	{
		const float* input = &_accessorData[_accessorNumComponents * _index];

		for (cgltf_size ii = 0; ii < _outElementSize; ++ii)
		{
			_out[ii] = (ii < _accessorNumComponents) ? input[ii] : 0.0f;
		}
	}

	static void buildSkeleton(cgltf_node* _joint, uint8_t _parentIdx, cgltf_data* _data,
			NodeJointIdxMap& _nodeJointIdxMap, TJointArray& _jointArray)
	{
		uint32_t nodeIdx = (int32_t)cgltf_node_index(_data, _joint);

		NodeJointIdxMap::iterator it = _nodeJointIdxMap.find(nodeIdx);
		BX_ASSERT(it != _nodeJointIdxMap.end(), "Invalid pathType");
		uint32_t idx = it->second;
		_jointArray[idx].m_parentIdx = _parentIdx;

		for (cgltf_size childIndex = 0; childIndex < _joint->children_count; ++childIndex)
		{
			buildSkeleton(_joint->children[childIndex], (uint8_t)idx, _data, _nodeJointIdxMap, _jointArray);
		}
	}

	static void processGltfNode(cgltf_node* _node, TMesh* _mesh, TGroup* _group,
		cgltf_data* _data, NodeJointIdxMap* _nodeJointIdxMap)
	{
		cgltf_mesh* mesh = _node->mesh;
		if (NULL != mesh)
		{
			Vec3Array positions;
			Vec2Array texcoords;
			IVec4Array jointIndices;
			Vec4Array weights;

			for (cgltf_size primitiveIndex = 0;
				primitiveIndex < mesh->primitives_count; ++primitiveIndex)
			{
				cgltf_primitive* primitive = &mesh->primitives[primitiveIndex];
				cgltf_size numVertex = primitive->attributes[0].data->count;

				for (cgltf_size attributeIndex = 0;
					attributeIndex < primitive->attributes_count; ++attributeIndex)
				{
					cgltf_attribute* attribute = &primitive->attributes[attributeIndex];
					cgltf_accessor* accessor = attribute->data;
					cgltf_size accessorCount = accessor->count;

					BX_ASSERT(numVertex == accessorCount, "Invalid attribute count");

					cgltf_size numComponents = cgltf_num_components(accessor->type);

					switch (accessor->component_type)
					{
						case cgltf_component_type_r_32f:
						{
							// POSITION, NORMAL, TEXCOORD, WEIGHT, ....
							cgltf_size floatCount = cgltf_accessor_unpack_floats(accessor, NULL, 0);
							float* accessorData = (float*)malloc(floatCount * sizeof(float));
							cgltf_accessor_unpack_floats(accessor, accessorData, floatCount);

							cgltf_size numComponents = cgltf_num_components(accessor->type);
							if (attribute->type == cgltf_attribute_type_position && attribute->index == 0)
							{
								positions.reserve(accessorCount);

								bx::Vec3 pos(bx::InitNone);
								for (cgltf_size v = 0; v < accessorCount; ++v)
								{
									gltfReadFloat(accessorData, numComponents, v, &pos.x, 3);
									positions.push_back(pos);
								}
							}
							else if (attribute->type == cgltf_attribute_type_texcoord && attribute->index == 0)
							{
								texcoords.reserve(accessorCount);

								Vec2 texcoord;
								for (cgltf_size v = 0; v < accessorCount; ++v)
								{
									gltfReadFloat(accessorData, numComponents, v, &texcoord.x, 2);
									texcoords.push_back(texcoord);
								}
							}
							else if (attribute->type == cgltf_attribute_type_weights && attribute->index == 0)
							{
								weights.reserve(accessorCount);

								Vec4 weight;
								for (cgltf_size v = 0; v < accessorCount; ++v)
								{
									gltfReadFloat(accessorData, numComponents, v, &weight.x, 4);
									weights.push_back(weight);
								}
							}

							free(accessorData);
						}
						break;

						case cgltf_component_type_r_16u:
						case cgltf_component_type_r_8u:
						{
							// JOINT, ...
							cgltf_size floatCount = cgltf_accessor_unpack_floats(accessor, NULL, 0);
							float* accessorData = (float*)malloc(floatCount * sizeof(float));
							cgltf_accessor_unpack_floats(accessor, accessorData, floatCount);

							cgltf_size numComponents = cgltf_num_components(accessor->type);

							if (attribute->type == cgltf_attribute_type_joints && attribute->index == 0)
							{
								for (size_t i = 0; i < accessorCount; i++)
								{
									float tmp[4];
									gltfReadFloat(accessorData, numComponents, i, tmp, 4);

									IVec4 joint;
									joint.x = (uint8_t)tmp[0];
									joint.y = (uint8_t)tmp[1];
									joint.z = (uint8_t)tmp[2];
									joint.w = (uint8_t)tmp[3];
									jointIndices.push_back(joint);
								}
							}
						}
						break;

						default:
							break;
					}
				}

				if (primitive->indices != NULL)
				{
					cgltf_accessor* accessor = primitive->indices;

					_group->m_indices = (uint16_t*)malloc(accessor->count * sizeof(uint16_t));
					_group->m_numIndices = accessor->count;

					for (cgltf_size v = 0; v < accessor->count; v += 3)
					{
						for (int i = 0; i < 3; ++i)
						{
							uint16_t vertexIndex = uint16_t(cgltf_accessor_read_index(accessor, v + i));
							_group->m_indices[v + i] = vertexIndex;
						}
					}
				}
				else
				{
					_group->m_indices = (uint16_t*)malloc(numVertex * sizeof(uint16_t));
					_group->m_numIndices = numVertex;

					for (cgltf_size v = 0; v < numVertex; v += 3)
					{
						for (int i = 0; i < 3; ++i)
						{
							uint16_t vertexIndex = int32_t(v + i);
							_group->m_indices[v + i] = vertexIndex;
						}
					}
				}

				// Now only process PBR metallic material
				if (primitive->material != NULL && primitive->material->has_pbr_metallic_roughness)
				{
					cgltf_pbr_metallic_roughness& pbrMetallicRoughness =
						primitive->material->pbr_metallic_roughness;

					cgltf_texture* texture = pbrMetallicRoughness.base_color_texture.texture;
					if (texture != NULL)
					{
						_group->m_material = pbrMetallicRoughness.base_color_texture.texture->image->uri;
					}
				}

				{
					_group->m_layout
						.begin()
						.add(bgfx::Attrib::Position, 3, bgfx::AttribType::Float)
						.add(bgfx::Attrib::TexCoord0, 2, bgfx::AttribType::Float)
						.add(bgfx::Attrib::Indices, 4, bgfx::AttribType::Uint8)
						.add(bgfx::Attrib::Weight, 4, bgfx::AttribType::Float)
						.end();

					_group->m_numVertices = numVertex;

					_group->m_vertices = (Vertex*)malloc(numVertex * sizeof(Vertex));
					for (cgltf_size i = 0; i < numVertex; i++)
					{
						_group->m_vertices[i].m_x = positions[i].x;
						_group->m_vertices[i].m_y = positions[i].y;
						_group->m_vertices[i].m_z = positions[i].z;

						_group->m_vertices[i].m_u = texcoords[i].x;
						_group->m_vertices[i].m_v = texcoords[i].y;

						_group->m_vertices[i].m_idx0 = jointIndices[i].x;
						_group->m_vertices[i].m_idx1 = jointIndices[i].y;
						_group->m_vertices[i].m_idx2 = jointIndices[i].z;
						_group->m_vertices[i].m_idx3 = jointIndices[i].w;

						_group->m_vertices[i].m_w0 = weights[i].x;
						_group->m_vertices[i].m_w1 = weights[i].y;
						_group->m_vertices[i].m_w2 = weights[i].z;
						_group->m_vertices[i].m_w3 = weights[i].w;
					}

					_group->m_vbh = bgfx::createVertexBuffer(
						bgfx::makeRef(_group->m_vertices, sizeof(Vertex) * numVertex), _group->m_layout);

					_group->m_ibh = bgfx::createIndexBuffer(
						bgfx::makeRef(_group->m_indices, sizeof(uint16_t) * _group->m_numIndices));

					bx::toAabb(_group->m_aabb, _group->m_vertices, numVertex, sizeof(Vertex));

					if (_group->m_material.size() > 0)
					{
						TextureMap::iterator it = _mesh->m_textureMap.find(_group->m_material);
						if (it == _mesh->m_textureMap.end())
						{
							tinystl::string texturePath;
							texturePath.append("assets/");
							texturePath.append(_group->m_material.c_str());
							texturePath.append(".dds");
							bgfx::TextureHandle texture = loadTexture(texturePath.c_str());

							_mesh->m_textureMap.insert(tinystl::make_pair(_group->m_material, texture));
						}
					}
				}

				_mesh->m_groups.push_back(*_group);
			}
		}

		cgltf_skin* skin = _node->skin;
		if (NULL != skin)
		{
			cgltf_accessor* accessor = skin->inverse_bind_matrices;
			cgltf_size accessorCount = accessor->count;
			cgltf_size numJoint = skin->joints_count;

			BX_ASSERT(numJoint == accessorCount, "Invalid joint count");
			_mesh->m_skinSkeleton.m_numJoint = numJoint;

			cgltf_size floatCount = cgltf_accessor_unpack_floats(accessor, NULL, 0);

			_mesh->m_skinSkeleton.m_inverseBindMatrix = (float*)malloc(floatCount * sizeof(float));
			cgltf_accessor_unpack_floats(accessor, _mesh->m_skinSkeleton.m_inverseBindMatrix, floatCount);

			_mesh->m_skinSkeleton.m_globalTransformMatrix = (float*)malloc(floatCount * sizeof(float));;

			for (cgltf_size v = 0; v < numJoint; ++v)
			{
				cgltf_node* node = skin->joints[v];
				uint32_t nodeIdx = (uint32_t)cgltf_node_index(_data, node);
				_nodeJointIdxMap->insert(tinystl::make_pair(nodeIdx, (uint32_t)v));

				TJoint jointData;
				if (node->has_rotation)
				{
					jointData.has_rotate = 1;
					jointData.m_rotate[0] = node->rotation[0];
					jointData.m_rotate[1] = node->rotation[1];
					jointData.m_rotate[2] = node->rotation[2];
					jointData.m_rotate[3] = -node->rotation[3];
				}
				else
				{
					jointData.has_rotate = 0;
				}

				if (node->has_translation)
				{
					jointData.has_tranlation = 1;
					jointData.m_tranlation[0] = node->translation[0];
					jointData.m_tranlation[1] = node->translation[1];
					jointData.m_tranlation[2] = node->translation[2];
					jointData.m_tranlation[3] = 0.0f;
				}
				else
				{
					jointData.has_tranlation = 0;
				}

				if (node->has_scale)
				{
					jointData.has_scale = 1;
					jointData.m_scale[0] = node->scale[0];
					jointData.m_scale[1] = node->scale[1];
					jointData.m_scale[2] = node->scale[2];
					jointData.m_scale[3] = 0.0f;
				}
				else
				{
					jointData.has_scale = 0;
				}

				_mesh->m_skinSkeleton.m_joints.push_back(jointData);
			}

			//cgltf_node* rootJoint = skin->skeleton != NULL ? skin->skeleton : skin->joints[0];
			cgltf_node* rootJoint = skin->joints[0];
			buildSkeleton(rootJoint, UINT8_MAX, _data, *_nodeJointIdxMap, _mesh->m_skinSkeleton.m_joints);

			{
				_mesh->m_skinSkeleton.m_inverseBindMtxUh = bgfx::createUniform("u_inverseBindMatrix",
					bgfx::UniformType::Mat4, numJoint);

				_mesh->m_skinSkeleton.m_globalTransformMtxUh = bgfx::createUniform("u_globalTransformMatrix",
					bgfx::UniformType::Mat4, numJoint);
			}


		}

		for (cgltf_size childIndex = 0; childIndex < _node->children_count; ++childIndex)
		{
			processGltfNode(_node->children[childIndex], _mesh, _group, _data, _nodeJointIdxMap);
		}
	}


	void calcGlobalTransformMtx(float* _translateArray, float* _rotateArray, float* _scaleArray,
		bool* _changedArray, TJointArray& _joints, uint32_t idx, float* result)
	{
		TJoint* currentJoint = &(_joints[idx]);

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
		else if(currentJoint->has_scale)
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
				_rotateArray[idx * 4 + 3],
			};

			//bx::Quaternion normalizedQ = bx::normalize(q);
			bx::mtxFromQuaternion(rotate, q);
		}
		else if (currentJoint->has_rotate)
		{
			bx::Quaternion q = {
				currentJoint->m_rotate[0],
				currentJoint->m_rotate[1],
				currentJoint->m_rotate[2],
				currentJoint->m_rotate[3],
			};
			//bx::Quaternion normalizedQ = bx::normalize(q);
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
			float f0 = _translateArray[idx * 3];
			float f1 = _translateArray[idx * 3 + 1];
			float f2 = _translateArray[idx * 3 + 2];
			bx::mtxTranslate(trans, f0, f1, f2);
		}
		else if (currentJoint->has_tranlation)
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
	}

	// --------------------------
	TGroup::TGroup()
	{
		m_vbh.idx = bgfx::kInvalidHandle;
		m_ibh.idx = bgfx::kInvalidHandle;
		m_numVertices = 0;
		m_vertices = NULL;
		m_numIndices = 0;
		m_indices = NULL;
	}

	void TGroup::destroy()
	{
		if (bgfx::isValid(m_vbh))
		{
			bgfx::destroy(m_vbh);
		}

		if (bgfx::isValid(m_ibh))
		{
			bgfx::destroy(m_ibh);
		}

		if (m_vertices != NULL)
		{
			free(m_vertices);
		}

		if (m_indices != NULL)
		{
			free(m_indices);
		}
	}

	// --------------------------
	TSkinSkeleton::TSkinSkeleton()
	{
		m_inverseBindMtxUh.idx = bgfx::kInvalidHandle;
		m_isInverseBindMtxUhSet = false;

		m_globalTransformMtxUh.idx = bgfx::kInvalidHandle;

		m_inverseBindMatrix = NULL;
		m_globalTransformMatrix = NULL;

		m_numJoint = 0;
	}

	void TSkinSkeleton::destroy()
	{
		if (bgfx::isValid(m_inverseBindMtxUh))
		{
			bgfx::destroy(m_inverseBindMtxUh);
		}

		if (bgfx::isValid(m_globalTransformMtxUh))
		{
			bgfx::destroy(m_globalTransformMtxUh);
		}

		m_joints.clear();

		if (NULL != m_inverseBindMatrix)
		{
			free(m_inverseBindMatrix);
		}

		if (NULL != m_globalTransformMatrix)
		{
			free(m_globalTransformMatrix);
		}
	}

	void TSkinSkeleton::update()
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

	// --------------------------
	TAnimator::TAnimator(TMesh* _mesh)
		:m_mesh(_mesh), m_currentTime(0.0f), m_playing(false)
	{
		//m_isPlayingUh.idx = bgfx::kInvalidHandle;
		m_isPlayingUh = bgfx::createUniform("u_flgs", bgfx::UniformType::Vec4, 1);
	}

	void TAnimator::update(float _dt)
	{
		if (!m_playing)
		{
			stop();
			return;
		}

		step(_dt);
		m_mesh->m_skinSkeleton.update();

		float flgs[4];
		flgs[0] = 2.0f;
		bgfx::setUniform(m_isPlayingUh, flgs, 1);
	}

	void TAnimator::play(uint32_t idx)
	{
		BX_ASSERT(idx < m_mesh->m_animations.size(), "Invalid animtion index");
		m_currentAnimation = &(m_mesh->m_animations[idx]);

		m_currentTime = 0.0f;
		m_playing = true;
	}

	void TAnimator::stop()
	{
		m_playing = false;

		float flgs[4];
		flgs[0] = -2.0f;
		bgfx::setUniform(m_isPlayingUh, flgs, 1);
	}

	void TAnimator::step(float _dt)
	{
		m_currentTime += _dt;
		float animationTime = fmod(m_currentTime, m_currentAnimation->m_duration);

		if (animationTime < 0.034f)
		{
			return;
		}

		float translateArray[MAX_JOINTS * 3];
		float rotateArray[MAX_JOINTS * 4];
		float scaleArray[MAX_JOINTS * 3];
		bool changedArray[MAX_JOINTS * 3];	// 0: translate 1: scale 2: rotate

		bx::memSet(changedArray, false, sizeof(changedArray));

		for (size_t i = 0; i < m_currentAnimation->m_numChannel; i++)
		{
			const TChannel& channel = m_currentAnimation->m_channels[i];

			for (size_t j = 0; j < channel.m_numKeys - 1; j++)
			{
				float previousTime = m_currentAnimation->m_keyframes[channel.m_startKey + j];
				float nextTime = m_currentAnimation->m_keyframes[channel.m_startKey + j + 1];

				if (animationTime >= previousTime && animationTime <= nextTime)
				{
					float interpolationFactor = (animationTime - previousTime) / (nextTime - previousTime);

					uint32_t previousValueIdx = channel.m_startValue + j * channel.m_strike;
					uint32_t nextValueIdx = channel.m_startValue + (j + 1) * channel.m_strike;

					if (channel.m_type == (uint32_t)Channel::Type::Translation)
					{
						changedArray[channel.m_targetJointIdx * 3] = true;

						bx::Vec3 previouV = {
							m_currentAnimation->m_values[previousValueIdx],
							m_currentAnimation->m_values[previousValueIdx + 1],
							m_currentAnimation->m_values[previousValueIdx + 2]
						};

						bx::Vec3 nextV = {
							m_currentAnimation->m_values[nextValueIdx],
							m_currentAnimation->m_values[nextValueIdx + 1],
							m_currentAnimation->m_values[nextValueIdx + 2]
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
							m_currentAnimation->m_values[previousValueIdx],
							m_currentAnimation->m_values[previousValueIdx + 1],
							m_currentAnimation->m_values[previousValueIdx + 2]
						};

						bx::Vec3 nextV = {
							m_currentAnimation->m_values[nextValueIdx],
							m_currentAnimation->m_values[nextValueIdx + 1],
							m_currentAnimation->m_values[nextValueIdx + 2]
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
							m_currentAnimation->m_values[previousValueIdx],
							m_currentAnimation->m_values[previousValueIdx + 1],
							m_currentAnimation->m_values[previousValueIdx + 2],
							m_currentAnimation->m_values[previousValueIdx + 3]
						};

						bx::Quaternion nextQ = {
							m_currentAnimation->m_values[nextValueIdx],
							m_currentAnimation->m_values[nextValueIdx + 1],
							m_currentAnimation->m_values[nextValueIdx + 2],
							m_currentAnimation->m_values[nextValueIdx + 3]
						};

						bx::Quaternion result = bx::lerp(previouQ, nextQ, interpolationFactor);
						rotateArray[channel.m_targetJointIdx * channel.m_strike] = result.x;
						rotateArray[channel.m_targetJointIdx * channel.m_strike + 1] = result.y;
						rotateArray[channel.m_targetJointIdx * channel.m_strike + 2] = result.z;
						rotateArray[channel.m_targetJointIdx * channel.m_strike + 3] = -result.w;
					}
					else
					{
						DBG("Invalid channel.m_type");
					}

					break;
				}
			}
		}


		for (size_t i = 0; i < m_mesh->m_skinSkeleton.m_numJoint; i++)
		{
			float* globalTransformMtx = m_mesh->m_skinSkeleton.m_globalTransformMatrix + i * 16;
			calcGlobalTransformMtx(translateArray, rotateArray, scaleArray, changedArray,
				m_mesh->m_skinSkeleton.m_joints, i, globalTransformMtx);
		}
	}

	// --------------------------
	TMesh::TMesh()
	{
		m_sampler.idx = bgfx::kInvalidHandle;
	}

	void TMesh::load(const bx::FilePath& _filePath)
	{
		bx::FileReader fr;
		if (!bx::open(&fr, _filePath))
		{
			bx::printf("Unable to open input file '%s'.", _filePath);
			return;
		}

		uint32_t size = (uint32_t)bx::getSize(&fr);
		char* rawData = new char[size + 1];
		size = bx::read(&fr, rawData, size, bx::ErrorAssert{});
		rawData[size] = '\0';
		bx::close(&fr);

		TGroup group;
		NodeJointIdxMap nodeJointIdxMap;

		cgltf_options options = { };
		cgltf_data* data = NULL;
		cgltf_result result = cgltf_parse(&options, rawData, size, &data);

		delete[] rawData;

		if (result == cgltf_result_success)
		{
			char* path = (char*)malloc(_filePath.getPath().getLength() + 1);
			bx::memCopy(path, _filePath.getPath().getPtr(), _filePath.getPath().getLength());
			path[_filePath.getPath().getLength()] = 0;
			result = cgltf_load_buffers(&options, data, path);
			free(path);

			if (result == cgltf_result_success)
			{
				// mesh
				for (cgltf_size sceneIndex = 0; sceneIndex < data->scenes_count; ++sceneIndex)
				{
					cgltf_scene* scene = &data->scenes[sceneIndex];

					for (cgltf_size nodeIndex = 0; nodeIndex < scene->nodes_count; ++nodeIndex)
					{
						cgltf_node* node = scene->nodes[nodeIndex];

						processGltfNode(node, this, &group, data, &nodeJointIdxMap);
					}
				}

				// animation
				for (cgltf_size animationIndex = 0; animationIndex < data->animations_count; ++animationIndex)
				{
					TAnimation animation;

					cgltf_animation* rawAnimation = &data->animations[animationIndex];

					animation.m_numChannel = rawAnimation->channels_count;
					animation.m_duration = 0.0f;

					animation.m_channels.reserve(rawAnimation->channels_count);
					for (cgltf_size channelIdx = 0; channelIdx < rawAnimation->channels_count; ++channelIdx)
					{
						cgltf_animation_channel channelNode = rawAnimation->channels[channelIdx];

						TChannel channel;

						cgltf_node* node = channelNode.target_node;
						uint32_t nodeIdx = (int32_t)cgltf_node_index(data, node);

						NodeJointIdxMap::iterator it = nodeJointIdxMap.find(nodeIdx);
						BX_ASSERT(it != nodeJointIdxMap.end(), "Invalid pathType");
						channel.m_targetJointIdx = it->second;

						{
							// channel.input
							cgltf_accessor* accessor = channelNode.sampler->input;
							cgltf_size floatCount = cgltf_accessor_unpack_floats(accessor, NULL, 0);
							float* accessorData = (float*)malloc(floatCount * sizeof(float));
							cgltf_accessor_unpack_floats(accessor, accessorData, floatCount);

							if (accessor->has_max && accessor->max[0] > animation.m_duration)
							{
								animation.m_duration = accessor->max[0];
							}

							channel.m_startKey = (uint32_t)animation.m_keyframes.size();
							channel.m_numKeys = (uint32_t)floatCount;

							for (size_t i = 0; i < floatCount; i++)
							{
								animation.m_keyframes.push_back(accessorData[i]);
							}
							free(accessorData);
						}

						cgltf_animation_path_type pathType = channelNode.target_path;
						switch (pathType)
						{
							case cgltf_animation_path_type_translation:
							{
								channel.m_type = (uint32_t)Channel::Type::Translation;
								channel.m_strike = 3;
							}
							break;

							case cgltf_animation_path_type_rotation:
							{
								channel.m_type = (uint32_t)Channel::Type::Rotation;
								channel.m_strike = 4;
							}
							break;

							case cgltf_animation_path_type_scale:
							{
								channel.m_type = (uint32_t)Channel::Type::Scale;
								channel.m_strike = 3;
							}
							break;

							default:
								BX_ASSERT(false, "Invalid pathType");
								break;
						}

						{
							// channel.output
							cgltf_accessor* accessor = channelNode.sampler->output;
							cgltf_size floatCount = cgltf_accessor_unpack_floats(accessor, NULL, 0);
							float* accessorData = (float*)malloc(floatCount * sizeof(float));
							cgltf_accessor_unpack_floats(accessor, accessorData, floatCount);

							channel.m_startValue = (uint32_t)animation.m_values.size();
							channel.m_numValues = (uint32_t)floatCount;

							for (size_t i = 0; i < floatCount; i++)
							{
								animation.m_values.push_back(accessorData[i]);
							}
							free(accessorData);
						}

						animation.m_channels.push_back(channel);
					}

					m_animations.push_back(animation);
				}
			}

			cgltf_free(data);
		}
	}

	void TMesh::unload()
	{
		for (TGroupArray::iterator it = m_groups.begin(), itEnd = m_groups.end(); it != itEnd; ++it)
		{
			TGroup& group = *it;
			group.destroy();
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

		m_skinSkeleton.destroy();
	}

	void TMesh::submit(bgfx::ViewId _id, bgfx::ProgramHandle _pg, const float* _mtx,
		uint64_t _state, DebugDrawEncoder* _dde)
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

		for (TGroupArray::const_iterator it = m_groups.begin(), itEnd = m_groups.end();
			it != itEnd; ++it)
		{
			const TGroup& group = *it;
			bgfx::setIndexBuffer(group.m_ibh);
			bgfx::setVertexBuffer(0, group.m_vbh);

			TextureMap::const_iterator tmIt = m_textureMap.find(group.m_material);
			if (tmIt != m_textureMap.end())
			{
				bgfx::TextureHandle txh = tmIt->second;
				bgfx::setTexture(0, m_sampler, txh);
			}

			bgfx::submit(_id, _pg, 0, BGFX_DISCARD_INDEX_BUFFER | BGFX_DISCARD_VERTEX_STREAMS);

			if (_dde != NULL)
			{
				_dde->push();
				_dde->setWireframe(true);

				_dde->setTransform(_mtx);
				_dde->setColor(0xff0000ff);
				_dde->draw(group.m_aabb);
				_dde->pop();
			}
		}

		bgfx::discard();
	}

	// --------------------------
	TMesh* loadGltf(const bx::FilePath& _filePath)
	{

		TMesh* mesh = new TMesh;
		mesh->load(_filePath);

		return mesh;
	}
}
