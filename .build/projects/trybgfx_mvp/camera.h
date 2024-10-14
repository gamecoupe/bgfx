#ifndef TRYBGFX_DEBUGDRAW_H_CAMERA_GUARD
#define TRYBGFX_DEBUGDRAW_H_CAMERA_GUARD

#include "bx/bx.h"
#include "bx/math.h"

#include "bgfx/bgfx.h"

namespace trybgfx
{
	struct MouseButton
	{
		enum Enum
		{
			None,
			Left,
			Middle,
			Right,

			Count
		};
	};

	struct MouseState
	{
		MouseState()
			: m_mx(0)
			, m_my(0)
			, m_mz(0)
		{
			for (uint32_t ii = 0; ii < MouseButton::Count; ++ii)
			{
				m_buttons[ii] = MouseButton::None;
			}
		}

		int32_t m_mx;
		int32_t m_my;
		int32_t m_mz;
		uint8_t m_buttons[MouseButton::Count];
	};

	struct Camera
	{
		struct MouseCoords
		{
			int32_t m_mx;
			int32_t m_my;
			int32_t m_mz;
		};

		Camera(float _aspectRatio) :m_aspectRatio(_aspectRatio)
		{
			reset();
		}

		void reset()
		{
			m_mouseNow.m_mx = 0;
			m_mouseNow.m_my = 0;
			m_mouseNow.m_mz = 0;

			m_mouseLast.m_mx = 0;
			m_mouseLast.m_my = 0;
			m_mouseLast.m_mz = 0;

			m_eye.x = 0.0f;
			m_eye.y = 0.0f;
			m_eye.z = -35.0f;

			m_at.x = 0.0f;
			m_at.y = 0.0f;
			m_at.z = -1.0f;

			m_up.x = 0.0f;
			m_up.y = 1.0f;
			m_up.z = 0.0f;

			m_fov = 60.0f;
			m_nearClip = 0.1f;
			m_farClip = 1000.0f;

			m_mouseSpeed = 0.0020f;
			m_gamepadSpeed = 0.04f;
			m_moveSpeed = 30.0f;
		}

		void getViewMtx(float* _viewMtx)
		{
			bx::mtxLookAt(_viewMtx, bx::load<bx::Vec3>(&m_eye.x), bx::load<bx::Vec3>(&m_at.x), bx::load<bx::Vec3>(&m_up.x));
		}

		void getProjMtx(float* _projMtx)
		{
			bx::mtxProj(_projMtx, m_fov, m_aspectRatio, m_nearClip, m_farClip, bgfx::getCaps()->homogeneousDepth);
		}

		void setPosition(const bx::Vec3& _pos)
		{
			m_eye = _pos;
		}

		void setFocalPoint(const bx::Vec3& _at)
		{
			m_at = _at;
		}

		MouseCoords m_mouseNow;
		MouseCoords m_mouseLast;

		bx::Vec3 m_eye = bx::InitZero;
		bx::Vec3 m_at = bx::InitZero;
		bx::Vec3 m_up = bx::InitZero;

		float m_fov;
		float m_aspectRatio;
		float m_nearClip;
		float m_farClip;

		float m_mouseSpeed;
		float m_gamepadSpeed;
		float m_moveSpeed;
	};
}

#endif
