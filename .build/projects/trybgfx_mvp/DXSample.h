#ifndef TRYBGFX_DXSAMPLE_H_HEADER_GUARD
#define TRYBGFX_DXSAMPLE_H_HEADER_GUARD

#include "bgfx/bgfx.h"
#include "bx/math.h"

#include "utilties.h"
#include "gltfloader.h"

#include <windows.h>

class DXSample
{
public:
    DXSample(uint32_t width, uint32_t height);
    virtual ~DXSample();

    virtual void OnInit(HWND hwnd);
    virtual void OnUpdate();
    virtual void OnDestroy();

    // Samples override the event handlers to handle specific messages.
	virtual void OnKeyDown(uint8_t /*key*/);
	virtual void OnKeyUp(uint8_t /*key*/);

    // Accessors.
	uint32_t GetWidth() const           { return m_width; }
	uint32_t GetHeight() const          { return m_height; }

protected:
    // Viewport dimensions.
    uint32_t m_width;
	uint32_t m_height;

	HWND m_hwnd;

	bgfx::ProgramHandle m_pgh;

	//bgfx::UniformHandle m_texColorUh;

	trybgfx::TMesh* m_testMesh;

	trybgfx::TAnimator* m_animator;


	//bx::Vec3 m_at = { 0.0f, 10.0f, 0.0f };
	//bx::Vec3 m_eye = { 0.0f, 10.0f, -40.0f };
	bx::Vec3 m_at = { 0.0f, 0.0f, 0.0f };
	bx::Vec3 m_eye = { 0.0f, 0.0f, -10.0f };
	//bx::Vec3 m_eye = { 0.0f, 0.0f, -40.0f };

	float m_view[16];
	float m_proj[16];

	int64_t m_timeOffset;
	int64_t m_timeTotal;
};

#endif // !TRYBGFX_DXSAMPLE_H_HEADER_GUARD
