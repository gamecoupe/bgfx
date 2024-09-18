#include "DXSample.h"

#include "bgfx/bgfx.h"
#include "bgfx/platform.h"

#include "bx/timer.h"

#include "DebugDraw.h"

DXSample::DXSample(uint32_t width, uint32_t height) :
    m_width(width),
    m_height(height),
	m_hwnd(NULL)
{
}

DXSample::~DXSample()
{
}


void DXSample::OnInit(HWND hwnd)
{
	m_hwnd = hwnd;

	/*
	* When bgfx is compiled with option BGFX_CONFIG_MULTITHREADED=1 (default is on) bgfx::renderFrame can be called by user.
	* It’s required to be called before bgfx::init from thread that will be used as render thread.
	*
	* If both bgfx::renderFrame and bgfx::init are called from the same thread,
	* bgfx will switch to execute in single threaded mode, and calling bgfx::renderFrame is not required,
	* since it will be called automatically during bgfx::frame call.
	*/
	bgfx::renderFrame();

	bgfx::Init bgfxInit;
	bgfxInit.type = bgfx::RendererType::Direct3D12;
	bgfxInit.resolution.width = m_width;
	bgfxInit.resolution.height = m_height;
	bgfxInit.resolution.reset = BGFX_RESET_VSYNC;
	bgfxInit.platformData.nwh = m_hwnd;
	bgfx::init(bgfxInit);

	bgfx::setDebug(BGFX_DEBUG_TEXT);

	bgfx::setViewClear(0, BGFX_CLEAR_COLOR | BGFX_CLEAR_DEPTH, 0x443355FF, 1.0f, 0);
	bgfx::setViewRect(0, 0, 0, m_width, m_height);

	//m_pgh = loadShaderProgram("simple/build_d3d/vs_charactor", "simple/build_d3d/fs_charactor");
	m_pgh = loadShaderProgram("charactor/build_d3d/vs_charactor", "charactor/build_d3d/fs_charactor");

	m_testMesh = trybgfx::loadGltf("assets/meshes/tom/scene.gltf");
	//m_testMesh = trybgfx::loadGltf("assets/meshes/orc/section 9 rendering b.gltf");
	//m_testMesh = trybgfx::loadGltf("assets/meshes/fox/fox.gltf");
	//m_testMesh = trybgfx::loadGltf("assets/meshes/fighting_girl/scene.gltf");

	m_testMesh->m_sampler = bgfx::createUniform("s_texColor", bgfx::UniformType::Sampler);

	m_animator = new trybgfx::TAnimator(m_testMesh);
	//m_animator->play(0);

	m_timeOffset = bx::getHPCounter();
	m_timeTotal = m_timeOffset;


	trybgfx::ddInit();
}

// Update frame-based values.
void DXSample::OnUpdate()
{
	// This dummy draw call is here to make sure that view 0 is cleared
	// if no other draw calls are submitted to view 0.
	bgfx::touch(0);

	float time = (float)((bx::getHPCounter() - m_timeOffset) / double(bx::getHPFrequency()));
	m_timeOffset = bx::getHPCounter();

	{
		bx::mtxLookAt(m_view, m_eye, m_at);
		bx::mtxProj(m_proj, 60.0f, float(m_width) / float(m_height),
			0.1f, 100.0f, bgfx::getCaps()->homogeneousDepth);
		bgfx::setViewTransform(0, m_view, m_proj);
	}

	// model matrix
	float mtx[16];
	//bx::mtxRotateXY(mtx, 0.0f, time * 0.5f * bx::kPi);
	float time2 = (float)((bx::getHPCounter() - m_timeTotal) / double(bx::getHPFrequency()));
	bx::mtxRotateXY(mtx, 0.0f, time2 * 0.5f * bx::kPi);
	//bx::mtxRotateXY(mtx, -0.5f * bx::kPi, time2 * 0.5f * bx::kPi);
	//bx::mtxRotateXY(mtx, 0.5f * bx::kPi, time2 * 0.5f * bx::kPi);

	// model matrix
	float mtx2[16];
	//bx::mtxRotateXY(mtx, 0.0f, time * 0.5f * bx::kPi);
	bx::mtxScale(mtx2, 0.1f);
	//bx::mtxScale(mtx2, 1.0f);

	float finalMtx[16];
	bx::mtxMul(finalMtx, mtx2, mtx);

	trybgfx::DebugDrawEncoder dde;

	dde.begin(0);
	m_animator->update(time);
	//m_animator->update(0.0f);
	m_testMesh->submit(0, m_pgh, finalMtx, BGFX_STATE_MASK, &dde);
	//m_testMesh->submit(0, m_pgh, finalMtx, BGFX_STATE_MASK);


	dde.drawAxis(0.0f, 0.0f, 0.0f);


	//dde.push();
	//	bx::Aabb aabb =
	//	{
	//		{  5.0f, 1.0f, 1.0f },
	//		{ 10.0f, 5.0f, 5.0f },
	//	};
	//	dde.setWireframe(true);
	//	//dde.setColor(intersect(&dde, ray, aabb) ? kSelected : 0xff00ff00);
	//	dde.draw(aabb);
	//dde.pop();

	dde.end();


	// Advance to next frame. Rendering thread will be kicked to
	// process submitted rendering primitives.
	bgfx::frame();
}


void DXSample::OnDestroy()
{
	{
		m_testMesh->unload();
		delete m_testMesh;
	}

	bgfx::destroy(m_pgh);

	trybgfx::ddShutdown();

	bgfx::shutdown();
}

void DXSample::OnKeyDown(uint8_t key)
{
	if (key == 'W')
	{
		m_eye.z = m_eye.z + 1.0f;
	}
	else if (key == 'S')
	{
		m_eye.z = m_eye.z - 1.0f;
	}
	else if (key == 'A')
	{
		m_eye.x = m_eye.x - 1.0f;
	}
	else if (key == 'D')
	{
		m_eye.x = m_eye.x + 1.0f;
	}
	else if (key == VK_UP)
	{
		m_eye.y = m_eye.y + 1.0f;
	}
	else if (key == VK_DOWN)
	{
		m_eye.y = m_eye.y - 1.0f;
	}

	m_at = m_eye;
	m_at.z += 1.0f;
}

void DXSample::OnKeyUp(uint8_t key)
{
	if (key == 'L')
	{
		m_animator->stop();
	}

	if (key == '1')
	{
		m_animator->play(0);
	}
	else if (key == '2')
	{
		m_animator->play(1);
	}
	else if (key == '3')
	{
		m_animator->play(2);
	}
}
