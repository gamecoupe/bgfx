#include "DXSample.h"

#include "bgfx/bgfx.h"
#include "bgfx/platform.h"

#include "bx/timer.h"

#include "inputWindows.h"

#include "DebugDraw.h"

uint32_t InputTypeManager::m_nextInputTypeID = 0;

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

	// load shader
	m_staticMeshPgh = loadShaderProgram("simple/build_d3d/vs_charactor", "simple/build_d3d/fs_charactor");
	m_AnimatedMeshZupPgh = loadShaderProgram("charactor/build_d3d/vs_charactor_zup", "charactor/build_d3d/fs_charactor");
	m_AnimatedMeshPgh = loadShaderProgram("charactor/build_d3d/vs_charactor", "charactor/build_d3d/fs_charactor");

	// load mesh
	m_levelMesh = trybgfx::loadGltf("assets/meshes/sonic_level01/scene.gltf");
	m_charactorMesh = trybgfx::loadGltf("assets/meshes/sonic/scene.gltf");
	//m_charactorMesh = trybgfx::loadGltf("assets/meshes/tom/scene.gltf");
	//m_testMesh = trybgfx::loadGltf("assets/meshes/orc/section 9 rendering b.gltf");
	//m_testMesh = trybgfx::loadGltf("assets/meshes/fox/fox.gltf");
	//m_testMesh = trybgfx::loadGltf("assets/meshes/fighting_girl/scene.gltf");

	m_timeOffset = bx::getHPCounter();
	m_timeTotal = m_timeOffset;

	// init animation
	m_animator = new trybgfx::TAnimator(m_charactorMesh);
	//m_animator->play(0);
	//m_animator->play(15);
	//m_animator->play(8);
	m_nextPaly = 8;

	m_camera = new trybgfx::Camera(float(m_width) / float(m_height));

	bx::mtxTranslate(m_cameraTransform, m_eye.x, m_eye.y, m_eye.z);
	bx::mtxInverse(m_cameraTransformInv, m_cameraTransform);

	trybgfx::ddInit();

	// input
	m_input.addDevice<WindowsInputDevice>("windows");

	m_input.addCollector("game");

	m_input.getCollector("game")->addButtonToButtonWork("forward", "windows", "W");
	m_input.getCollector("game")->addButtonToButtonWork("back", "windows", "S");
	m_input.getCollector("game")->addButtonToButtonWork("turn_left", "windows", "A");
	m_input.getCollector("game")->addButtonToButtonWork("turn_right", "windows", "D");

	m_input.getCollector("game")->addButtonToButtonWork("turbo", "windows", "Space");
}

void DXSample::UpdateCamera()
{
	float tmpMtx[16];
	bx::mtxMul(tmpMtx, m_cameraTransformInv, m_charactorTransform);

	float tmpMtx2[16];
	bx::mtxMul(tmpMtx2, tmpMtx, m_cameraTransform);

	float tmp[4] = { 0.0f, 100.0f, -100.0f, 1.0f };
	//float tmp[4] = { 100.0f, 100.0f, 0.0f, 1.0f };
	float res[4];
	bx::vec4MulMtx(res, tmp, m_charactorTransform);

	//m_init = true;
	m_eye.x = res[0];
	m_eye.y = res[1];
	m_eye.z = res[2];

	m_at.x = m_charactorPosition.x;
	m_at.y = m_charactorPosition.y + 2.f;
	m_at.z = m_charactorPosition.z;

	{
		m_camera->setPosition(m_eye);
		m_camera->setFocalPoint(m_at);

		m_camera->getViewMtx(m_view);
		m_camera->getProjMtx(m_proj);
		bgfx::setViewTransform(0, m_view, m_proj);
	}
}

// Update frame-based values.
void DXSample::OnUpdate()
{
	// This dummy draw call is here to make sure that view 0 is cleared
	// if no other draw calls are submitted to view 0.
	bgfx::touch(0);

	float time = (float)((bx::getHPCounter() - m_timeOffset) / double(bx::getHPFrequency()));
	m_timeOffset = bx::getHPCounter();

	// model matrix
	float mtx[16];
	//bx::mtxRotateXY(mtx, 0.0f, time * 0.5f * bx::kPi);
	float time2 = (float)((bx::getHPCounter() - m_timeTotal) / double(bx::getHPFrequency()));
	//bx::mtxRotateXY(mtx, 0.0f, 0.0f);
	//bx::mtxRotateXY(mtx, 0.0f, time2 * 0.5f * bx::kPi);
	bx::mtxRotateXYZ(mtx, m_charactorRotation.x, m_charactorRotation.y, m_charactorRotation.z);
	//bx::mtxRotateXY(mtx, 0.5f * bx::kPi, time2 * 0.5f * bx::kPi);
	//bx::mtxRotateXY(mtx, 0.5f * bx::kPi, time2 * 0.5f * bx::kPi);
	//bx::mtxRotateXY(mtx, 0.5f * bx::kPi, time2 * 0.5f * bx::kPi);

	// model matrix
	float mtx2[16];
	//bx::mtxRotateXY(mtx, 0.0f, time * 0.5f * bx::kPi);
	//bx::mtxScale(mtx2, 0.1f);
	//bx::mtxScale(mtx2, 1.0f);
	bx::mtxScale(mtx2, 0.05f);

	float mtx3[16];
	bx::mtxTranslate(mtx3, m_charactorPosition.x, m_charactorPosition.y, m_charactorPosition.z);
	//bx::mtxTranslate(mtx3, 0.0f, 0.0f, 0.0f);

	float tmp[16];
	//bx::mtxMul(tmp, mtx, mtx2);
	bx::mtxMul(tmp, mtx2, mtx);
	//float finalMtx[16];
	//bx::mtxMul(finalMtx, mtx3, tmp);
	bx::mtxMul(m_charactorTransform, tmp, mtx3);

	trybgfx::DebugDrawEncoder dde;
	dde.begin(0);

	UpdateCamera();

	m_animator->update(time);
	//m_animator->update(0.0f);

	//m_charactorMesh->submit(0, m_AnimatedMeshPgh, finalMtx, BGFX_STATE_MASK, &dde);
	m_charactorMesh->submit(0, m_AnimatedMeshZupPgh, m_charactorTransform, BGFX_STATE_MASK);

	float finalMtx2[16];
	bx::mtxRotateXY(mtx, 0.0f, 0.0f);
	bx::mtxScale(mtx2, 0.1f);
	bx::mtxMul(finalMtx2, mtx2, mtx);
	m_levelMesh->submit(0, m_staticMeshPgh, finalMtx2, BGFX_STATE_MASK);

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

	m_input.update();

	m_animator->play(m_nextPaly);
}


void DXSample::OnDestroy()
{
	{
		m_charactorMesh->unload();
		delete m_charactorMesh;

		m_levelMesh->unload();
		delete m_levelMesh;

		m_animator->destroy();
		delete m_animator;

		delete m_camera;
	}

	bgfx::destroy(m_AnimatedMeshPgh);
	bgfx::destroy(m_staticMeshPgh);

	trybgfx::ddShutdown();

	bgfx::shutdown();
}

void DXSample::OnKeyDown(uint8_t key)
{
	if (key == 'W')
	{
		m_nextPaly = 15;
		//m_eye.z = m_eye.z + 1.0f;

		if (!m_init)
		{
			m_init = true;
			m_charactorPosition.z -= 0.6f * bx::cos(-m_charactorRotation.y);
			m_charactorPosition.x -= 0.6f * bx::sin(-m_charactorRotation.y);
			return;
		}

		m_charactorPosition.z += 0.1f * bx::cos(-m_charactorRotation.y);
		m_charactorPosition.x += 0.1f * bx::sin(-m_charactorRotation.y);
		//m_charactorPosition.z += 1.0f;

	}

	if (key == 'S')
	{
		m_nextPaly = 15;
		//m_eye.z = m_eye.z - 1.0f;
		m_charactorPosition.z -= 0.1f * bx::cos(-m_charactorRotation.y);
		m_charactorPosition.x -= 0.1f * bx::sin(-m_charactorRotation.y);
	}
	if (key == 'A')
	{
		m_nextPaly = 8;
		//m_eye.x = m_eye.x - 1.0f;
		m_charactorRotation.y += 0.03f;
	}
	if (key == 'D')
	{
		m_nextPaly = 8;
		//m_eye.x = m_eye.x + 1.0f;
		m_charactorRotation.y -= 0.03f;
	}

	if (key == VK_UP)
	{
		m_eye.y = m_eye.y + 1.0f;
	}
	else if (key == VK_DOWN)
	{
		m_eye.y = m_eye.y - 1.0f;
	}

	//m_at = m_eye;
	//m_at.z += 1.0f;
}

void DXSample::OnKeyUp(uint8_t key)
{
	if (key == 'L')
	{
		m_animator->stop();
	}

	if (key == 'P')
	{
		m_animator->playPre();
	}
	else if (key == 'N')
	{
		m_animator->playNext();
	}

	if (key == 'W' || key == 'S' || key == 'A' || key == 'D')
	{
		m_init = false;
		m_nextPaly = 8;
	}
}

void DXSample::HandleInput()
{
	if (m_input.isButtonWorkHold("turn_right"))
	{
		m_nextPaly = 15;
		//m_eye.x = m_eye.x + 1.0f;
		m_charactorRotation.y -= 0.03f;
	}

	if (m_input.isButtonWorkHold("turn_left"))
	{
		m_nextPaly = 15;
		//m_eye.x = m_eye.x - 1.0f;
		m_charactorRotation.y += 0.03f;
	}

	if (m_input.isButtonWorkHold("forward"))
	{
		float speed = 0.03f;
		if (m_input.isButtonWorkHold("turbo"))
		{
			m_nextPaly = 2;
			speed = 0.15f;
		}
		else
		{
			m_nextPaly = 15;
			speed = 0.03f;
		}

		m_charactorPosition.z += speed * bx::cos(-m_charactorRotation.y);
		m_charactorPosition.x += speed * bx::sin(-m_charactorRotation.y);
	}


	if (m_input.isButtonWorkFree("forward") &&
		m_input.isButtonWorkFree("turn_left") &&
		m_input.isButtonWorkFree("turn_right") &&
		m_input.isButtonWorkFree("back"))
	{
		m_nextPaly = 8;
	}

}
