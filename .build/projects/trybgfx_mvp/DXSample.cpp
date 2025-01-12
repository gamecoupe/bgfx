#include "DXSample.h"

#include "bgfx/bgfx.h"
#include "bgfx/platform.h"

#include "bx/timer.h"

#include "inputWindows.h"

#include "DebugDraw.h"

#include "myImgui.h"

uint32_t InputTypeManager::m_nextInputTypeID = 0;

DXSample::DXSample(uint32_t width, uint32_t height) :
    m_width(width), m_height(height), m_hwnd(NULL)
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

	// Physics System
	m_physics = new trybgfx::TPhysics();
	m_physics->init();

	// load shader
	m_staticMeshPgh = loadShaderProgram("simple/build_d3d/vs_charactor", "simple/build_d3d/fs_charactor");
	m_AnimatedMeshZupPgh = loadShaderProgram("charactor/build_d3d/vs_charactor_zup", "charactor/build_d3d/fs_charactor");
	m_AnimatedMeshPgh = loadShaderProgram("charactor/build_d3d/vs_charactor", "charactor/build_d3d/fs_charactor");

	// load mesh
	//m_levelMesh = trybgfx::loadGltf("assets/meshes/sonic_level01/scene.gltf");
	m_tmplevelMesh = trybgfx::loadGltf("assets/meshes/test3/untitled.gltf");

	trybgfx::TStaticActor* actor = new trybgfx::TStaticActor(m_physics, "noname", true, true);
	//actor->load("assets/meshes/testlevel/ground/ground.gltf");
	actor->load("assets/meshes/sonic_level01/scene.gltf");
	m_levelRepository.push_back(actor);

	//actor = new trybgfx::TStaticActor(m_physics, "noname", true, true);
	//actor->load("assets/meshes/testlevel/flattop/flattop.gltf");
	//m_levelRepository.push_back(actor);

	//actor = new trybgfx::TStaticActor(m_physics, "noname", true, true);
	//actor->load("assets/meshes/testlevel/flatslice1/flatslice1.gltf");
	//m_levelRepository.push_back(actor);

	//actor = new trybgfx::TStaticActor(m_physics, "noname", true, true);
	//actor->load("assets/meshes/testlevel/flatslice2/flatslice2.gltf");
	//m_levelRepository.push_back(actor);

	//actor = new trybgfx::TStaticActor(m_physics, "noname", false, false);
	//actor->load("assets/meshes/testlevel/steps/steps.gltf");
	//m_levelRepository.push_back(actor);

	//actor = new trybgfx::TStaticActor(m_physics, "noname", true, true);
	//actor->load("assets/meshes/testlevel/stepColl/stepColl.gltf");
	//m_levelRepository.push_back(actor);

	//actor = new trybgfx::TStaticActor(m_physics, "noname", true, true);
	//actor->load("assets/meshes/testlevel/wall1/scene.gltf");
	//m_levelRepository.push_back(actor);

	//actor = new trybgfx::TStaticActor(m_physics, "noname", true, true);
	//actor->load("assets/meshes/testlevel/wall2/scene.gltf");
	//m_levelRepository.push_back(actor);

	//actor = new trybgfx::TStaticActor(m_physics, "noname", true, true);
	//actor->load("assets/meshes/testlevel/wall3/scene.gltf");
	//m_levelRepository.push_back(actor);

	//actor = new trybgfx::TStaticActor(m_physics, "noname", true, true);
	//actor->load("assets/meshes/testlevel/wall4/scene.gltf");
	//m_levelRepository.push_back(actor);

	m_charactorMesh = trybgfx::loadGltf("assets/meshes/sonic/scene.gltf");
	m_charactorMesh->m_capsuleRadius *= m_characterScale;
	//m_charactorMesh = trybgfx::loadGltf("assets/meshes/tom/scene.gltf");
	//m_testMesh = trybgfx::loadGltf("assets/meshes/orc/section 9 rendering b.gltf");
	//m_testMesh = trybgfx::loadGltf("assets/meshes/fox/fox.gltf");
	//m_testMesh = trybgfx::loadGltf("assets/meshes/fighting_girl/scene.gltf");
	bx::Sphere sphere = m_charactorMesh->getSphere(m_charactorPosition);
	m_physics->addVolumeShape(sphere);

	//m_physics->addVolumeShape(m_charactorMesh->m_aabb, m_charactorPosition, m_charactorRotation, true);

	//m_physics->addVolumeShape(m_charactorMesh->getCapsule(m_charactorPosition));

	m_physics->optimizeBroadPhase();



	m_timeOffset = bx::getHPCounter();
	m_timeTotal = m_timeOffset;

	// shader flag unform
	trybgfx::ShaderFlags& flags = trybgfx::ShaderFlags::getInstance();
	flags.init();

	// init animation
	m_animator = new trybgfx::TAnimator(m_charactorMesh);
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
	m_input.getCollector("game")->addButtonToButtonWork("jump", "windows", "Return");

	m_input.getCollector("game")->addAxisToAxisWork("mouse", "windows", "Cursor");

	// imgui
	imguiCreate();
}

// Update frame-based values.
void DXSample::OnUpdate()
{
	// update physics system
	const float deltaTime = 1.0f / 60.0f;
	bx::Vec3 tmpPos = { 0.0f,0.0f ,0.0f };
	bx::Vec3 tmpAn = { 0.0f,0.0f ,0.0f };
	m_physics->UpdatePhysics(deltaTime, tmpPos, m_charactorVelocity, tmpAn);

	//m_charactorPosition.y = tmpPos.y - 32.0f;
	m_charactorPosition.y = tmpPos.y - m_charactorMesh->m_capsuleRadius + 1.0f;
	m_charactorPosition.x = tmpPos.x;
	m_charactorPosition.z = tmpPos.z;

	// update imgui
	POINT rect;
	GetCursorPos(&rect);
	ScreenToClient(m_hwnd, &rect);
	//std::cout << "AxisStatus = (" << rect.x << ", " << rect.y << ")" << std::endl;

	imguiBeginFrame(rect.x, rect.y,
		(GetAsyncKeyState(VK_LBUTTON) ? IMGUI_MBUT_LEFT : 0) |
		(GetAsyncKeyState(VK_RBUTTON) ? IMGUI_MBUT_RIGHT : 0) |
		(GetAsyncKeyState(VK_MBUTTON) ? IMGUI_MBUT_MIDDLE : 0)
		, 0
		, uint16_t(m_width)
		, uint16_t(m_height)
	);

	ImGui::SetNextWindowPos(
		ImVec2(m_width - m_width / 5.0f - 10.0f, 10.0f)
		, ImGuiCond_FirstUseEver
	);
	ImGui::SetNextWindowSize(
		ImVec2(m_width / 5.0f, m_height / 3.0f)
		, ImGuiCond_FirstUseEver
	);

	ImGui::Begin("Settings"
		, NULL
		, 0
	);

	static float amplitudeMul = 0.0f;
	ImGui::SliderFloat("Amplitude", &amplitudeMul, 0.0f, 1.0f);

	static float timeScale = 1.0f;
	ImGui::SliderFloat("T scale", &timeScale, -1.0f, 1.0f);

	//ImGui::Text("Camera");

	ImGui::End();

	imguiEndFrame();

	// update rendering

	// This dummy draw call is here to make sure that view 0 is cleared
	// if no other draw calls are submitted to view 0.
	bgfx::touch(0);

	float time = (float)((bx::getHPCounter() - m_timeOffset) / double(bx::getHPFrequency()));
	m_timeOffset = bx::getHPCounter();

	// model matrix
	float rotateMtx[16];
	bx::mtxRotateXYZ(rotateMtx, m_charactorRotation.x, m_charactorRotation.y, m_charactorRotation.z);

	//float time2 = (float)((bx::getHPCounter() - m_timeTotal) / double(bx::getHPFrequency()));
	//bx::mtxRotateXY(mtx, 0.5f * bx::kPi, time2 * 0.5f * bx::kPi);

	float scaleMtx[16];
	//bx::mtxScale(scaleMtx, 1.0f);
	bx::mtxScale(scaleMtx, m_characterScale);

	float translateMtx[16];
	bx::mtxTranslate(translateMtx, m_charactorPosition.x, m_charactorPosition.y, m_charactorPosition.z);

	float scaleRotateMtx[16];
	bx::mtxMul(scaleRotateMtx, scaleMtx, rotateMtx);
	bx::mtxMul(m_charactorTransform, scaleRotateMtx, translateMtx);

	trybgfx::DebugDrawEncoder dde;
	dde.begin(0);

	UpdateCamera();

	m_animator->update(time);
	//m_animator->update(0.0f);

	//m_charactorMesh->submit(0, m_AnimatedMeshPgh, finalMtx, BGFX_STATE_MASK, &dde);
	m_charactorMesh->submit(0, m_AnimatedMeshZupPgh, m_charactorTransform, BGFX_STATE_MASK);

	//float finalMtx[16];
	//bx::mtxRotateXY(rotateMtx, 0.0f, 0.0f);
	//bx::mtxScale(scaleMtx, 1.0f);
	//bx::mtxMul(finalMtx, scaleMtx, rotateMtx);
	//m_levelMesh->submit(0, m_staticMeshPgh, finalMtx, BGFX_STATE_MASK);
	for (trybgfx::TStaticActorRepository::iterator it = m_levelRepository.begin(), itEnd = m_levelRepository.end();
		it != itEnd; ++it)
	{
		trybgfx::TStaticActor* actor = *it;

		actor->submit(m_staticMeshPgh);
	}


	dde.drawAxis(0.0f, 0.1f, 0.0f, 10.0f);

	//// debug draw
	//bx::Sphere sphere = m_charactorMesh->getSphere(m_charactorPosition);
	//dde.setLod(4);
	//dde.setWireframe(true);
	//dde.draw(sphere);
	//dde.setWireframe(false);

	dde.end();

	// Advance to next frame. Rendering thread will be kicked to
	// process submitted rendering primitives.
	bgfx::frame();

	// update input system
	m_input.update();

	// update animation system
	m_animator->play(m_nextPaly);
}

void DXSample::HandleInput()
{
	//if (m_charactorVelocity.y < -1.0f)
	//{
	//	m_charactorVelocity.z = 0.0f;
	//	//m_nextPaly = 6;

	//	if (m_charactorPosition.y < -300.0f)
	//	{
	//		m_nextPaly = 3;
	//	}
	//	else
	//	{
	//		//m_nextPaly = 6;
	//		m_nextPaly = 8;
	//	}

	//	if (!m_init)
	//	{
	//		bx::Vec3 newV = { 0.0f, m_charactorVelocity.y, 0.0f };
	//		m_physics->setLinearVelocity(newV);
	//		m_init = true;
	//	}
	//	return;
	//}
	//m_init = false;

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
		float speed = 15.0f;
		if (m_input.isButtonWorkHold("turbo"))
		{
			m_nextPaly = 0;
			speed = 50.0f;
		}
		else
		{
			m_nextPaly = 15;
			speed = 15.0;
		}

		//m_charactorPosition.z += speed * bx::cos(-m_charactorRotation.y);
		//m_charactorPosition.x += speed * bx::sin(-m_charactorRotation.y);

		m_charactorVelocity.z = speed * bx::cos(-m_charactorRotation.y);
		m_charactorVelocity.x = speed * bx::sin(-m_charactorRotation.y);
		//m_charactorVelocity.y = 0.0f;
		//m_charactorVelocity.z = speed;
	}

	if (m_input.isButtonWorkPress("jump"))
	{
		m_charactorVelocity.y = 500.f;
	}
	else if(m_input.isButtonWorkRelease("jump"))
	{
		m_charactorVelocity.y = 0.0f;
	}

	if (m_input.isButtonWorkFree("forward") &&
		m_input.isButtonWorkFree("turn_left") &&
		m_input.isButtonWorkFree("turn_right") )
	{
		//m_nextPaly = 12;
		m_nextPaly = 8;

		m_charactorVelocity.x = 0.0f;
		m_charactorVelocity.z = 0.0f;
		//m_charactorVelocity.y = 0.0f;
	}

	m_physics->setLinearVelocity(m_charactorVelocity);
}

void DXSample::UpdateCamera()
{
	float tmpMtx[16];
	bx::mtxMul(tmpMtx, m_cameraTransformInv, m_charactorTransform);

	float tmpMtx2[16];
	bx::mtxMul(tmpMtx2, tmpMtx, m_cameraTransform);

	//float tmp[4] = { 0.0f, 100.0f, -100.0f, 1.0f };
	float tmp[4] = { 0.0f, 50.0f, -80.0f, 1.0f };
	//float tmp[4] = { 0.0f, 500.0f, -100.0f, 1.0f };
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

void DXSample::OnDestroy()
{
	{
		m_charactorMesh->unload();
		delete m_charactorMesh;

		m_tmplevelMesh->unload();
		delete m_tmplevelMesh;

		for (trybgfx::TStaticActorRepository::iterator it = m_levelRepository.begin(), itEnd = m_levelRepository.end();
			it != itEnd; ++it)
		{
			trybgfx::TStaticActor* actor = *it;

			actor->unload();
		}

		//m_animator->destroy();
		delete m_animator;

		delete m_camera;
	}

	bgfx::destroy(m_AnimatedMeshPgh);
	bgfx::destroy(m_AnimatedMeshZupPgh);
	bgfx::destroy(m_staticMeshPgh);

	imguiDestroy();

	trybgfx::ddShutdown();

	bgfx::shutdown();
}
