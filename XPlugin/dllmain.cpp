// dllmain.cpp : Defines the entry point for the DLL application.

#include <XPLMPlugin.h>
#include <XPLMProcessing.h>
#include <XPLMMenus.h>
#include <XPLMDataAccess.h>
#include <XPLMPlanes.h>
#include <XPWidgets.h>
#include <XPStandardWidgets.h>

#include <string>
#include <vector>
#include <fstream>
#include <map>

#include "../PID.h"

void AutoThrottleMenuHandler(void* menuRef, void* itemRef);
float getAutoSpeed(void* ref);
void setAutoSpeed(void* ref, float val);
int holdSpeedUpHandler(XPLMCommandRef cmd, XPLMCommandPhase phase, void* ref);
int holdSpeedDownHandler(XPLMCommandRef cmd, XPLMCommandPhase phase, void* ref);

int controllerWidgetCb(XPWidgetMessage msg, XPWidgetID widget, intptr_t param1, intptr_t param2);
void CreateControllerWidget();

const std::string PluginName = "AutoThrottle";
const std::string Signature = "com.v8judd.AutoThrottle";
const std::string Description = "Simple throttle controller";

XPLMMenuID autoThrottleMenuID;
int autoThrottleMenuIdx;

struct globals_t
{
	PID* pid = nullptr;
	std::string pluginPath{ "" };
	std::string plane = { "" };

	XPLMDataRef timeRef = nullptr;
	XPLMDataRef throttleRef = nullptr;
	XPLMDataRef iasRef = nullptr;
	XPLMDataRef apSpeedRef = nullptr; // Autopilot set speed
	XPLMDataRef holdSpeedRef = nullptr;

	XPWidgetID controllerWidget = nullptr;
	XPWidgetID lblHoldSpeed = nullptr;

	XPLMCommandRef holdSpeedUpCmd = nullptr;
	XPLMCommandRef holdSpeedDownCmd = nullptr;

	bool autoThrEnabled = false;
	XPLMFlightLoopID fltLoopId = nullptr;
	std::ofstream log;
	int logCnt = 0;
	float holdSpeed = 200;
	float pidT = 0;
	float limMin = 0;
	float limMax = 0;
}globals;

void loadControllerConfig(PIDController& ctrl)
{
	std::ifstream fs{ globals.pluginPath + "\\pid.ini" };
	fs.seekg(0, std::ios::end);
	int len = fs.tellg();
	fs.seekg(0, std::ios::beg);
	std::string str;
	std::vector<std::string> lines;

	while (!fs.eof())
	{
		fs >> str;
		lines.push_back(str);
	}

	std::map<std::string, float> cfg;

	// parse values
	for (auto& l : lines)
	{
		auto pos = l.find('=');
		auto key = l.substr(0, pos);
		auto val = l.substr(pos + 1);
		cfg.emplace(std::make_pair(key, atof(val.c_str())));
	}
	ctrl.Kp = cfg["kp"];
	ctrl.Ki = cfg["ki"];
	ctrl.Kd = cfg["kd"];
	ctrl.tau = cfg["tau"];
	ctrl.limMin = cfg["limMin"];
	ctrl.limMax = cfg["limMax"];
	ctrl.limMinInt = cfg["limIntMin"];
	ctrl.limMaxInt = cfg["limIntMax"];
	globals.holdSpeed = cfg["setpoint"];
	globals.pidT = cfg["pid_time"];
	globals.limMax = cfg["limMax"];
	globals.limMin = cfg["limMin"];

	if (globals.log.is_open())
	{
		globals.log.flush();
		globals.log.close();
	}

	globals.log.open(globals.pluginPath + "\\log" + std::to_string(globals.logCnt) + ".csv");
	++globals.logCnt;

	globals.log << "Kp: " << ctrl.Kp << std::endl;
	globals.log << "Ki: " << ctrl.Ki << std::endl;
	globals.log << "Kd: " << ctrl.Kd << std::endl;
	globals.log << "tau: " << ctrl.tau << std::endl;
	globals.log << "limMin: " << ctrl.limMin << std::endl;
	globals.log << "limMax: " << ctrl.limMax << std::endl;
	globals.log << "intLimMin: " << ctrl.limMinInt << std::endl;
	globals.log << "intLimMax: " << ctrl.limMaxInt << std::endl;
}

XPLMCreateFlightLoop_t controllerLoop{
	sizeof(XPLMCreateFlightLoop_t),
	xplm_FlightLoop_Phase_AfterFlightModel,
	[](float timeSinceLastCall, float timeSinceLastLoop, int counter, void* ref)->float {

		static float lastTime = 0;
		static float t = 0;
		static float lastLogTime = 0;

		std::string lv = std::to_string(globals.holdSpeed);
		XPSetWidgetDescriptor(globals.lblHoldSpeed, lv.c_str());

		if (!globals.autoThrEnabled)
		{
			lastLogTime = 0;
			lastTime = 0;
			return globals.pidT;
		}

		if (0 == lastTime)
		{
			if (globals.log.is_open())
			{
				globals.log << "setpoint: " << globals.holdSpeed << std::endl;
				globals.log << "t;error;speed;out;setpoint" << std::endl;
			}

			lastTime = XPLMGetDataf(globals.timeRef);
			t = 0;
		}
		auto deltaT = XPLMGetDataf(globals.timeRef) - lastTime;
		if (deltaT <= 0.000001f)
			return globals.pidT;

		globals.pid->setTime(deltaT);
		auto ias = XPLMGetDataf(globals.iasRef);

		auto prevErr = globals.pid->data().prevError;
		if (prevErr > 15)
			globals.pid->setMaxLimit(0.95f);
		else if (prevErr < 5)
			globals.pid->setMaxLimit(0.625);
		else
			globals.pid->setMaxLimit(globals.limMax);

		if (prevErr < 15)
			globals.pid->setMinLimit(0);
		else if (prevErr > -5)
			globals.pid->setMinLimit(0.375);
		else
			globals.pid->setMinLimit(globals.limMin);

		auto err = globals.pid->update(globals.holdSpeed, ias);

		XPLMSetDataf(globals.throttleRef, globals.pid->data().out);

		lastTime = XPLMGetDataf(globals.timeRef);
		t += deltaT;
		if (0 == lastLogTime)
			lastLogTime = t;
		if (t - lastLogTime > 0.1f)
		{
			lastLogTime = t;
			if (globals.log.is_open())
			{
				globals.log << t << ";" << err << ";" << ias << ";" << globals.pid->data().out << ";" << globals.holdSpeed << std::endl;
			}
		}
		return globals.pidT;
	}
};

PLUGIN_API int XPluginStart(char* name, char* sig, char* desc)
{
	strcpy_s(name, 256, PluginName.c_str());
	strcpy_s(sig, 256, Signature.c_str());
	strcpy_s(desc, 256, Description.c_str());

	XPLMDebugString("[TK] AutoThrottle plugin loaded\n");
	XPLMEnableFeature("XPLM_USE_NATIVE_WIDGET_WINDOWS", 1);

	autoThrottleMenuIdx = XPLMAppendMenuItem(XPLMFindPluginsMenu(), "AutoThrottle", nullptr, 0);
	autoThrottleMenuID = XPLMCreateMenu("AutoThrottle", XPLMFindPluginsMenu(), autoThrottleMenuIdx, AutoThrottleMenuHandler, nullptr);
	XPLMAppendMenuItem(autoThrottleMenuID, "Enable", (void*)"enable", 0);
	XPLMAppendMenuItem(autoThrottleMenuID, "Disable", (void*)"disable", 0);
	XPLMAppendMenuItem(autoThrottleMenuID, "Reload Config", (void*)"reload", 0);
	XPLMAppendMenuSeparator(autoThrottleMenuID);
	XPLMAppendMenuItem(autoThrottleMenuID, "Show Config", (void*)"config", 0);

	globals.timeRef = XPLMFindDataRef("sim/time/total_running_time_sec");
	globals.throttleRef = XPLMFindDataRef("sim/cockpit2/engine/actuators/throttle_ratio_all");
	globals.iasRef = XPLMFindDataRef("sim/cockpit2/gauges/indicators/airspeed_kts_pilot");
	globals.apSpeedRef = XPLMFindDataRef("sim/cockpit2/autopilot/airspeed_dial_kts");
	globals.holdSpeedRef = XPLMRegisterDataAccessor("v8judd/auto_throttle/hold_speed", xplmType_Float, true, nullptr, nullptr, getAutoSpeed, setAutoSpeed, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
	globals.holdSpeedUpCmd = XPLMCreateCommand("v8judd/auto_throttle/hold_speed_up", "Hold speed up");
	globals.holdSpeedDownCmd = XPLMCreateCommand("v8judd/auto_throttle/hold_speed_down", "Hold speed down");

	XPLMRegisterCommandHandler(globals.holdSpeedUpCmd, holdSpeedUpHandler, 1, nullptr);
	XPLMRegisterCommandHandler(globals.holdSpeedDownCmd, holdSpeedDownHandler, 1, nullptr);

	char filePath[512] = { 0 };
	XPLMGetPluginInfo(XPLMGetMyID(), nullptr, filePath, nullptr, nullptr);
	std::string tmp{ filePath };
	auto pos = tmp.find_last_of('\\');
	globals.pluginPath = tmp.substr(0, pos);

	PIDController ctrl{ 0 };
	loadControllerConfig(ctrl);
	globals.pid = new PID{ ctrl };

	controllerLoop.refcon = nullptr;
	controllerLoop.structSize = sizeof(controllerLoop);
	globals.fltLoopId = XPLMCreateFlightLoop(&controllerLoop);

	return 1;
}

PLUGIN_API void XPluginStop()
{
	XPLMDebugString("[TK] XPluginStop() called\n");
	if (globals.controllerWidget != nullptr)
	{
		XPDestroyWidget(globals.controllerWidget, 1);
		globals.controllerWidget = nullptr;
	}
	XPLMDestroyMenu(autoThrottleMenuID);

	delete globals.pid;
}

PLUGIN_API int XPluginEnable()
{
	XPLMDebugString("[TK] XPluginEnable() called\n");

	return 1;
}

PLUGIN_API void XPluginDisable()
{
	XPLMDebugString("[TK] XPluginDisable() called\n");
	XPLMDestroyFlightLoop(globals.fltLoopId);
}

PLUGIN_API void XPluginReceiveMessage(XPLMPluginID from, int msg, void* param)
{
	XPLMDebugString("[TK] XPluginReceiveMessage() called\n");
	switch (msg)
	{
		case XPLM_MSG_PLANE_LOADED:
			if (reinterpret_cast<int>(param) == 0)
			{
				char file[256] = { 0 }, path[512] = { 0 };

				// user aircraft loaded
				XPLMGetNthAircraftModel(0, file, path);
				globals.plane = file;
				if (globals.plane.compare("Cessna_CitationX.acf") == 0)
					XPLMScheduleFlightLoop(globals.fltLoopId, 0.02f, 0);
				else
					XPLMScheduleFlightLoop(globals.fltLoopId, 0, 0);

			}
			break;
			//case XPLM_MSG_PLANE_UNLOADED:
			//	break;
	}
}

void AutoThrottleMenuHandler(void* menuRef, void* itemRef)
{
	if (nullptr == itemRef)
		return;

	std::string str{ reinterpret_cast<char*>(itemRef) };
	if ("enable" == str)
	{
		// enable controller
		globals.autoThrEnabled = true;

	} else if ("disable" == str)
	{
		// disable controller
		globals.autoThrEnabled = false;

	} else if ("reload" == str)
	{
		globals.autoThrEnabled = false;
		// reload controller config from file
		PIDController ctrl{ 0 };
		loadControllerConfig(ctrl);
		globals.pid->updateConfig(ctrl);
	} else if ("config" == str)
	{
		if (globals.controllerWidget == nullptr)
			CreateControllerWidget();
		else
			XPShowWidget(globals.controllerWidget);
	}
}

float getAutoSpeed(void* ref)
{
	return globals.holdSpeed;
}

void setAutoSpeed(void* ref, float val)
{
	if (nullptr == ref)
		return;

	globals.holdSpeed = val;
}

int holdSpeedUpHandler(XPLMCommandRef cmd, XPLMCommandPhase phase, void* ref)
{
	if (phase == 0)
	{
		if (globals.holdSpeed <= 320)
			globals.holdSpeed++;
	}
	return 0;
}

int holdSpeedDownHandler(XPLMCommandRef cmd, XPLMCommandPhase phase, void* ref)
{
	if (phase == 0)
	{
		if (globals.holdSpeed >= 120)
			globals.holdSpeed--;
	}
	return 0;
}


void CreateControllerWidget()
{
	int l, t, r, b;
	XPLMGetScreenBoundsGlobal(&l, &t, &r, &b); // 0, 1080, 1920, 0
	int x = l + 50, y = t - 250, w = 200, h = 100;
	int x2 = w + x;
	int y2 = y - h;
	globals.controllerWidget = XPCreateWidget(x, y, x2, y2, 1, "AutoThrottle Controller", 1, nullptr, xpWidgetClass_MainWindow);
	XPSetWidgetProperty(globals.controllerWidget, xpProperty_MainWindowType, xpMainWindowStyle_MainWindow);
	XPSetWidgetProperty(globals.controllerWidget, xpProperty_MainWindowHasCloseBoxes, 1);
	auto lblText = XPCreateWidget(x + 10, y - 20, x + 50, y - 40, 1, "Speed: ", 0, globals.controllerWidget, xpWidgetClass_Caption);
	std::string str = std::to_string(200.0f);

	globals.lblHoldSpeed = XPCreateWidget(x + 55, y - 20, x2 - 25, y - 40, 1, str.c_str(), 0, globals.controllerWidget, xpWidgetClass_Caption);

	//auto wnd = XPGetWidgetUnderlyingWindow(globals.controllerWidget);
	//XPLMSetWindowPositioningMode(wnd, xplm_WindowPositionFree, -1);
	//XPLMSetWindowResizingLimits(wnd, 200, 200, 300, 300);
	//XPLMSetWindowTitle(wnd, "Sample Window");

	XPAddWidgetCallback(globals.controllerWidget, controllerWidgetCb);
	XPShowWidget(globals.controllerWidget);
	XPBringRootWidgetToFront(globals.controllerWidget);
}

int controllerWidgetCb(XPWidgetMessage msg, XPWidgetID widget, intptr_t param1, intptr_t param2)
{
	if (msg == xpMessage_CloseButtonPushed)
	{
		XPHideWidget(globals.controllerWidget);
		return 1;
	}

	return 0;
}