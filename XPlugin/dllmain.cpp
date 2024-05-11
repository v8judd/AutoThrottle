// dllmain.cpp : Defines the entry point for the DLL application.

#include <XPLMPlugin.h>
#include <XPLMProcessing.h>
#include <XPLMMenus.h>
#include <XPLMDataAccess.h>
#include <XPLMPlanes.h>
#include <XPWidgets.h>
#include <XPStandardWidgets.h>
#include <XPLMGraphics.h>

#include <string>
#include <vector>
#include <fstream>
#include <map>
#include <sstream>

#include "../PID.h"

///
/// ideas: 
///  - disable AT and set idle when 100 ft above ground (AGL)
///  - minimum setable speeds per aircraft
///	 - take into account ITT / max Torque when setting max output value

/// <summary>
/// 
/// </summary>
/// <param name="menuRef"></param>
/// <param name="itemRef"></param>
void AutoThrottleMenuHandler(void* menuRef, void* itemRef);
float getAutoSpeed(void* ref);
void setAutoSpeed(void* ref, float val);
int holdSpeedUpHandler(XPLMCommandRef cmd, XPLMCommandPhase phase, void* ref);
int holdSpeedDownHandler(XPLMCommandRef cmd, XPLMCommandPhase phase, void* ref);
int autoThrottleToggleHandler(XPLMCommandRef cmd, XPLMCommandPhase phase, void* ref);

int controllerWidgetCb(XPWidgetMessage msg, XPWidgetID widget, intptr_t param1, intptr_t param2);
void CreateControllerWidget();

const std::string PluginName = "AutoThrottle";
const std::string Signature = "com.v8judd.AutoThrottle";
const std::string Description = "Simple throttle controller";

XPLMMenuID autoThrottleMenuID;
int autoThrottleMenuIdx;

struct globals_t
{
	std::unique_ptr<PID> pid = nullptr;
	std::string pluginPath{ "" };
	std::string plane = { "" };

	XPLMDataRef timeRef = nullptr;
	XPLMDataRef throttleRef = nullptr;
	XPLMDataRef iasRef = nullptr;
	XPLMDataRef apSpeedRef = nullptr; // Autopilot set speed
	XPLMDataRef holdSpeedRef = nullptr;

	XPWidgetID controllerWidget = nullptr;
	XPWidgetID lblHoldSpeed = nullptr;
	XPLMWindowID controllerWnd = nullptr;

	XPLMCommandRef holdSpeedUpCmd = nullptr;
	XPLMCommandRef holdSpeedDownCmd = nullptr;
	XPLMCommandRef autoThrottleToggleCmd = nullptr;

	bool autoThrEnabled = false;
	XPLMFlightLoopID fltLoopId = nullptr;
	std::ofstream log;
	int logCnt = 0;
	float holdSpeed = 200;
	float pidT = 0;
	float limMin = 0;
	float limMax = 0;
}globals;

bool loadControllerConfig(const std::string& fileName, PIDController& ctrl)
{
	std::ifstream fs{ globals.pluginPath + "\\" + fileName };
	std::string str;
	std::vector<std::string> lines;

	if (!fs.is_open())
	{
		std::ostringstream ss;
		ss << "[TK] failed to load config for aircraft: " << globals.plane << std::endl;
		ss << "does the file " << fileName << " exist?" << std::endl;
		XPLMDebugString(ss.str().c_str());
		return false;
	}

	while (!fs.eof())
	{
		fs >> str;
		if (str.empty()) // filter empty lines
			continue;

		if (str.substr(0, 2) == "//" || str.substr(0, 1) == "#") // filter comments
			continue;

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
	ctrl.T = globals.pidT;

	return true;
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
				globals.log << "t;error;speed;out;setpoint;Int;Diff" << std::endl;
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
		//if (prevErr > 15)
		//	globals.pid->setMaxLimit(0.95f);
		//else if (prevErr < 5)
		//	globals.pid->setMaxLimit(0.625);
		//else
			globals.pid->setMaxLimit(globals.limMax);

			//if (prevErr < 15)
			//	globals.pid->setMinLimit(0);
			//else if (prevErr > -5)
			//	globals.pid->setMinLimit(0.375);
			//else
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
					globals.log << t << ";";
					globals.log << err << ";";
					globals.log << ias << ";";
					globals.log << globals.pid->data().out << ";";
					globals.log << globals.holdSpeed << ";";
					globals.log << globals.pid->data().integrator << ";";
					globals.log << globals.pid->data().differentiator << std::endl;
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
	globals.autoThrottleToggleCmd = XPLMCreateCommand("v8judd/auto_throttle/ATtoggle", "AutoThrottle toggle");

	XPLMRegisterCommandHandler(globals.holdSpeedUpCmd, holdSpeedUpHandler, 1, nullptr);
	XPLMRegisterCommandHandler(globals.holdSpeedDownCmd, holdSpeedDownHandler, 1, nullptr);
	XPLMRegisterCommandHandler(globals.autoThrottleToggleCmd, autoThrottleToggleHandler, 1, nullptr);

	char filePath[512] = { 0 };
	XPLMGetPluginInfo(XPLMGetMyID(), nullptr, filePath, nullptr, nullptr);
	std::string tmp{ filePath };
	auto pos = tmp.find_last_of('\\');
	globals.pluginPath = tmp.substr(0, pos);

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

	//delete globals.pid;
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
				std::string acFile{ file };
				auto pos = acFile.find_last_of('.');
				if (std::string::npos != pos)
				{
					acFile = acFile.substr(0, pos);
				}

				globals.plane = acFile;
				PIDController ctrl{ 0 };

				// if controller config fails to load -> abort
				if (!loadControllerConfig(acFile + ".ini", ctrl))
					break;

				// re-initialize new pointer to PID 
				globals.pid.reset(new PID{ ctrl });

				if (globals.plane.compare("Cessna_CitationX") == 0)
					XPLMScheduleFlightLoop(globals.fltLoopId, globals.pidT, 0);
				else if (globals.plane.compare("C90B") == 0)
					XPLMScheduleFlightLoop(globals.fltLoopId, globals.pidT, 0);
				//else
					//XPLMScheduleFlightLoop(globals.fltLoopId, 0, 0);

			}
			break;

		case XPLM_MSG_PLANE_UNLOADED:
			XPLMScheduleFlightLoop(globals.fltLoopId, 0, 0);
			break;
	}
}

void enableAutoThrottle()
{
	// NEW: start new log when auto throttle enabled
	if (globals.log.is_open())
	{
		globals.log.flush();
		globals.log.close();
	}

	globals.log.open(globals.pluginPath + "\\" + globals.plane + "_log" + std::to_string(globals.logCnt) + ".csv");
	++globals.logCnt;

	auto& ctrl = globals.pid->data();

	globals.log << "Airframe: " << globals.plane << std::endl;
	globals.log << "Kp: " << ctrl.Kp << std::endl;
	globals.log << "Ki: " << ctrl.Ki << std::endl;
	globals.log << "Kd: " << ctrl.Kd << std::endl;
	globals.log << "tau: " << ctrl.tau << std::endl;
	globals.log << "limMin: " << ctrl.limMin << std::endl;
	globals.log << "limMax: " << ctrl.limMax << std::endl;
	globals.log << "intLimMin: " << ctrl.limMinInt << std::endl;
	globals.log << "intLimMax: " << ctrl.limMaxInt << std::endl;
	globals.log << "holdSpeed: " << globals.holdSpeed << std::endl;
	globals.log << "T: " << globals.pidT << std::endl;

	globals.autoThrEnabled = true;
}

void disableAutoThrottle()
{
	if (globals.log.is_open())
	{
		globals.log.flush();
		globals.log.close();
	}

	globals.autoThrEnabled = false;
}

void AutoThrottleMenuHandler(void* menuRef, void* itemRef)
{
	if (nullptr == itemRef)
		return;

	std::string str{ reinterpret_cast<char*>(itemRef) };
	if ("enable" == str)
	{
		enableAutoThrottle();

	} else if ("disable" == str)
	{
		disableAutoThrottle();

	} else if ("reload" == str)
	{
		globals.autoThrEnabled = false;
		// reload controller config from file
		PIDController ctrl{ 0 };
		loadControllerConfig(globals.plane + ".ini", ctrl);
		globals.pid->updateConfig(ctrl);
	} else if ("config" == str)
	{
		if (globals.controllerWnd == nullptr)
		{
			CreateControllerWidget();
			if (!XPLMGetWindowIsVisible(globals.controllerWnd))
				XPLMSetWindowIsVisible(globals.controllerWnd, 1);
		} else
		{
			//XPShowWidget(globals.controllerWidget);
			if (!XPLMGetWindowIsVisible(globals.controllerWnd))
				XPLMSetWindowIsVisible(globals.controllerWnd, 1);
			else
				XPLMSetWindowIsVisible(globals.controllerWnd, 0);
		}
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
	static float startTime = 0;

	if (phase == xplm_CommandBegin)
	{
		if (globals.holdSpeed <= 320)
			globals.holdSpeed++;

		startTime = XPLMGetElapsedTime();
	} else if (phase == xplm_CommandContinue && XPLMGetElapsedTime() - startTime > 0.5)
	{
		if (globals.holdSpeed <= 320)
			globals.holdSpeed++;
	}
	return 0;
}

int holdSpeedDownHandler(XPLMCommandRef cmd, XPLMCommandPhase phase, void* ref)
{
	static float startTime = 0;

	if (phase == xplm_CommandBegin)
	{
		if (globals.holdSpeed >= 120)
			globals.holdSpeed--;

		startTime = XPLMGetElapsedTime();
	} else if (phase == xplm_CommandContinue && XPLMGetElapsedTime() - startTime > 0.5)
	{
		if (globals.holdSpeed >= 120)
			globals.holdSpeed--;
	}
	return 0;
}

int autoThrottleToggleHandler(XPLMCommandRef cmd, XPLMCommandPhase phase, void* ref)
{
	if (phase == 0)
	{
		if (globals.autoThrEnabled)
			disableAutoThrottle();
		else
			enableAutoThrottle();
	}
	return 0;
}

//void CreateControllerWidget()
//{
//	int l, t, r, b;
//	XPLMGetScreenBoundsGlobal(&l, &t, &r, &b); // 0, 1080, 1920, 0
//	int x = l + 50, y = t - 250, w = 200, h = 100;
//	int x2 = w + x;
//	int y2 = y - h;
//	globals.controllerWidget = XPCreateWidget(x, y, x2, y2, 1, "AutoThrottle Controller", 1, nullptr, xpWidgetClass_MainWindow);
//	XPSetWidgetProperty(globals.controllerWidget, xpProperty_MainWindowType, xpMainWindowStyle_MainWindow);
//	XPSetWidgetProperty(globals.controllerWidget, xpProperty_MainWindowHasCloseBoxes, 1);
//	auto lblText = XPCreateWidget(x + 10, y - 20, x + 50, y - 40, 1, "Speed: ", 0, globals.controllerWidget, xpWidgetClass_Caption);
//	std::string str = std::to_string(200.0f);
//
//	globals.lblHoldSpeed = XPCreateWidget(x + 55, y - 20, x2 - 25, y - 40, 1, str.c_str(), 0, globals.controllerWidget, xpWidgetClass_Caption);
//
//	//auto wnd = XPGetWidgetUnderlyingWindow(globals.controllerWidget);
//	//XPLMSetWindowPositioningMode(wnd, xplm_WindowPositionFree, -1);
//	//XPLMSetWindowResizingLimits(wnd, 200, 200, 300, 300);
//	//XPLMSetWindowTitle(wnd, "Sample Window");
//
//	XPAddWidgetCallback(globals.controllerWidget, controllerWidgetCb);
//	XPShowWidget(globals.controllerWidget);
//	XPBringRootWidgetToFront(globals.controllerWidget);
//}

void wndDrawFn(XPLMWindowID wndId, void* ref)
{
	XPLMSetGraphicsState(0, 0, 0, 0, 1, 1, 0);

	int l = 0, t = 0, r = 0, b = 0;

	XPLMGetWindowGeometry(wndId, &l, &t, &r, &b);
	int h = t - b;
	XPLMDrawTranslucentDarkBox(l, t, r, b);
	float color[] = { 1.0,1.0,1.0 };
	const char lblText[] = "Set Speed:";
	int textHeight = 0;
	XPLMGetFontDimensions(xplmFont_Proportional, nullptr, &textHeight, nullptr);

	std::string s = std::to_string(static_cast<int>(globals.holdSpeed));
	int lblWidth = XPLMMeasureString(xplmFont_Proportional, lblText, strlen(lblText));
	int valPos = l + 5 + lblWidth + 5;
	XPLMDrawString(color, l + 5, b + ((h / 2) - (textHeight / 2)), const_cast<char*>(lblText), nullptr, xplmFont_Proportional);
	XPLMDrawString(color, valPos, b + ((h / 2) - (textHeight / 2)), const_cast<char*>(s.c_str()), nullptr, xplmFont_Proportional);
	int statusPos = XPLMMeasureString(xplmFont_Proportional, s.c_str(), s.length()) + 5 + valPos;
	XPLMDrawString(color, statusPos, b + ((h / 2) - (textHeight / 2)), (char*)"On", nullptr, xplmFont_Proportional);

	if (globals.autoThrEnabled)
	{
		float colorOn[] = { 1.0, 0, 0 };
		XPLMDrawString(colorOn, statusPos, b + ((h / 2) - (textHeight / 2)), (char*)"On", nullptr, xplmFont_Proportional);
	} else
		XPLMDrawString(color, statusPos, b + ((h / 2) - (textHeight / 2)), (char*)"Off", nullptr, xplmFont_Proportional);
}

void CreateControllerWidget()
{
	int l, t, r, b;
	XPLMGetScreenBoundsGlobal(&l, &t, &r, &b); // 0, 1080, 1920, 0
	int x = l + 25, y = t - 100;
	int w = 130, h = 50;

	XPLMCreateWindow_t wnd{ 0 };
	wnd.structSize = sizeof(wnd);
	wnd.left = x;
	wnd.top = y;
	wnd.right = x + w;
	wnd.bottom = y - h;
	wnd.drawWindowFunc = wndDrawFn;
	wnd.handleCursorFunc = nullptr;
	wnd.handleKeyFunc = nullptr;
	wnd.handleMouseClickFunc = nullptr;
	wnd.handleMouseWheelFunc = nullptr;
	wnd.handleRightClickFunc = nullptr;
	wnd.decorateAsFloatingWindow = xplm_WindowDecorationNone;
	wnd.layer = xplm_WindowLayerFlightOverlay;
	wnd.visible = false;
	wnd.refcon = nullptr;

	globals.controllerWnd = XPLMCreateWindowEx(&wnd);
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