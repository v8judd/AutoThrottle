// dllmain.cpp : Defines the entry point for the DLL application.

#include <XPLM/XPLMPlugin.h>
#include <XPLM/XPLMProcessing.h>
#include <XPLM/XPLMMenus.h>
#include <XPLM/XPLMDataAccess.h>
#include <XPLM/XPLMPlanes.h>
#include <string>
#include <vector>
#include <fstream>
#include <map>

#include "../PID.h"

void AutoThrottleMenuHandler(void* menuRef, void* itemRef);

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
	XPLMDataRef apSpeed = nullptr; // Autopilot set speed

	bool autoThrEnabled = false;
	XPLMFlightLoopID fltLoopId = nullptr;
	float setSpeed = 0; // setpoint AP speed
	std::ofstream log;
	int logCnt = 0;

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

	if (globals.log.is_open())
	{
		globals.log.flush();
		globals.log.close();
	}

	globals.log.open(globals.pluginPath + "\\log" + std::to_string(globals.logCnt) + ".log");
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

		if (!globals.autoThrEnabled)
		{
			lastTime = 0;
			return 0.02f;
		}

		if (0 == lastTime)
		{
			if (globals.log.is_open())
			{
				globals.log << "setpoint: " << globals.setSpeed << std::endl;
				globals.log << "t;error;out" << std::endl;
			}

			lastTime = XPLMGetDataf(globals.timeRef);
			t = 0;
		}
		auto deltaT = XPLMGetDataf(globals.timeRef) - lastTime;
		if (deltaT <= 0.000001f)
			return 0.02f;

		globals.pid->setTime(deltaT);
		auto ias = XPLMGetDataf(globals.iasRef);
		auto err = globals.pid->update(globals.setSpeed, ias);
		XPLMSetDataf(globals.throttleRef, globals.pid->data().out);

		lastTime = XPLMGetDataf(globals.timeRef);
		t += deltaT;

		if (globals.log.is_open())
		{
			globals.log << t << ";" << err << ";" << globals.pid->data().out << std::endl;
		}

		return 0.02f;
	}
};

PLUGIN_API int XPluginStart(char* name, char* sig, char* desc)
{
	strcpy_s(name, 256, PluginName.c_str());
	strcpy_s(sig, 256, Signature.c_str());
	strcpy_s(desc, 256, Description.c_str());

	XPLMDebugString("[TK] AutoThrottle plugin loaded\n");

	autoThrottleMenuIdx = XPLMAppendMenuItem(XPLMFindPluginsMenu(), "AutoThrottle", nullptr, 0);
	autoThrottleMenuID = XPLMCreateMenu("AutoThrottle", XPLMFindPluginsMenu(), autoThrottleMenuIdx, AutoThrottleMenuHandler, nullptr);
	XPLMAppendMenuItem(autoThrottleMenuID, "Enable", (void*)"enable", 0);
	XPLMAppendMenuItem(autoThrottleMenuID, "Disable", (void*)"disable", 0);
	XPLMAppendMenuItem(autoThrottleMenuID, "Reload Config", (void*)"reload", 0);

	globals.timeRef = XPLMFindDataRef("sim/time/total_running_time_sec");
	globals.throttleRef = XPLMFindDataRef("sim/cockpit2/engine/actuators/throttle_ratio_all");
	globals.iasRef = XPLMFindDataRef("sim/cockpit2/gauges/indicators/airspeed_kts_pilot");
	globals.apSpeed = XPLMFindDataRef("sim/cockpit2/autopilot/airspeed_dial_kts");

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

	//XPLMDestroyMenu(autoThrottleMenuID);
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
		// for now the setpoint is the current
		// speed when the controller is enabled
		globals.setSpeed = XPLMGetDataf(globals.iasRef);

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
	}
}