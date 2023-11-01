// AutoThrottle.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>

#include "PID.h"

#define SAMPLE_TIME_S 0.01f

/* Maximum run-time of simulation */
#define SIMULATION_TIME_MAX 3.0f


float TestSystem_Update(float inp)
{

	static float output = 0.0f;
	static const float alpha = 0.02f;

	output = (SAMPLE_TIME_S * inp + output) / (1.0f + alpha * SAMPLE_TIME_S);

	return output;
}

void loadControllerConfig(PIDController& ctrl)
{
	std::ifstream fs{ "pid.ini" };
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

	int x = 0;
}

int main()
{
	PIDController pc{ 0 };
	pc.T = 0.01f;
	loadControllerConfig(pc);

	PID pid{ pc };

	/* Simulate response using test system */
	float setpoint = 252.0f;
	static float out = 0;
	int i = 0;

	printf("Time (s)\tSystem Output\tControllerOutput\r\n");
	for (float t = 0.0f; t <= SIMULATION_TIME_MAX; t += SAMPLE_TIME_S)
	{
		/* Get measurement from system */
		float measurement = /*250 + (i * SAMPLE_TIME_S)*/TestSystem_Update(250.0f + i);

		/* Compute new control signal */
		out = pid.update(setpoint, measurement);

		printf("%f\t%f\t%f\r\n", t, measurement, out);
		++i;
	}

	return 0;
}
