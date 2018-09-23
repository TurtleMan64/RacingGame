#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <iostream>
#include <cmath>
#include <fstream>
#include <string>
#include <cstring>
#include <algorithm>

#include "input.h"
#include "../renderEngine/skymanager.h"
#include "../engineTester/main.h"
#include "../entities/camera.h"
#include "../entities/car.h"
#include "maths.h"
#include "../toolbox/split.h"
#include <random>
#include <chrono>

extern GLFWwindow* window;

InputStruct Input::inputs{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

//settings

bool freeMouse = true;

float mouseSensitivityX = 0.25f;
float mouseSensitivityY = 0.25f;

float stickSensitivityX = 2.5f;
float stickSensitivityY = 2.5f;

float triggerSensitivity = 2;

int CONTROLLER_ID = 0;

int BUTTON_A      = 0;
int BUTTON_X      = 1;
int BUTTON_B      = 2;
int BUTTON_Y      = 3;
int BUTTON_LB     = 4;
int BUTTON_RB     = 5;
int BUTTON_SELECT = 6;
int BUTTON_START  = 7;

int   STICK_LX       = 0;
float STICK_LX_SCALE = 1;
int   STICK_LY       = 1;
float STICK_LY_SCALE = 1;
int   STICK_RX       = 3;
float STICK_RX_SCALE = 1;
int   STICK_RY       = 4;
float STICK_RY_SCALE = 1;

float STICK_LXDEADZONE = 0.1f;
float STICK_LYDEADZONE = 0.1f;
float STICK_RXDEADZONE = 0.1f;
float STICK_RYDEADZONE = 0.1f;

int   TRIGGER_L  =  7;
float LT_NEUTRAL = -1;
float LT_MAX     =  1;
float LT_RANGE   =  2;
int   TRIGGER_R  =  5;
float RT_NEUTRAL = -1;
float RT_MAX     =  1;
float RT_RANGE   =  2;

float TRIGGER_DEADZONE = 0.3f;



void Input::pollInputs()
{
	glfwPollEvents();

	Input::inputs.INPUT_PREVIOUS_JUMP     = Input::inputs.INPUT_JUMP;
	Input::inputs.INPUT_PREVIOUS_ACTION   = Input::inputs.INPUT_ACTION;
	Input::inputs.INPUT_PREVIOUS_ACTION2  = Input::inputs.INPUT_ACTION2;
	Input::inputs.INPUT_PREVIOUS_SHOULDER = Input::inputs.INPUT_SHOULDER;
	Input::inputs.INPUT_PREVIOUS_SPECIAL  = Input::inputs.INPUT_SPECIAL;
	Input::inputs.INPUT_PREVIOUS_START    = Input::inputs.INPUT_START;

	Input::inputs.INPUT_PREVIOUS_X  = Input::inputs.INPUT_X;
	Input::inputs.INPUT_PREVIOUS_Y  = Input::inputs.INPUT_Y;
	Input::inputs.INPUT_PREVIOUS_X2 = Input::inputs.INPUT_X2;
	Input::inputs.INPUT_PREVIOUS_Y2 = Input::inputs.INPUT_Y2;
	Input::inputs.INPUT_PREVIOUS_R2 = Input::inputs.INPUT_L2;
	Input::inputs.INPUT_PREVIOUS_L2 = Input::inputs.INPUT_R2;


	Input::inputs.INPUT_JUMP = false;
	Input::inputs.INPUT_ACTION = false;
	Input::inputs.INPUT_ACTION2 = false;
	Input::inputs.INPUT_SHOULDER = false;
	Input::inputs.INPUT_SELECT = false;
	Input::inputs.INPUT_SPECIAL = false;
	Input::inputs.INPUT_START = false;

	Input::inputs.INPUT_X = 0;
	Input::inputs.INPUT_Y = 0;
	Input::inputs.INPUT_X2 = 0;
	Input::inputs.INPUT_Y2 = 0;
	Input::inputs.INPUT_L2 = 0;
	Input::inputs.INPUT_R2 = 0;



	int present = glfwJoystickPresent(CONTROLLER_ID);
	if (present == 1)
	{
		int axesCount;
		const float *axes = glfwGetJoystickAxes(CONTROLLER_ID, &axesCount);

		Input::inputs.INPUT_X = axes[STICK_LX] * STICK_LX_SCALE;
		Input::inputs.INPUT_Y = axes[STICK_LY] * STICK_LY_SCALE;

		Input::inputs.INPUT_X2 = axes[STICK_RX] * STICK_RX_SCALE;
		Input::inputs.INPUT_Y2 = axes[STICK_RY] * STICK_RY_SCALE;

		if (abs(Input::inputs.INPUT_X)  < STICK_LXDEADZONE) { Input::inputs.INPUT_X  = 0; }
		if (abs(Input::inputs.INPUT_Y)  < STICK_LYDEADZONE) { Input::inputs.INPUT_Y  = 0; }
		if (abs(Input::inputs.INPUT_X2) < STICK_RXDEADZONE) { Input::inputs.INPUT_X2 = 0; }
		if (abs(Input::inputs.INPUT_Y2) < STICK_RYDEADZONE) { Input::inputs.INPUT_Y2 = 0; }

		Input::inputs.INPUT_X2 = Input::inputs.INPUT_X2 * stickSensitivityX;
		Input::inputs.INPUT_Y2 = Input::inputs.INPUT_Y2 * stickSensitivityY;


		float triggerLValue = 0;
		float triggerRValue = 0;


		float rawValue = (axes[TRIGGER_L] - LT_NEUTRAL) / LT_RANGE;
		if (rawValue >= TRIGGER_DEADZONE) { triggerLValue = rawValue; }

		rawValue = (axes[TRIGGER_R] - RT_NEUTRAL) / RT_RANGE;
		if (rawValue >= TRIGGER_DEADZONE) { triggerRValue = rawValue; }


		Input::inputs.INPUT_X2 += triggerSensitivity * (triggerLValue - triggerRValue);

		Input::inputs.INPUT_L2 = triggerLValue;
		Input::inputs.INPUT_R2 = triggerRValue;


		int buttonCount;
		const unsigned char *buttons = glfwGetJoystickButtons(CONTROLLER_ID, &buttonCount);

		Input::inputs.INPUT_JUMP     = buttons[BUTTON_A];
		Input::inputs.INPUT_ACTION   = buttons[BUTTON_B];
		Input::inputs.INPUT_ACTION2  = buttons[BUTTON_X];
		Input::inputs.INPUT_SPECIAL  = buttons[BUTTON_Y];
		Input::inputs.INPUT_SHOULDER = buttons[BUTTON_RB];
		Input::inputs.INPUT_START    = buttons[BUTTON_START];



		//const char *name = glfwGetJoystickName(GLFW_JOYSTICK_1);
		//std::fprintf(stdout, "joystick name: %s\n", name);
	}




	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
	{
		glfwSetWindowShouldClose(window, true);
	}

	if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS)
	{
		Input::inputs.INPUT_JUMP = true;
	}
	if (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS)
	{
		Input::inputs.INPUT_ACTION = true;
	}
	if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS)
	{
		Input::inputs.INPUT_ACTION2 = true;
	}
	if (glfwGetKey(window, GLFW_KEY_F) == GLFW_PRESS)
	{
		Input::inputs.INPUT_SPECIAL = true;
	}
	if (glfwGetKey(window, GLFW_KEY_ENTER) == GLFW_PRESS)
	{
		Input::inputs.INPUT_START = true;
	}
	if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS)
	{
		Input::inputs.INPUT_SHOULDER = true;
	}

	if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
	{
		Input::inputs.INPUT_Y = -1;
	}
	if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
	{
		Input::inputs.INPUT_Y = 1;
	}
	if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
	{
		Input::inputs.INPUT_X = -1;
	}
	if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
	{
		Input::inputs.INPUT_X = 1;
	}


	if (glfwGetKey(window, GLFW_KEY_0) == GLFW_PRESS)
	{
		SkyManager::increaseTimeOfDay(0.5f);
	}
	if (glfwGetKey(window, GLFW_KEY_9) == GLFW_PRESS)
	{
		SkyManager::increaseTimeOfDay(-0.5f);
	}

	#ifdef DEV_MODE
	if (glfwGetKey(window, GLFW_KEY_J) == GLFW_PRESS)
	{
		if (Global::gameMainVehicle != nullptr)
		{
			//Global::gamePlayer->goUp();
		}
	}
	#endif


	#ifdef DEV_MODE
	if (Input::inputs.INPUT_SHOULDER && !Input::inputs.INPUT_PREVIOUS_SHOULDER)
	{
		if (Global::gameMainVehicle != nullptr)
		{
			std::fprintf(stdout, "Time of day: %f\n", SkyManager::getTimeOfDay());
			std::fprintf(stdout, "[%f, %f, %f]\n", Global::gameMainVehicle->getPosition()->x, Global::gameMainVehicle->getPosition()->y, Global::gameMainVehicle->getPosition()->z);
			//std::fprintf(stdout, "player rot = %f\n", Global::gamePlayer->getRotY());
			//std::fprintf(stdout, "cam yaw: %f,   cam pitch: %f\n", Global::gameCamera->getYaw(), Global::gameCamera->getPitch());
			std::fprintf(stdout, "\n");
		}
	}
	#endif


	float mag = sqrtf(Input::inputs.INPUT_X*Input::inputs.INPUT_X + Input::inputs.INPUT_Y*Input::inputs.INPUT_Y);
	if (mag > 1)
	{
		Input::inputs.INPUT_X = Input::inputs.INPUT_X / mag;
		Input::inputs.INPUT_Y = Input::inputs.INPUT_Y / mag;
	}

	Input::inputs.approxXLeftPrevious = Input::inputs.approxXLeft;
	Input::inputs.approxXLeft = (int)round(Input::inputs.INPUT_X);
	Input::inputs.approxYLeftPrevious = Input::inputs.approxYLeft;
	Input::inputs.approxYLeft = (int)round(Input::inputs.INPUT_Y);

	if (Input::inputs.approxXLeft != 0)
	{
		Input::inputs.MENU_X = Input::inputs.approxXLeft - Input::inputs.approxXLeftPrevious;
	}

	if (Input::inputs.approxYLeft != 0)
	{
		Input::inputs.MENU_Y = Input::inputs.approxYLeft - Input::inputs.approxYLeftPrevious;
	}
}


void Input::init()
{
	Input::inputs.uniqueVar = 1149650285; //Value that is very easy to find with a memory scan

	//load sensitivity and button mappings from external file

	std::ifstream file("Settings/CameraSensitivity.ini");
	if (!file.is_open())
	{
		std::fprintf(stdout, "Error: Cannot load file 'Settings/CameraSensitivity.ini'\n");
		file.close();
	}
	else
	{
		std::string line;

		while (!file.eof())
		{
			getline(file, line);

			char lineBuf[512];
			memcpy(lineBuf, line.c_str(), line.size()+1);

			int splitLength = 0;
			char** lineSplit = split(lineBuf, ' ', &splitLength);

			if (splitLength == 2)
			{
				if (strcmp(lineSplit[0], "Mouse_X") == 0)
				{
					mouseSensitivityX = std::stof(lineSplit[1], nullptr);
				}
				else if (strcmp(lineSplit[0], "Mouse_Y") == 0)
				{
					mouseSensitivityY = std::stof(lineSplit[1], nullptr);
				}
				else if (strcmp(lineSplit[0], "Stick_X") == 0)
				{
					stickSensitivityX = std::stof(lineSplit[1], nullptr);
				}
				else if (strcmp(lineSplit[0], "Stick_Y") == 0)
				{
					stickSensitivityY = std::stof(lineSplit[1], nullptr);
				}
				else if (strcmp(lineSplit[0], "Triggers") == 0)
				{
					triggerSensitivity = std::stof(lineSplit[1], nullptr);
				}
			}

			free(lineSplit);
		}
		file.close();
	}


	std::ifstream file2("Settings/ControllerConfig.ini");
	if (!file2.is_open())
	{
		std::fprintf(stdout, "Error: Cannot load file 'Settings/ControllerConfig.ini'\n");
		file2.close();
	}
	else
	{
		std::string line;

		while (!file2.eof())
		{
			getline(file2, line);

			char lineBuf[512];
			memcpy(lineBuf, line.c_str(), line.size()+1);

			int splitLength = 0;
			char** lineSplit = split(lineBuf, ' ', &splitLength);

			if (splitLength == 2)
			{
				if (strcmp(lineSplit[0], "A") == 0)
				{
					BUTTON_A = std::stoi(lineSplit[1], nullptr, 10);
				}
				else if (strcmp(lineSplit[0], "B") == 0)
				{
					BUTTON_B = std::stoi(lineSplit[1], nullptr, 10);
				}
				else if (strcmp(lineSplit[0], "X") == 0)
				{
					BUTTON_X = std::stoi(lineSplit[1], nullptr, 10);
				}
				else if (strcmp(lineSplit[0], "Y") == 0)
				{
					BUTTON_Y = std::stoi(lineSplit[1], nullptr, 10);
				}
				else if (strcmp(lineSplit[0], "LB") == 0)
				{
					BUTTON_LB = std::stoi(lineSplit[1], nullptr, 10);
				}
				else if (strcmp(lineSplit[0], "RB") == 0)
				{
					BUTTON_RB = std::stoi(lineSplit[1], nullptr, 10);
				}
				else if (strcmp(lineSplit[0], "Start") == 0)
				{
					BUTTON_START = std::stoi(lineSplit[1], nullptr, 10);
				}
				else if (strcmp(lineSplit[0], "Controller_ID") == 0)
				{
					int raw = std::stoi(lineSplit[1], nullptr, 10);
					switch (raw)
					{
						case 0:  CONTROLLER_ID = GLFW_JOYSTICK_1;  break;
						case 1:  CONTROLLER_ID = GLFW_JOYSTICK_2;  break;
						case 2:  CONTROLLER_ID = GLFW_JOYSTICK_3;  break;
						case 3:  CONTROLLER_ID = GLFW_JOYSTICK_4;  break;
						case 4:  CONTROLLER_ID = GLFW_JOYSTICK_5;  break;
						case 5:  CONTROLLER_ID = GLFW_JOYSTICK_6;  break;
						case 6:  CONTROLLER_ID = GLFW_JOYSTICK_7;  break;
						case 7:  CONTROLLER_ID = GLFW_JOYSTICK_8;  break;
						case 8:  CONTROLLER_ID = GLFW_JOYSTICK_9;  break;
						case 9:  CONTROLLER_ID = GLFW_JOYSTICK_10; break;
						case 10: CONTROLLER_ID = GLFW_JOYSTICK_11; break;
						case 11: CONTROLLER_ID = GLFW_JOYSTICK_12; break;
						case 12: CONTROLLER_ID = GLFW_JOYSTICK_13; break;
						case 13: CONTROLLER_ID = GLFW_JOYSTICK_14; break;
						case 14: CONTROLLER_ID = GLFW_JOYSTICK_15; break;
						case 15: CONTROLLER_ID = GLFW_JOYSTICK_16; break;
						default: CONTROLLER_ID = GLFW_JOYSTICK_1;  break;
					}
				}
			}
			else if (splitLength == 4)
			{
				if (strcmp(lineSplit[0], "Stick_LX") == 0)
				{
					STICK_LX = std::stoi(lineSplit[1], nullptr, 10);
					STICK_LXDEADZONE = std::stof(lineSplit[2], nullptr);
					STICK_LX_SCALE = std::stof(lineSplit[3], nullptr);
				}
				else if (strcmp(lineSplit[0], "Stick_LY") == 0)
				{
					STICK_LY = std::stoi(lineSplit[1], nullptr, 10);
					STICK_LYDEADZONE = std::stof(lineSplit[2], nullptr);
					STICK_LY_SCALE = std::stof(lineSplit[3], nullptr);
				}
				else if (strcmp(lineSplit[0], "Stick_RX") == 0)
				{
					STICK_RX = std::stoi(lineSplit[1], nullptr, 10);
					STICK_RXDEADZONE = std::stof(lineSplit[2], nullptr);
					STICK_RX_SCALE = std::stof(lineSplit[3], nullptr);
				}
				else if (strcmp(lineSplit[0], "Stick_RY") == 0)
				{
					STICK_RY = std::stoi(lineSplit[1], nullptr, 10);
					STICK_RYDEADZONE = std::stof(lineSplit[2], nullptr);
					STICK_RY_SCALE = std::stof(lineSplit[3], nullptr);
				}
			}
			else if (splitLength == 5)
			{
				if (strcmp(lineSplit[0], "Trigger_L") == 0)
				{
					TRIGGER_L = std::stoi(lineSplit[1], nullptr, 10);
					LT_NEUTRAL = std::stof(lineSplit[2], nullptr);
					LT_MAX = std::stof(lineSplit[3], nullptr);
					TRIGGER_DEADZONE = std::stof(lineSplit[4], nullptr);
					LT_RANGE = LT_MAX - LT_NEUTRAL;
				}
				else if (strcmp(lineSplit[0], "Trigger_R") == 0)
				{
					TRIGGER_R = std::stoi(lineSplit[1], nullptr, 10);
					RT_NEUTRAL = std::stof(lineSplit[2], nullptr);
					RT_MAX = std::stof(lineSplit[3], nullptr);
					TRIGGER_DEADZONE = std::stof(lineSplit[4], nullptr);
					RT_RANGE = RT_MAX - RT_NEUTRAL;
				}
			}

			free(lineSplit);
		}
		file2.close();
	}

	glfwPollEvents();

	int present = glfwJoystickPresent(CONTROLLER_ID);
	if (present == 1)
	{
		int axesCount;
		glfwGetJoystickAxes(CONTROLLER_ID, &axesCount);
		STICK_LX  = std::min(STICK_LX,  axesCount - 1);
		STICK_LY  = std::min(STICK_LY,  axesCount - 1);
		STICK_RX  = std::min(STICK_RX,  axesCount - 1);
		STICK_RY  = std::min(STICK_RY,  axesCount - 1);
		TRIGGER_L = std::min(TRIGGER_L, axesCount - 1);
		TRIGGER_R = std::min(TRIGGER_R, axesCount - 1);

		int buttonCount;
		glfwGetJoystickButtons(CONTROLLER_ID, &buttonCount);
		BUTTON_A     = std::min(BUTTON_A,     buttonCount - 1);
		BUTTON_B     = std::min(BUTTON_B,     buttonCount - 1);
		BUTTON_X     = std::min(BUTTON_X,     buttonCount - 1);
		BUTTON_Y     = std::min(BUTTON_Y,     buttonCount - 1);
		BUTTON_LB    = std::min(BUTTON_LB,    buttonCount - 1);
		BUTTON_RB    = std::min(BUTTON_RB,    buttonCount - 1);
		BUTTON_START = std::min(BUTTON_START, buttonCount - 1);
	}
}