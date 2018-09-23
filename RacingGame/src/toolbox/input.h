#ifndef INPUT_H
#define INPUT_H

struct InputStruct
{
	volatile int uniqueVar;

	volatile float INPUT_X;
	volatile float INPUT_Y;
	volatile float INPUT_X2;
	volatile float INPUT_Y2;
	volatile float INPUT_R2;
	volatile float INPUT_L2;

	volatile bool INPUT_JUMP;
	volatile bool INPUT_ACTION;
	volatile bool INPUT_ACTION2;
	volatile bool INPUT_SHOULDER;
	volatile bool INPUT_SELECT;
	volatile bool INPUT_SPECIAL;
	volatile bool INPUT_START;

	volatile float INPUT_PREVIOUS_X;
	volatile float INPUT_PREVIOUS_Y;
	volatile float INPUT_PREVIOUS_X2;
	volatile float INPUT_PREVIOUS_Y2;
	volatile float INPUT_PREVIOUS_R2;
	volatile float INPUT_PREVIOUS_L2;

	volatile bool INPUT_PREVIOUS_JUMP;
	volatile bool INPUT_PREVIOUS_ACTION;
	volatile bool INPUT_PREVIOUS_ACTION2;
	volatile bool INPUT_PREVIOUS_SHOULDER;
	volatile bool INPUT_PREVIOUS_SPECIAL;
	volatile bool INPUT_PREVIOUS_START;

	volatile int MENU_X;
	volatile int MENU_Y;

	volatile int approxXLeft;
	volatile int approxXLeftPrevious;
	volatile int approxYLeft;
	volatile int approxYLeftPrevious;
};

class Input
{
public:
	static InputStruct inputs;

	static void init();

	static void pollInputs();
};

#endif