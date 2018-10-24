#ifndef CAR_H
#define CAR_H

class TexturedModel;
class Triangle3D;
class ParticleTexture;
class Source;

#include <list>
#include "entity.h"
#include "../toolbox/vector.h"

struct EngineExhaust
{
	int textureID;
	float posUp;
	float posAt;
	float posSide;
	float lengthScale;
	float size;
};

class Car : public Entity
{
private:
	static std::list<TexturedModel*> models  [30];
	static std::list<EngineExhaust>  exhausts[30];
	int vehicleID;

	const float VEL_SLOWEST = 0.0006f; //Slowest that the car can move
	const float FLOOR_OFFSET = 0.1f; //How much you stay away from the floor

	const float smoothTransitionThreshold = 0.6f; //This should not change bewteen racers
	const float surfaceTension = 10.0f;     //To not fly off the track
	const float hitWallTimePunish = 0.125f; //How long you can't move after hitting a wall

	const float camAngleLookdown = 0.2f;    //How much the camera looks down
	const float camRadius        = 7.5;     //Camera radius at zero speed
	const float camRadiusAdjust  = 0.0005f; //How much the camera zooms out due to speed
	const float camFovAdjust     = 0.05f;   //
	const float camHeightOffset  = 2.5f;    //Point above the vehicle that the camera focuses on
	const float camSmoothness    = 5.0f;    //Smoothes currNormSmooth and camDirSmooth

	const float terminalSpeed      = 298.0f; //Normal speed cap
	const float terminalAccelGas   = 0.45f;  //How fast you accelerate towards terminal speed
	const float terminalAccelBrake = 1.0f;   //How fast you accelerate towards 0 while braking
	const float terminalAccelCoast = 0.1f;   //How fast you accelerate towards 0 while coasting

	const float turnSpeed  = 1.38f;
	const float turnPunish = 0.01f;  //How much you slow down due to turning

	const float diveSpeed  = 4.5f;
	const float divePunish = 2.5f;

	const float strafePercentage     = 0.2f;  //Percent that strafing goes sideways
	const float strafeTerminalPunish = 0.95f; //Percent that strafing lowers the terminal speed

	      float slipTimer          = 0.0f;
		  float slipAccumulated    = 0.0f;  //How much slip you have accumulated from jerking the wheel
	const float slipTimerMax       = 0.75f; //What the slipTimer gets set to when you trigger a slip
	const float slipThreshold      = 4.5f;  //Point at which you trigger a slip
	      float slipAngle          = 0.0f;  //Current angle that you are facing during the slip
	      float slipAngleTarget    = 0.0f;  //What the slip angle is approaching
	const float slipAngleAccel     = 5.0f;  //How fast the slip angle approaches the target
	const float slipAngleMax       = 40.0f; //Max that the angle can slip by
	const float slipPower          = 1.5f;  //How much you turn when you slip turn
	const float slipNegativePower  = 1.1f;  //Multiplier for when you lose speed during slipping
	const float slipPositivePower  = 0.75f; //Multiplier for when you gain speed during slipping
	      float slipTimerRight     = 0.0f;  //How long you have been slipping to the right for
		  float slipTimerLeft      = 0.0f;  //How long you have been slipping to the left for
	const float slipTimerThreshold = 0.5f;  //Time threshold for when you start loosing speed vs gaining it during slipping

	const float spinTimeMax   = 1.65f; //How long the spin attack lasts
	const float spinTimeDelay = 2.5f;  //How long until you can do another spin attack
	      float spinTimer     = 0.0f;

	      float sideAttackTimer   = 0.0f;
		  float sideAttackDir     = 1.0f;    //Direction of the side attack
	const float sideAttackTimeMax = 0.1666f; //How long you side attack for
	const float sideAttackSpeed   = 150.0f;  //How fast you move during the attack at the beginning

	const float boostSpeed        = 462.0f; //Max speed when boosting
	const float boostKick         = 0.7f;   //Initial speed addition you get from starting a boost
	const float boostDuration     = 1.0f;   //How long you boost for
	const float boostDelayMax     = 1.15f;  //Time until you can boost again
	      float boostDelayTimer   = 0.0f;
	const float boostHealthPunish = 0.2f;   //How much health you loose for a boost

	      float camAngleAdditionalLookdown                 = 0.0f; //Additional lookdown due to driving through a steep concave or convex area
	const float camAngleAdditionalLookdownSmoothness       = 0.4f; //How fast the lookdown approaches the target
	      float camAngleAdditionalLookdownTarget           = 0.0f;
	const float camAngleAdditionalLookdownTargetSmoothness = 2.5f; //How fast the target approaches 0
	const float camAngleAdditionalLookdownMax              = 0.3f; //Max value of the lookdown
	const float camAngleAdditionalLookdownScale            = 0.005f; //How much the target gets added to by each new triangle collision

	      float health   = 1.0f;
	const float healRate = 0.666f; //Health per second that you recover while in health pads
	const float hitWallHealthPunish = 0.3f; //How much health you loose when hitting a wall perpendicularly at max speed

	const float weight = 1.0f; //Weight of the vehicle. Affects the recoil from colliding with another vehicle

	const float gravityForce = 180.0f;

	Vector3f vel; //Velocity that the car has
	Vector3f currNorm;       //Normal vector of the ground
	Vector3f currNormSmooth; //'Up' vector to be used in orienting the camera
	bool onPlane;
	float inAirTimer = 0.0f;
	Triangle3D* currentTriangle = nullptr; //The triangle that the vehicle is currently on
	float canMoveTimer = 0.0f;
	float deadTimer = -1.0f; //Negative = alive
	float fallOutTimer = -1.0f;

	//Variables used solely for audio
	Source* sourceEngine = nullptr;
	Source* sourceStrafe = nullptr;
	float sourceStrafeTimer = 0; //How long you have been strafing for
	Source* sourceSlipSlowdown = nullptr;
	Source* sourceDanger = nullptr;
	Source* sourceHeal = nullptr;

	Vector3f camDir;       //Direction that the car wants the camera to be facing
	Vector3f camDirSmooth; //The actual direction the camera will be facing

	Vector3f camDeathPosition; //Position that the camera stays at when you die

	float exhaustLength = 0.0f;         //Length of the echaust coming out of the engines (scale)
	float exhaustLengthTarget = 0.0f;   //What the exhaustLength is trying to be
	const float exhaustApproach = 5.0f; //How fast the exhaustLength approaches exhaustLengthTarget

	//Variables to keep track of what lap you're on
	int lastCheckpointID = 0;
	int currentLap = 0;
	int lapDistance = 0; //Accumulator for how many checkpoints youve passed through (in the right direction)

	bool  inputGas;
	bool  inputBrake;
	bool  inputBoost;
	bool  inputAttackSpin;
	bool  inputAttackSide;
	float inputWheel;
	float inputWheelJerk;
	float inputDive;
	float inputL;
	float inputR;

	bool inputGasPrevious;
	bool inputBrakePrevious;
	bool inputBoostPrevious;
	bool inputAttackSpinPrevious;
	bool inputAttackSidePrevious;

	void setInputs();

	void createEngineParticles(Vector3f* initPos, Vector3f* endPos, float initialScale, int count, int textureIndex);

	void checkpointTest();

public:
	Car();
	Car(int vehicleID, float x, float y, float z, float xDir, float yDir, float zDir);

	void step();

	void giveMeABoost();

	void setVelocity(float xVel, float yVel, float zVel);

	void setLapDistance(int newDistance);

	void setCanMoveTimer(float newTimer);

	std::list<TexturedModel*>* getModels();

	void loadVehicleInfo();

	static void deleteStaticModels();

	bool isVehicle();
};
#endif