#include <glad/glad.h>

#include "entity.h"
#include "../models/models.h"
#include "../toolbox/vector.h"
#include "car.h"
#include "../renderEngine/renderEngine.h"
#include "../objLoader/objLoader.h"
#include "../engineTester/main.h"
#include "../entities/camera.h"
#include "../collision/collisionchecker.h"
#include "../collision/triangle3d.h"
#include "../toolbox/maths.h"
#include "../audio/audioplayer.h"
#include "../particles/particle.h"
#include "../particles/particleresources.h"
#include "../toolbox/input.h"
#include "../fontMeshCreator/guinumber.h"
#include "../toolbox/split.h"
#include "../audio/source.h"
#include "checkpoint.h"
#include "../guis/guimanager.h"

#include <list>
#include <iostream>
#include <algorithm>
#include <fstream>

std::list<TexturedModel*> Car::models[30];
std::list<EngineExhaust> Car::exhausts[30];
MachineStats Car::machineBaseStats[30];

extern float dt;

Car::Car()
{

}

Car::Car(int vehicleID, float x, float y, float z, float xDir, float yDir, float zDir)
{
	this->vehicleID = vehicleID;
	position.set(x, y, z);
	vel.set(xDir, yDir, zDir);
	currNorm.set(0, 1, 0);
	currNormSmooth.set(0, 1, 0);
	onPlane = false;
	visible = true;

	camDirSmooth.set(0, 0, -1);
	camDir      .set(0, 0, -1);

	loadVehicleInfo();
	canMoveTimer = 1.0f;
}

Car::~Car()
{
	if (sourceEngine       != nullptr) sourceEngine->stop();
	if (sourceStrafe       != nullptr) sourceStrafe->stop();
	if (sourceSlipSlowdown != nullptr) sourceSlipSlowdown->stop();
	if (sourceDanger       != nullptr) sourceDanger->stop();
	if (sourceHeal         != nullptr) sourceHeal->stop();
}

void Car::step()
{
	if (isDead)
	{
		return;
	}

	canMoveTimer    = std::fmaxf(0.0f, canMoveTimer    - dt);
	boostDelayTimer = std::fmaxf(0.0f, boostDelayTimer - dt);
	slipTimer       = std::fmaxf(0.0f, slipTimer       - dt);
	spinTimer       = std::fmaxf(0.0f, spinTimer       - dt);
	sideAttackTimer = std::fmaxf(0.0f, sideAttackTimer - dt);

	if (deadTimer >= 0.0f)
	{
		deadTimer += dt;
		if (deadTimer > 4.0f && !isDead)
		{
			explode();
		}
	}

	if (fallOutTimer >= 0.0f)
	{
		fallOutTimer += dt;
		if (fallOutTimer > 4.0f && !isDead)
		{
			explode();
		}
	}

	setInputs();

	//Vector3f prevPos(&position);

	//Spin attack
	if (onPlane && inputAttackSpin && !inputAttackSpinPrevious)
	{
		if (spinTimer == 0.0f)
		{
			spinTimer = spinTimeDelay;
		}
	}

	//Slipping
	if (Input::inputs.INPUT_L2 > 0.5f && 
		Input::inputs.INPUT_R2 > 0.5f)
	{
		slipTimer = slipTimerMax;
	}
	if (onPlane && currentTriangle->isSlippery())
	{
		slipTimer = slipTimerMax;
	}

	slipAccumulated += 100*std::fabsf(inputWheelJerk)*dt*(vel.length()/terminalSpeed);
	if (slipTimer > 0.0f)
	{
		slipAccumulated += 25*std::fabsf(inputWheel)*dt*(vel.length()/terminalSpeed);
	}

	slipAccumulated = Maths::approach(slipAccumulated, 0.0f, 5.0f, dt);

	if (slipAccumulated > slipThreshold)
	{
		slipTimer = slipTimerMax;
	}

	if (!onPlane)
	{
		slipTimer = 0.0f;
		slipAccumulated = 0.0f;
	}

	slipAngleTarget = std::fminf( slipAngleMax, slipAngleMax*inputWheel*(vel.length()/terminalSpeed));
	slipAngleTarget = std::fmaxf(-slipAngleMax, slipAngleTarget);
	if (slipTimer == 0.0f)
	{
		slipAngleTarget = 0.0f;
	}


	slipAngle = Maths::approach(slipAngle, slipAngleTarget, slipAngleAccel, dt);

	if (fabsf(slipAngle) <= 10.0f)
	{
		slipTimerRight = 0;
		slipTimerLeft  = 0;
	}
	else if (slipAngle > 0)
	{
		slipTimerRight += dt;
		slipTimerLeft   = 0;
	}
	else
	{
		slipTimerLeft += dt;
		slipTimerRight = 0;
	}

	float slipPunishScale = std::fabsf((slipAngle*slipAngle) / (slipAngleMax*slipAngleMax));
	//if (slipPunishScale < 0.75f) //TODO: fix this so that you cant gain insane speed by holding the magic angle
	{
		//slipPunishScale = -slipPunishScale;
	}

	if (slipTimerRight+slipTimerLeft < slipTimerThreshold)
	{
		slipPunishScale = -slipPunishScale*slipPositivePower;
	}
	else
	{
		slipPunishScale = slipPunishScale*slipNegativePower;
	}

	if (slipPunishScale > 0)
	{
		slipPunishScale = 0;
	}

	if (!inputGas && slipPunishScale < 0)
	{
		slipPunishScale = 0;
	}
	
	//turn due to slipping
	float xIn2 = -slipPower*(slipAngle/slipAngleMax)*dt;
	vel.scale(1 - slipPunishScale*(std::fabsf((slipAngle/slipAngleMax)*dt))); //slow down or speed up
	float u2 = currNorm.x;
	float v2 = currNorm.y;
	float w2 = currNorm.z;
	float x2 = vel.x;
	float y2 = vel.y;
	float z2 = vel.z;
	float result2[3];
	Maths::rotatePoint(result2, 0, 0, 0, u2, v2, w2, x2, y2, z2, xIn2);
	vel.x = result2[0];
	vel.y = result2[1];
	vel.z = result2[2];
	

	//Strafing
	Vector3f upDir5(&currNorm);
	Vector3f atDir5(&vel);
	Vector3f rightDir5 = atDir5.cross(&upDir5);
	upDir5 = rightDir5.cross(&atDir5);

	//Vector3f perpen5 = vel.cross(&currNorm);

	float u5 = upDir5.x;
	float v5 = upDir5.y;
	float w5 = upDir5.z;
	float x5 = atDir5.x;
	float y5 = atDir5.y;
	float z5 = atDir5.z;
	float buf5[3];
	Maths::rotatePoint(buf5, 0, 0, 0, u5, v5, w5, x5, y5, z5, Maths::toRadians(-slipAngle));
	atDir5.set(buf5[0], buf5[1], buf5[2]);

	u5 = atDir5.x;
	v5 = atDir5.y;
	w5 = atDir5.z;
	x5 = rightDir5.x;
	y5 = rightDir5.y;
	z5 = rightDir5.z;
	Maths::rotatePoint(buf5, 0, 0, 0, u5, v5, w5, x5, y5, z5, Maths::toRadians(0));
	Vector3f strafeDir(buf5[0], buf5[1], buf5[2]);
	strafeDir.normalize();

	Vector3f strafeRAmount(&strafeDir);
	Vector3f strafeLAmount(&strafeDir);
	strafeRAmount.scale( inputR);
	strafeLAmount.scale(-inputL);
	Vector3f totalStrafe = (strafeRAmount + strafeLAmount);
	totalStrafe.scale(strafePercentage);
	totalStrafe.scale(vel.length());
	float strafeScale = std::fabsf(inputR-inputL);
	float strafePunish = 1.0f - strafeScale*(1.0f - strafeTerminalPunish);

	//Side attack
	if (onPlane && inputAttackSide && !inputAttackSidePrevious)
	{
		if (sideAttackTimer == 0.0f && fabsf(inputWheel) > 0.2f && vel.length() > 5.0f)
		{
			sideAttackTimer = sideAttackTimeMax;
			sideAttackDir = -1.0f;
			if (inputWheel > 0)
			{
				sideAttackDir = 1.0f;
			}
		}
	}

	Vector3f sideAttackVel(&strafeDir);
	if (sideAttackTimer > 0)
	{
		sideAttackVel.scale(sideAttackDir*sideAttackSpeed*(sideAttackTimer/sideAttackTimeMax));
	}
	else
	{
		sideAttackVel.set(0, 0, 0);
	}


	//Boost
	if (inputBoost && !inputBoostPrevious && currentLap >= 1)
	{
		if (boostDelayTimer == 0.0f && onPlane && health > 0.0f)
		{
			AudioPlayer::play(boostIndex, getPosition());

			float boostRatio = 1.0f;
			if (health < boostHealthPunish)
			{
				boostRatio = health/boostHealthPunish;
				health = 0.0f;
			}
			else
			{
				health -= boostHealthPunish;
			}

			boostDelayTimer = boostDelayMax*boostRatio;

			float oldSpeed = vel.length();
			float newSpeed = oldSpeed + boostKick*boostRatio*(boostSpeed - oldSpeed);

			if (newSpeed > oldSpeed)
			{
				float ratio = newSpeed/oldSpeed;
				vel.x*=ratio;
				vel.y*=ratio;
				vel.z*=ratio;
			}
		}
	}

	if (onPlane && currentTriangle->isMiniBoost())
	{
		if (boostDelayTimer == 0.0f)
		{
			AudioPlayer::play(0, getPosition());
		}
		boostDelayTimer = (boostDuration)*.1f + (boostDelayMax-boostDuration);
	}

	float brakePunish = 1.0f;
	if (onPlane && currentTriangle->isBrake())
	{
		brakePunish = 0.5f;
	}

	float spinPunish = 1.0f;
	if (spinTimer > spinTimeDelay-spinTimeMax)
	{
		spinPunish = 0.5f;
	}

	//Accelerate/Brake/Coast
	if (inputGas && onPlane)
	{
		float oldSpeed = vel.length();

		float terminal = terminalSpeed;
		if (boostDelayTimer > boostDelayMax-boostDuration)
		{
			terminal = boostSpeed;
		}

		float newSpeed = Maths::approach(oldSpeed, terminal*strafePunish*brakePunish*spinPunish, terminalAccelGas, dt);

		float ratio = newSpeed/oldSpeed;
		vel.x*=ratio;
		vel.y*=ratio;
		vel.z*=ratio;
	}
	else if (inputBrake && onPlane)
	{
		float oldSpeed = vel.length();
		float newSpeed = Maths::approach(oldSpeed, 0, (terminalAccelBrake/brakePunish)/spinPunish, dt);
		if (newSpeed <= VEL_SLOWEST)
		{
			vel.normalize();
			vel.scale(VEL_SLOWEST);
		}
		else
		{
			float ratio = newSpeed/oldSpeed;
			vel.x*=ratio;
			vel.y*=ratio;
			vel.z*=ratio;
		}
	}
	else
	{
		float oldSpeed = vel.length();
		float newSpeed = Maths::approach(oldSpeed, 0, (terminalAccelCoast/brakePunish)/spinPunish, dt);
		if (newSpeed <= VEL_SLOWEST)
		{
			vel.normalize();
			vel.scale(VEL_SLOWEST);
		}
		else
		{
			float ratio = newSpeed/oldSpeed;
			vel.x*=ratio;
			vel.y*=ratio;
			vel.z*=ratio;
		}
	}

	//Turning
	float xIn = (-inputWheel*turnSpeed)*dt;
	vel.scale(1 - turnPunish*(std::fabsf(xIn))); //slow down when turning
	float u = currNorm.x;
	float v = currNorm.y;
	float w = currNorm.z;
	float x = vel.x;
	float y = vel.y;
	float z = vel.z;
	float result[3];
	Maths::rotatePoint(result, 0, 0, 0, u, v, w, x, y, z, xIn);
	vel.x = result[0];
	vel.y = result[1];
	vel.z = result[2];

	if (onPlane)
	{
		inAirTimer = 0.0f;
	}
	else
	{
		inAirTimer+=dt;
	}

	//Nose diving or pulling up
	if (!onPlane && inAirTimer > 0.25f)
	{
		float yIn = (inputDive*diveSpeed)*dt;

		if (vel.y > 0)
		{
			yIn = fminf(yIn, 0);
		}

		Vector3f perpen = vel.cross(&currNorm);

		float u3 = perpen.x;
		float v3 = perpen.y;
		float w3 = perpen.z;
		float x3 = vel.x;
		float y3 = vel.y;
		float z3 = vel.z;
		float buf[3];
		Maths::rotatePoint(buf, 0, 0, 0, u3, v3, w3, x3, y3, z3, yIn);

		float oldAngle = Maths::toDegrees(atan2f(-vel.z,  vel.x));
		float newAngle = Maths::toDegrees(atan2f(-buf[2], buf[0]));

		if (fabsf(Maths::compareTwoAngles(newAngle, oldAngle)) < 90)
		{
			if (vel.y < 0 && buf[1] > 0)
			{
				vel.x = buf[0];
				vel.y = 0;
				vel.z = buf[2];
			}
			else
			{
				vel.x = buf[0];
				vel.y = buf[1];
				vel.z = buf[2];
			}

			if (inputDive > 0)
			{
				vel.scale(1 - inputDive*divePunish*dt); //slow down when diving
			}
		}
	}
	//Prevent angle from being too steep
	if (!onPlane)
	{
		if (vel.y < 0)
		{
			float angle = atan2f(vel.y, sqrtf(vel.x*vel.x+vel.z*vel.z));
			if (angle < Maths::toRadians(-70))
			{
				Vector3f perpen = vel.cross(&currNorm);

				float u3 = perpen.x;
				float v3 = perpen.y;
				float w3 = perpen.z;
				float x3 = vel.x;
				float y3 = vel.y;
				float z3 = vel.z;
				float buf[3];
				Maths::rotatePoint(buf, 0, 0, 0, u3, v3, w3, x3, y3, z3, -(angle+Maths::toRadians(70)));
				vel.x = buf[0];
				vel.y = buf[1];
				vel.z = buf[2];
			}
		}
	}


	Vector3f overallVel = vel + totalStrafe + sideAttackVel;

	//speed before adjusting
	float originalSpeed = vel.length();

	CollisionChecker::setCheckPlayer();
	if (CollisionChecker::checkCollision(getX(), getY(), getZ(), getX()+overallVel.x*dt, getY()+overallVel.y*dt, getZ()+overallVel.z*dt))
	{
		Vector3f* colNormal = &CollisionChecker::getCollideTriangle()->normal;

		if (onPlane  == false) //Air to ground
		{
			if (CollisionChecker::getCollideTriangle()->isWall())
			{
				Vector3f newDirection = Maths::projectOntoPlane(&vel, colNormal);
				if (newDirection.lengthSquared() != 0)
				{
					if (newDirection.length() < VEL_SLOWEST)
					{
						newDirection.normalize();
						newDirection.scale(VEL_SLOWEST);
					}
					vel.set(&newDirection);
				}

				setPosition(CollisionChecker::getCollidePosition());
				increasePosition(colNormal->x*FLOOR_OFFSET, colNormal->y*FLOOR_OFFSET, colNormal->z*FLOOR_OFFSET);
			}
			else
			{
				currentTriangle = CollisionChecker::getCollideTriangle();
				Vector3f newDirection = Maths::projectOntoPlane(&vel, colNormal);
				if (newDirection.lengthSquared() != 0)
				{
					if (newDirection.length() < VEL_SLOWEST)
					{
						newDirection.normalize();
						newDirection.scale(VEL_SLOWEST);
					}
					vel.set(&newDirection);
				}

				setPosition(CollisionChecker::getCollidePosition());
				increasePosition(colNormal->x*FLOOR_OFFSET, colNormal->y*FLOOR_OFFSET, colNormal->z*FLOOR_OFFSET);

				currNorm.set(colNormal);
			}
		}
		else //Ground to a different triangle
		{
			//check if you can smoothly transition from previous triangle to this triangle
			float dotProduct = currNorm.dot(colNormal);
			if (dotProduct < smoothTransitionThreshold || CollisionChecker::getCollideTriangle()->isWall())
			{
				float velBefore = vel.length();

				Vector3f bounceVec = Maths::bounceVector(&overallVel, colNormal, 1.0f);
				Vector3f newDir = bounceVec + bounceVec + overallVel;
				newDir.normalize();
				Vector3f proj = Maths::projectOntoPlane(&overallVel, colNormal);
				float newSpeed = std::max(proj.length()*0.75f, VEL_SLOWEST);
				newDir.scale(newSpeed);
				vel.set(&newDir);

				Vector3f overallVelNorm(&overallVel);
				overallVelNorm.normalize();
				float speedFactor = overallVel.length()/terminalSpeed;
				float angleFactor = std::fabsf(overallVelNorm.dot(colNormal));
				health -= hitWallHealthPunish*angleFactor*speedFactor;

				if (health < 0.0f)
				{
					vel.normalize();
					vel.scale(std::fminf(velBefore*1.2f, 700));
				}

				canMoveTimer = hitWallTimePunish;
				AudioPlayer::play(4, getPosition());

				increasePosition(colNormal->x*FLOOR_OFFSET, colNormal->y*FLOOR_OFFSET, colNormal->z*FLOOR_OFFSET);
			}
			else
			{
				currentTriangle = CollisionChecker::getCollideTriangle();
				Vector3f newDirection = Maths::projectOntoPlane(&vel, colNormal);
				if (newDirection.lengthSquared() != 0)
				{
					//Cam angle additional
					Vector3f perpen = Maths::calcThirdAxis(&vel, &currNorm);
					Vector3f coordsFlat = Maths::coordinatesRelativeToBasis(&vel, &currNorm, &perpen, &newDirection);
					camAngleAdditionalLookdownTarget -= coordsFlat.y*camAngleAdditionalLookdownScale;

					newDirection.normalize();
					newDirection.x*=originalSpeed;
					newDirection.y*=originalSpeed;
					newDirection.z*=originalSpeed;
					if (newDirection.length() < VEL_SLOWEST)
					{
						newDirection.normalize();
						newDirection.scale(VEL_SLOWEST);
					}
					vel.set(&newDirection);
				}

				currNorm.set(colNormal);

				Vector3f newPosition(CollisionChecker::getCollidePosition());
				//newPosition.add(colNormal->x*FLOOR_OFFSET, colNormal->y*FLOOR_OFFSET, colNormal->z*FLOOR_OFFSET); //not sure if it would be better to calculate new positoin like this instead
				Vector3f travelDelta = newPosition - position;

				setPosition(CollisionChecker::getCollidePosition());
				increasePosition(colNormal->x*FLOOR_OFFSET, colNormal->y*FLOOR_OFFSET, colNormal->z*FLOOR_OFFSET);

				float distanceTraveled = travelDelta.length();
				float distanceRemaining = (overallVel.length()*dt)-distanceTraveled;

				Vector3f nextVel(&vel);
				nextVel.normalize();
				nextVel.scale(distanceRemaining);

				while (distanceRemaining > 0.0f)
				{
					CollisionChecker::setCheckPlayer();
					if (CollisionChecker::checkCollision(getX(), getY(), getZ(), getX()+nextVel.x, getY()+nextVel.y, getZ()+nextVel.z))
					{
						colNormal = &CollisionChecker::getCollideTriangle()->normal;

						//check if you can smoothly transition from previous triangle to this triangle
						dotProduct = currNorm.dot(colNormal);
						if (dotProduct < smoothTransitionThreshold || CollisionChecker::getCollideTriangle()->isWall())
						{
							float velBefore = vel.length();

							Vector3f bounceVec = Maths::bounceVector(&overallVel, colNormal, 1.0f);
							Vector3f newDir = bounceVec + bounceVec + overallVel;
							newDir.normalize();
							Vector3f proj = Maths::projectOntoPlane(&overallVel, colNormal);
							float newSpeed = std::max(proj.length()*0.75f, VEL_SLOWEST);
							newDir.scale(newSpeed);
							vel.set(&newDir);

							Vector3f overallVelNorm(&overallVel);
							overallVelNorm.normalize();
							float speedFactor = overallVel.length()/terminalSpeed;
							float angleFactor = std::fabsf(overallVelNorm.dot(colNormal));
							health -= hitWallHealthPunish*angleFactor*speedFactor;

							if (health < 0.0f)
							{
								vel.normalize();
								vel.scale(std::fminf(velBefore*1.2f, 700));
							}

							canMoveTimer = hitWallTimePunish;
							AudioPlayer::play(4, getPosition());

							increasePosition(colNormal->x*FLOOR_OFFSET, colNormal->y*FLOOR_OFFSET, colNormal->z*FLOOR_OFFSET);

							distanceRemaining = 0.0f;
						}
						else
						{
							currentTriangle = CollisionChecker::getCollideTriangle();
							newDirection = Maths::projectOntoPlane(&nextVel, colNormal);
							if (newDirection.lengthSquared() != 0)
							{
								//Cam angle additional
								Vector3f perpen = Maths::calcThirdAxis(&nextVel, &currNorm);
								Vector3f coordsFlat = Maths::coordinatesRelativeToBasis(&nextVel, &currNorm, &perpen, &newDirection);
								camAngleAdditionalLookdownTarget -= coordsFlat.y*camAngleAdditionalLookdownScale;

								newDirection.normalize();
								newDirection.x*=originalSpeed;
								newDirection.y*=originalSpeed;
								newDirection.z*=originalSpeed;
								if (newDirection.length() < VEL_SLOWEST)
								{
									newDirection.normalize();
									newDirection.scale(VEL_SLOWEST);
								}
								vel.set(&newDirection);
							}

							currNorm.set(colNormal);

							newPosition.set(CollisionChecker::getCollidePosition());
							travelDelta = newPosition - position;


							setPosition(CollisionChecker::getCollidePosition());
							increasePosition(colNormal->x*FLOOR_OFFSET, colNormal->y*FLOOR_OFFSET, colNormal->z*FLOOR_OFFSET);

							distanceTraveled = travelDelta.length();
							distanceRemaining = distanceRemaining-distanceTraveled;

							nextVel.set(&vel);
							nextVel.normalize();
							nextVel.scale(distanceRemaining);
						}
					}
					else //no more collisions, travel the remaining distance
					{
						increasePosition(nextVel.x, nextVel.y, nextVel.z);
						distanceRemaining = 0.0f;
					}
				}
			}
		}
		
		onPlane = true;
	}
	else //No initial collision
	{
		increasePosition(overallVel.x*dt, overallVel.y*dt, overallVel.z*dt);

		bool checkPassed = false;
		CollisionChecker::setCheckPlayer();
		if (onPlane)
		{
			checkPassed = CollisionChecker::checkCollision(getX(), getY(), getZ(), getX() - currNorm.x*surfaceTension, getY() - currNorm.y*surfaceTension, getZ() - currNorm.z*surfaceTension);
		}
		if (checkPassed)
		{
			float dotProduct = currNorm.dot(&(CollisionChecker::getCollideTriangle()->normal));

			if (dotProduct < smoothTransitionThreshold || CollisionChecker::getCollideTriangle()->isWall()) //It's a wall, pretend the collision check didn't see it
			{
				Vector3f perpen = Maths::calcThirdAxis(&vel, &currNorm);
				Vector3f coordsFlat = Maths::coordinatesRelativeToBasis(&vel, &currNorm, &perpen, &(CollisionChecker::getCollideTriangle()->normal));

				if (coordsFlat.x > 0) //Only ignore walls that are cliffs, not walls
				{
					checkPassed = false;
				}
			}
		}
			
		if (checkPassed)
		{
			Vector3f* colNormal = &CollisionChecker::getCollideTriangle()->normal;

			float dotProduct = currNorm.dot(&(CollisionChecker::getCollideTriangle()->normal));
			if (dotProduct < smoothTransitionThreshold || CollisionChecker::getCollideTriangle()->isWall())
			{
				CollisionChecker::falseAlarm();

				float velBefore = vel.length();

				Vector3f bounceVec = Maths::bounceVector(&overallVel, colNormal, 1.0f);
				Vector3f newDir = bounceVec + bounceVec + overallVel;
				newDir.normalize();
				Vector3f proj = Maths::projectOntoPlane(&overallVel, colNormal);
				float newSpeed = std::max(proj.length()*0.75f, VEL_SLOWEST);
				newDir.scale(newSpeed);
				vel.set(&newDir);

				Vector3f overallVelNorm(&overallVel);
				overallVelNorm.normalize();
				float speedFactor = overallVel.length()/terminalSpeed;
				float angleFactor = std::fabsf(overallVelNorm.dot(colNormal));
				health -= hitWallHealthPunish*angleFactor*speedFactor;

				if (health < 0.0f)
				{
					vel.normalize();
					vel.scale(std::fminf(velBefore*1.2f, 700));
				}

				canMoveTimer = hitWallTimePunish;
				AudioPlayer::play(4, getPosition());

				increasePosition(colNormal->x*FLOOR_OFFSET, colNormal->y*FLOOR_OFFSET, colNormal->z*FLOOR_OFFSET);
			}
			else
			{
				currentTriangle = CollisionChecker::getCollideTriangle();
				Vector3f* normal = &CollisionChecker::getCollideTriangle()->normal;

				setPosition(CollisionChecker::getCollidePosition());
				increasePosition(normal->x*FLOOR_OFFSET, normal->y*FLOOR_OFFSET, normal->z*FLOOR_OFFSET);
				
				//speed before adjusting
				float speed = vel.length();
				
				Vector3f newDirection = Maths::projectOntoPlane(&vel, normal);

				//Cam angle additional
				Vector3f perpen = Maths::calcThirdAxis(&vel, &currNorm);
				Vector3f coordsFlat = Maths::coordinatesRelativeToBasis(&vel, &currNorm, &perpen, &newDirection);
				camAngleAdditionalLookdownTarget -= coordsFlat.y*camAngleAdditionalLookdownScale;

				newDirection.normalize();
				newDirection.x*=speed;
				newDirection.y*=speed;
				newDirection.z*=speed;
				if (newDirection.length() < VEL_SLOWEST)
				{
					newDirection.normalize();
					newDirection.scale(VEL_SLOWEST);
				}
				vel.set(&newDirection);
				
				currNorm.set(normal);
				onPlane = true;
			}
		}
		else
		{
			CollisionChecker::falseAlarm();
			onPlane = false;

			Vector3f up(0, 1, 0);

			Vector3f diff2 = up - currNorm;
			diff2.scale(camSmoothness*dt);
			Vector3f newDir(currNorm);
			newDir = newDir + diff2;
			newDir.normalize();
			currNorm.set(&newDir);

			newDir.scale(-gravityForce*dt);

			if (Global::raceStartTimer < 0)
			{
				vel = vel + newDir;
			}
			else
			{
				if (CollisionChecker::checkCollision(getX(), getY(), getZ(), getX(), getY() - 20, getZ()))
				{
					currentTriangle = CollisionChecker::getCollideTriangle();
					onPlane = true;
					currNorm.set(0, 1, 0);
				}
			}
		}
	}

	camDir.set(&vel);
	camDir.normalize();

	camAngleAdditionalLookdownTarget = std::fmaxf(camAngleAdditionalLookdownTarget, -camAngleAdditionalLookdownMax);
	camAngleAdditionalLookdownTarget = std::fminf(camAngleAdditionalLookdownTarget,  camAngleAdditionalLookdownMax);

	camAngleAdditionalLookdown       = Maths::approach(camAngleAdditionalLookdown, camAngleAdditionalLookdownTarget, camAngleAdditionalLookdownSmoothness, dt);
	camAngleAdditionalLookdownTarget = Maths::approach(camAngleAdditionalLookdownTarget, 0, camAngleAdditionalLookdownSmoothness, dt);


	Vector3f diff2 = camDir - camDirSmooth;
	diff2.scale(camSmoothness*dt);
	Vector3f newDir(camDirSmooth);
	newDir = newDir + diff2;
	newDir.normalize();
	camDirSmooth.set(&newDir);


	Vector3f diff3 = currNorm - currNormSmooth;
	diff3.scale(camSmoothness*dt);
	Vector3f newDir2(currNormSmooth);
	newDir2 = newDir2 + diff3;
	newDir2.normalize();
	currNormSmooth.set(&newDir2);




	//animating the vehicle

	float spinRot = 0.0f;
	if (spinTimer > spinTimeDelay-spinTimeMax)
	{
		float tim = (spinTimer - (spinTimeDelay-spinTimeMax));
		spinRot = 500*tim*tim;
	}

	if (onPlane)
	{
		Vector3f groundSpeeds = Maths::calculatePlaneSpeed(vel.x, vel.y, vel.z, &currNorm);

		float twistAngle = Maths::toDegrees(atan2f(-groundSpeeds.z, groundSpeeds.x));
		float nX = currNorm.x;
		float nY = currNorm.y;
		float nZ = currNorm.z;
		float normHLength = sqrtf(nX*nX + nZ*nZ);
		float pitchAngle = Maths::toDegrees(atan2f(nY, normHLength));
		float yawAngle = Maths::toDegrees(atan2f(-nZ, nX));
		float diff = Maths::compareTwoAngles(twistAngle, yawAngle);

		rotX = diff - slipAngle + spinRot;
		rotY = yawAngle;
		rotZ = pitchAngle;
		rotRoll = 10*(inputL - inputR)*(vel.length()/terminalSpeed);

		if (health < 0.0f)
		{
			rotX = diff + deadTimer*720;
		}

		updateTransformationMatrix();
	}
	else
	{
		float diffX = overallVel.x;
		float diffY = overallVel.y;
		float diffZ = overallVel.z;

		float normHLength = sqrtf(diffX*diffX + diffZ*diffZ);
		float pitchAngle  = Maths::toDegrees(atan2f(diffY, normHLength));
		float yawAngle = Maths::toDegrees(atan2f(-diffZ, diffX));

		rotX = 0 + spinRot;
		rotY = yawAngle;
		rotZ = pitchAngle+90;
		rotRoll = 10*(inputL - inputR)*(vel.length()/terminalSpeed);

		if (health < 0.0f)
		{
			rotX = 0 + deadTimer*720;
		}

		updateTransformationMatrix();
	}

	//animating the vehicle particles

	if (boostDelayTimer > boostDelayMax-boostDuration)
	{
		exhaustLengthTarget = 2.5f;
	}
	else if (onPlane && inputGas)
	{
		exhaustLengthTarget = 1.0f;
	}
	else
	{
		exhaustLengthTarget = 0.0f;
	}

	exhaustLength = Maths::approach(exhaustLength, exhaustLengthTarget, exhaustApproach, dt);

	if (exhaustLength > 0.002f)
	{
		Vector3f upDir(&currNorm);
		Vector3f atDir(&vel);
		Vector3f rightDir = atDir.cross(&upDir);
		upDir = rightDir.cross(&atDir);

		//Vector3f perpen = vel.cross(&currNorm);

		float u3 = upDir.x;
		float v3 = upDir.y;
		float w3 = upDir.z;
		float x3 = atDir.x;
		float y3 = atDir.y;
		float z3 = atDir.z;
		float buf[3];
		Maths::rotatePoint(buf, 0, 0, 0, u3, v3, w3, x3, y3, z3, Maths::toRadians(spinRot - slipAngle));
		atDir.set(buf[0], buf[1], buf[2]);

		u3 = atDir.x;
		v3 = atDir.y;
		w3 = atDir.z;
		x3 = rightDir.x;
		y3 = rightDir.y;
		z3 = rightDir.z;
		Maths::rotatePoint(buf, 0, 0, 0, u3, v3, w3, x3, y3, z3, Maths::toRadians(-rotRoll));
		rightDir.set(buf[0], buf[1], buf[2]);

		upDir.normalize();
		atDir.normalize();
		rightDir.normalize();

		for (EngineExhaust e : Car::exhausts[vehicleID])
		{
			Vector3f exhaustLengthVec(&atDir);
			exhaustLengthVec.scale(-overallVel.length()*e.lengthScale*exhaustLength);

			Vector3f enginePos(&position);
			enginePos.x +=    upDir.x*e.posUp;
			enginePos.y +=    upDir.y*e.posUp;
			enginePos.z +=    upDir.z*e.posUp;
			enginePos.x +=    atDir.x*e.posAt;
			enginePos.y +=    atDir.y*e.posAt;
			enginePos.z +=    atDir.z*e.posAt;
			enginePos.x += rightDir.x*e.posSide;
			enginePos.y += rightDir.y*e.posSide;
			enginePos.z += rightDir.z*e.posSide;

			Vector3f enginePosEnd = enginePos + exhaustLengthVec;

			int numberOfParticlesToCreate = (int)(3 + 10*exhaustLengthVec.length());

			Car::createEngineParticles(&enginePos, &enginePosEnd, e.size*exhaustLength, numberOfParticlesToCreate, e.textureID);
		}
	}



	//Making camera pan up or down depending on if you are driving in a steep concave or convex ground (Old method)

	//Vector3f planeWeCareAbout = Maths::calcThirdAxis(&vel, &currNorm);
	//planeWeCareAbout.normalize();
	//
	//Vector3f velFlat            = projectOntoPlane(&vel,            &planeWeCareAbout);
	//Vector3f currNormFlat       = projectOntoPlane(&currNorm,       &planeWeCareAbout);
	//Vector3f currNormSmoothFlat = projectOntoPlane(&currNormSmooth, &planeWeCareAbout);
	//
	//Vector3f normDiff = currNormFlat-currNormSmoothFlat;
	//Vector3f normDiffFlat = projectOntoPlane(&normDiff, &planeWeCareAbout);
	//
	//Vector3f velFlatNormalized(&velFlat);
	//velFlatNormalized.normalize();
	//Vector3f currNormFlatNormalized(&currNormFlat);
	//currNormFlatNormalized.normalize();
	//
	//Vector3f coordsFlat = Maths::coordinatesRelativeToBasis(&currNormFlatNormalized, &velFlatNormalized, &planeWeCareAbout, &normDiffFlat);
	//std::fprintf(stdout, "[%f %f %f]\n", coordsFlat.x, coordsFlat.y, coordsFlat.z);
	//
	//float extraPan = coordsFlat.y;


	//Animating the camera
	extern float VFOV_ADDITION;

	float speedScale = 1+(vel.length()*camRadiusAdjust);
	VFOV_ADDITION = (vel.length()*0.05f);
	Master_makeProjectionMatrix();

	Vector3f camOffset(&camDirSmooth);
	camOffset.normalize();
	camOffset.scale(camRadius*speedScale);

	float rotationVector[3];
	Maths::rotatePoint(rotationVector, 0, 0, 0, camDirSmooth.x, camDirSmooth.y, camDirSmooth.z, currNormSmooth.x, currNormSmooth.y, currNormSmooth.z, -(float)(M_PI/2));

	float newCameraOffset[3];
	Maths::rotatePoint(newCameraOffset, 0, 0, 0, rotationVector[0], rotationVector[1], rotationVector[2], camOffset.x, camOffset.y, camOffset.z, camAngleLookdown+camAngleAdditionalLookdown);
	camOffset.set(newCameraOffset[0], newCameraOffset[1], newCameraOffset[2]);

	Vector3f camHeight(&currNormSmooth);
	camHeight.normalize();
	camHeight.scale(camHeightOffset);

	Vector3f eye(getPosition());
	eye = eye - camOffset;
	eye = eye + camHeight;

	Vector3f target(getPosition());
	target = target + camHeight;

	Vector3f up(&currNormSmooth);
	up.normalize();

	float newUp[3];
	Maths::rotatePoint(newUp, 0, 0, 0, rotationVector[0], rotationVector[1], rotationVector[2], up.x, up.y, up.z, camAngleLookdown+camAngleAdditionalLookdown);
	up.set(newUp[0], newUp[1], newUp[2]);

	if (fallOutTimer < 0.0f)
	{
		Global::gameCamera->setViewMatrixValues(&eye, &target, &up);
		camDeathPosition.set(&eye);
	}
	else
	{
		Global::gameCamera->setViewMatrixValues(&camDeathPosition, &target, &up);
	}

	Global::gameMainVehicleSpeed = (int)(overallVel.length()*3.46f);

	//Vector3f posDiffDelta = position - prevPos;
	//std::fprintf(stdout, "delta pos = %f\n\n", posDiffDelta.length()/dt);



	if (health < 0)
	{
		if (deadTimer < 0.0f)
		{
			deadTimer = 0.0f;
		}
	}
	else if (onPlane && currentTriangle->isHeal() && deadTimer < 0.0f)
	{
		health += healRate*dt;
	}
	health = std::fminf(1.0f, health);


	//Sound effect stuff

	if (overallVel.length() > 10)
	{
		if (sourceEngine == nullptr)
		{
			sourceEngine = AudioPlayer::play(5, &position, overallVel.length()/250.0f, true);
		}

		if (sourceEngine != nullptr)
		{
			sourceEngine->setPitch(overallVel.length()/200.0f);
			sourceEngine->setPosition(getX(), getY(), getZ());
		}
	}
	else
	{
		if (sourceEngine != nullptr)
		{
			sourceEngine->stop();
			sourceEngine = nullptr;
		}
	}

	if (totalStrafe.length() > 5)
	{
		if (sourceStrafe == nullptr)
		{
			sourceStrafe = AudioPlayer::play(6, &position, 0.5f + sourceStrafeTimer/3.0f, true);
		}

		if (sourceStrafe != nullptr)
		{
			sourceStrafe->setPitch(0.5f + sourceStrafeTimer/3.0f);
			sourceStrafe->setPosition(getX(), getY(), getZ());
		}

		sourceStrafeTimer = std::min(1.5f, sourceStrafeTimer+dt);
	}
	else
	{
		sourceStrafeTimer = 0;
		if (sourceStrafe != nullptr)
		{
			sourceStrafe->stop();
			sourceStrafe = nullptr;
		}
	}

	if (fabsf(slipPunishScale) > 0.03f)
	{
		if (sourceSlipSlowdown == nullptr)
		{
			sourceSlipSlowdown = AudioPlayer::play(7, &position, overallVel.length()/250.0f, true);
		}

		if (sourceSlipSlowdown != nullptr)
		{
			sourceSlipSlowdown->setPitch(overallVel.length()/250.0f);
			sourceSlipSlowdown->setPosition(getX(), getY(), getZ());
		}
	}
	else
	{
		if (sourceSlipSlowdown != nullptr)
		{
			sourceSlipSlowdown->stop();
			sourceSlipSlowdown = nullptr;
		}
	}

	baseColour.set(1, 1, 1);

	if (health >= 0.0f && health <= 0.3f)
	{
		if (sourceDanger == nullptr)
		{
			sourceDanger = AudioPlayer::play(8, &position, 1.9f - 3*health, true);
		}

		if (sourceDanger != nullptr)
		{
			sourceDanger->setPitch(1.9f - 3*health);
			sourceDanger->setPosition(getX(), getY(), getZ());

			float t = sourceDanger->getSoundCompletion();
			baseColour.set(0.5f+t/2, t, t);
		}
	}
	else
	{
		if (sourceDanger != nullptr)
		{
			sourceDanger->stop();
			sourceDanger = nullptr;
		}

		if (health < 0.0f)
		{
			baseColour.set(0.25f, 0.05f, 0.05f);
		}
	}

	if (onPlane && currentTriangle->isHeal())
	{
		if (sourceHeal == nullptr)
		{
			sourceHeal = AudioPlayer::play(1, &position, 1.0f, true);
		}
		baseColour.set(1.9f, 1.5f, 1.7f);
	}
	else
	{
		if (sourceHeal != nullptr)
		{
			sourceHeal->stop();
			sourceHeal = nullptr;
		}
	}

	checkpointTest();

	Vector3f vnorm(&vel);
	vnorm.normalize();
	//std::fprintf(stdout, "pos  = [%f, %f, %f]\n", position.x, position.y, position.z);
	//std::fprintf(stdout, "norm = [%f, %f, %f]\n", currNorm.x, currNorm.y, currNorm.z);
	//std::fprintf(stdout, "dir  = [%f, %f, %f]\n", vnorm   .x, vnorm   .y, vnorm   .z);
	//std::fprintf(stdout, "%f %f %f   %f %f %f   %f %f %f\n", position.x, position.y, position.z, currNorm.x, currNorm.y, currNorm.z, vnorm.x, vnorm.y, vnorm.z);
}

void Car::createEngineParticles(Vector3f* initPos, Vector3f* endPos, float initialScale, int count, int textureIndex)
{
	Vector3f diff(endPos);
	diff = diff - initPos;

	diff.scale(1.0f / (count));

	for (int i = 0; i < count; i++)
	{
		Vector3f offset(&diff);
		offset.scale((float)i);
		offset = offset + initPos;

		Vector3f randomJitter = Maths::randomPointOnSphere(); //Not sure which one would look better, a uniform circle or 3 uniform points...
		//Vector3f randomJitter(Maths::nextUniform(),Maths::nextUniform(), Maths::nextUniform());
		randomJitter.scale(((float)i)/count);
		randomJitter.scale(0.2f); //Constant for how much the last point will be randomized
		offset = offset + randomJitter;

		float newScale = ((count-i)/((float)count));

		new Particle(textureIndex, &offset, 1.0f, initialScale*newScale, true);
	}
}

void Car::checkpointTest()
{
	if (fallOutTimer >= 0.0f)
	{
		return;
	}

	Checkpoint* myCheck = nullptr;
	int newCheckpointID = -1;
	for (Checkpoint* check : Global::gameCheckpointList)
	{
		if (check->isPointInsideMe(getPosition()))
		{
			if (newCheckpointID == -1 || check->ID < newCheckpointID)
			{
				newCheckpointID = check->ID;
				myCheck = check;
			}
		}
	}

	if (newCheckpointID == -1 && fallOutTimer < 0.0f)
	{
		AudioPlayer::play(12, getPosition());
		fallOutTimer = 0.0f;
	}

	if (newCheckpointID != -1 && newCheckpointID != lastCheckpointID)
	{
		int diff = newCheckpointID-lastCheckpointID;
		if (newCheckpointID == 0 && lastCheckpointID == Global::gameCheckpointLast)
		{
			diff = 1;
		}
		if (newCheckpointID == Global::gameCheckpointLast && lastCheckpointID == 0)
		{
			diff = -1;
		}

		if (abs(diff) >= 2)
		{
			AudioPlayer::play(12, getPosition());
			fallOutTimer = 0.0f;
		}

		lapDistance+=diff;

		if (lapDistance < 0)
		{
			currentLap--;
			lapDistance = Global::gameCheckpointLast;
		}
		else if (lapDistance > Global::gameCheckpointLast)
		{
			currentLap++;
			lapDistance = 0;

			switch (currentLap)
			{
				case 1:
					AudioPlayer::play(9,  getPosition());
					break;

				case 2:
					AudioPlayer::play(10, getPosition());
					AudioPlayer::playBGMWithIntro(2, 3);
					break;

				case 3:
					//AudioPlayer::playBGMWithIntro(2, 3);
					GuiManager::stopTimer();
					Global::finishStageTimer = 0;
					break;

				default: break;
			}
		}
	}

	if (newCheckpointID != -1)
	{
		lastCheckpointID = newCheckpointID;
	}

	//std::fprintf(stdout, "currentLap = %d    lapDistance = %d    checkID = %d\n", currentLap, lapDistance, newCheckpointID);
}

void Car::explode()
{
	if (sourceEngine       != nullptr) sourceEngine->stop();
	if (sourceStrafe       != nullptr) sourceStrafe->stop();
	if (sourceSlipSlowdown != nullptr) sourceSlipSlowdown->stop();
	if (sourceDanger       != nullptr) sourceDanger->stop();
	if (sourceHeal         != nullptr) sourceHeal->stop();

	vel.normalize();
	vel.scale(VEL_SLOWEST);

	AudioPlayer::play(18, getPosition());
	isDead = true;
	visible = false;

	float height = 0.0f;
	float spread = 4.0f;

	for (int i = 7; i != 0; i--)
	{
		Vector3f pos(
			getX() + spread*(Maths::random() - 0.5f),
			getY() + spread*(Maths::random() - 0.5f) + height,
			getZ() + spread*(Maths::random() - 0.5f));

		Vector3f vel2(0, 0, 0);

		new Particle(ParticleResources::textureExplosion1, &pos, &vel2,
			0, 1.5f, 0, 0.5f * Maths::random() + 1.0f, 0, false, false);
	}

	Vector3f pos(
		getX(),
		getY() + height,
		getZ());

	Vector3f vel2(0, 0, 0);
	
	new Particle(ParticleResources::textureExplosion2, &pos, &vel2,
		0, 2.0f, 0, 3.0f, 0, false, false);
}

void Car::giveMeABoost()
{
	boostDelayTimer = boostDelayMax*1.0f;

	float oldSpeed = vel.length();
	float newSpeed = oldSpeed + 1.0f*boostKick*(boostSpeed - oldSpeed);

	if (newSpeed > oldSpeed)
	{
		float ratio = newSpeed/oldSpeed;
		vel.x*=ratio;
		vel.y*=ratio;
		vel.z*=ratio;
	}
}

void Car::giveMeAJump()
{
	vel.y += 120;
	position.y += 5;
	onPlane = false;
}

void Car::setVelocity(float xVel, float yVel, float zVel)
{
	vel.x = xVel;
	vel.y = yVel;
	vel.z = zVel;
}

void Car::setCanMoveTimer(float newTimer)
{
	canMoveTimer = newTimer;
}

void Car::setLapDistance(int newDistance)
{
	lapDistance = newDistance;
}

void Car::setInputs()
{
	inputGas        = Input::inputs.INPUT_ACTION1;
	inputBrake      = Input::inputs.INPUT_ACTION2;
	inputAttackSide = Input::inputs.INPUT_ACTION3;
	inputBoost      = Input::inputs.INPUT_ACTION4;
	inputAttackSpin = Input::inputs.INPUT_RB;
	inputWheel      = Input::inputs.INPUT_X;
	inputDive       = Input::inputs.INPUT_Y;
	inputL          = Input::inputs.INPUT_L2;
	inputR          = Input::inputs.INPUT_R2;

	inputWheelJerk = Input::inputs.INPUT_X - Input::inputs.INPUT_PREVIOUS_X;

	inputGasPrevious        = Input::inputs.INPUT_PREVIOUS_ACTION1;
	inputBrakePrevious      = Input::inputs.INPUT_PREVIOUS_ACTION2;
	inputAttackSidePrevious = Input::inputs.INPUT_PREVIOUS_ACTION3;
	inputBoostPrevious      = Input::inputs.INPUT_PREVIOUS_ACTION4;
	inputAttackSpinPrevious = Input::inputs.INPUT_PREVIOUS_RB;

	if (canMoveTimer > 0.0f || deadTimer >= 0.0f || fallOutTimer >= 0.0f)
	{
		inputGas        = 0;
		inputBrake      = 0;
		inputBoost      = 0;
		inputWheel      = 0;
		inputDive       = 0;
		inputL          = 0;
		inputR          = 0;
		inputAttackSpin = 0;
	    inputAttackSide = 0;

		inputWheelJerk = 0;

		inputGasPrevious        = 0;
		inputBrakePrevious      = 0;
		inputBoostPrevious      = 0;
		inputAttackSidePrevious = 0;
		inputAttackSpinPrevious = 0;
	}
}

std::list<TexturedModel*>* Car::getModels()
{
	return &Car::models[vehicleID];
}

void Car::loadVehicleInfo()
{
	if (Car::exhausts[vehicleID].size() == 0)
	{
		std::string engineFilename = "";
		switch (vehicleID)
		{
			case 0: engineFilename = "res/Models/Machines/BlueFalcon/Engine.ini";   break;
			case 4: engineFilename = "res/Models/Machines/Arwing/Engine.ini";       break;
			case 5: engineFilename = "res/Models/Machines/RedGazelle/Engine.ini";   break;
			case 1: engineFilename = "res/Models/Machines/TwinNorita/Engine.ini";   break;
			case 3: engineFilename = "res/Models/Machines/BlackBull/Engine.ini";    break;
			case 2: engineFilename = "res/Models/Machines/SonicPhantom/Engine.ini"; break;
			default: break;
		}

		std::ifstream file(engineFilename);
		if (!file.is_open())
		{
			std::fprintf(stderr, "Error: Cannot load file '%s'\n", engineFilename.c_str());
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

				if (splitLength == 7 && lineSplit[0][0] != '#')
				{
					EngineExhaust newExahust;
					newExahust.textureID         = std::stoi(lineSplit[0], nullptr, 10);
					newExahust.posUp             = std::stof(lineSplit[1]);
					newExahust.posAt             = std::stof(lineSplit[2]) + std::stof(lineSplit[4]);
					newExahust.posSide           = std::stof(lineSplit[3]);
					newExahust.lengthScale       = std::stof(lineSplit[5]);
					newExahust.size              = std::stof(lineSplit[6]);
					Car::exhausts[vehicleID].push_back(newExahust);
				}
				free(lineSplit);
			}
			file.close();
		}
	}

	//if (Car::machineBaseStats[vehicleID].weight = 0)
	{
		std::string statsFilename = "";
		switch (vehicleID)
		{
			case 0: statsFilename = "res/Models/Machines/BlueFalcon/Machine.ini";   break;
			case 4: statsFilename = "res/Models/Machines/Arwing/Machine.ini";       break;
			case 5: statsFilename = "res/Models/Machines/RedGazelle/Machine.ini";   break;
			case 1: statsFilename = "res/Models/Machines/TwinNorita/Machine.ini";   break;
			case 3: statsFilename = "res/Models/Machines/BlackBull/Machine.ini";    break;
			case 2: statsFilename = "res/Models/Machines/SonicPhantom/Machine.ini"; break;
			default: break;
		}

		std::ifstream file(statsFilename);
		if (!file.is_open())
		{
			std::fprintf(stderr, "Error: Cannot load file '%s'\n", statsFilename.c_str());
			file.close();
		}
		else
		{
			std::string line;
			char lineBuf[512];
			int splitLength;

			getline(file, line);
			getline(file, line);

			memcpy(lineBuf, line.c_str(), line.size()+1);
			char** lineSplit = split(lineBuf, ';', &splitLength);

			terminalSpeed        = std::stof(lineSplit[1], nullptr); free(lineSplit); getline(file, line); memcpy(lineBuf, line.c_str(), line.size()+1); lineSplit = split(lineBuf, ';', &splitLength);
			terminalAccelGas	 = std::stof(lineSplit[1], nullptr); free(lineSplit); getline(file, line); memcpy(lineBuf, line.c_str(), line.size()+1); lineSplit = split(lineBuf, ';', &splitLength);
			terminalAccelBrake	 = std::stof(lineSplit[1], nullptr); free(lineSplit); getline(file, line); memcpy(lineBuf, line.c_str(), line.size()+1); lineSplit = split(lineBuf, ';', &splitLength);
			terminalAccelCoast	 = std::stof(lineSplit[1], nullptr); free(lineSplit); getline(file, line); memcpy(lineBuf, line.c_str(), line.size()+1); lineSplit = split(lineBuf, ';', &splitLength);
			turnSpeed			 = std::stof(lineSplit[1], nullptr); free(lineSplit); getline(file, line); memcpy(lineBuf, line.c_str(), line.size()+1); lineSplit = split(lineBuf, ';', &splitLength);
			turnPunish			 = std::stof(lineSplit[1], nullptr); free(lineSplit); getline(file, line); memcpy(lineBuf, line.c_str(), line.size()+1); lineSplit = split(lineBuf, ';', &splitLength);
			diveSpeed			 = std::stof(lineSplit[1], nullptr); free(lineSplit); getline(file, line); memcpy(lineBuf, line.c_str(), line.size()+1); lineSplit = split(lineBuf, ';', &splitLength);
			divePunish			 = std::stof(lineSplit[1], nullptr); free(lineSplit); getline(file, line); memcpy(lineBuf, line.c_str(), line.size()+1); lineSplit = split(lineBuf, ';', &splitLength);
			strafePercentage	 = std::stof(lineSplit[1], nullptr); free(lineSplit); getline(file, line); memcpy(lineBuf, line.c_str(), line.size()+1); lineSplit = split(lineBuf, ';', &splitLength);
			strafeTerminalPunish = std::stof(lineSplit[1], nullptr); free(lineSplit); getline(file, line); memcpy(lineBuf, line.c_str(), line.size()+1); lineSplit = split(lineBuf, ';', &splitLength);
			slipTimerMax		 = std::stof(lineSplit[1], nullptr); free(lineSplit); getline(file, line); memcpy(lineBuf, line.c_str(), line.size()+1); lineSplit = split(lineBuf, ';', &splitLength);
			slipThreshold		 = std::stof(lineSplit[1], nullptr); free(lineSplit); getline(file, line); memcpy(lineBuf, line.c_str(), line.size()+1); lineSplit = split(lineBuf, ';', &splitLength);
			slipAngleAccel		 = std::stof(lineSplit[1], nullptr); free(lineSplit); getline(file, line); memcpy(lineBuf, line.c_str(), line.size()+1); lineSplit = split(lineBuf, ';', &splitLength);
			slipAngleMax		 = std::stof(lineSplit[1], nullptr); free(lineSplit); getline(file, line); memcpy(lineBuf, line.c_str(), line.size()+1); lineSplit = split(lineBuf, ';', &splitLength);
			slipPower			 = std::stof(lineSplit[1], nullptr); free(lineSplit); getline(file, line); memcpy(lineBuf, line.c_str(), line.size()+1); lineSplit = split(lineBuf, ';', &splitLength);
			slipNegativePower	 = std::stof(lineSplit[1], nullptr); free(lineSplit); getline(file, line); memcpy(lineBuf, line.c_str(), line.size()+1); lineSplit = split(lineBuf, ';', &splitLength);
			slipPositivePower	 = std::stof(lineSplit[1], nullptr); free(lineSplit); getline(file, line); memcpy(lineBuf, line.c_str(), line.size()+1); lineSplit = split(lineBuf, ';', &splitLength);
			slipTimerThreshold	 = std::stof(lineSplit[1], nullptr); free(lineSplit); getline(file, line); memcpy(lineBuf, line.c_str(), line.size()+1); lineSplit = split(lineBuf, ';', &splitLength);
			spinTimeMax			 = std::stof(lineSplit[1], nullptr); free(lineSplit); getline(file, line); memcpy(lineBuf, line.c_str(), line.size()+1); lineSplit = split(lineBuf, ';', &splitLength);
			spinTimeDelay		 = std::stof(lineSplit[1], nullptr); free(lineSplit); getline(file, line); memcpy(lineBuf, line.c_str(), line.size()+1); lineSplit = split(lineBuf, ';', &splitLength);
			sideAttackTimeMax	 = std::stof(lineSplit[1], nullptr); free(lineSplit); getline(file, line); memcpy(lineBuf, line.c_str(), line.size()+1); lineSplit = split(lineBuf, ';', &splitLength);
			sideAttackSpeed		 = std::stof(lineSplit[1], nullptr); free(lineSplit); getline(file, line); memcpy(lineBuf, line.c_str(), line.size()+1); lineSplit = split(lineBuf, ';', &splitLength);
			boostSpeed			 = std::stof(lineSplit[1], nullptr); free(lineSplit); getline(file, line); memcpy(lineBuf, line.c_str(), line.size()+1); lineSplit = split(lineBuf, ';', &splitLength);
			boostKick			 = std::stof(lineSplit[1], nullptr); free(lineSplit); getline(file, line); memcpy(lineBuf, line.c_str(), line.size()+1); lineSplit = split(lineBuf, ';', &splitLength);
			boostDuration		 = std::stof(lineSplit[1], nullptr); free(lineSplit); getline(file, line); memcpy(lineBuf, line.c_str(), line.size()+1); lineSplit = split(lineBuf, ';', &splitLength);
			boostDelayMax		 = std::stof(lineSplit[1], nullptr); free(lineSplit); getline(file, line); memcpy(lineBuf, line.c_str(), line.size()+1); lineSplit = split(lineBuf, ';', &splitLength);
			boostHealthPunish	 = std::stof(lineSplit[1], nullptr); free(lineSplit); getline(file, line); memcpy(lineBuf, line.c_str(), line.size()+1); lineSplit = split(lineBuf, ';', &splitLength);
			healRate			 = std::stof(lineSplit[1], nullptr); free(lineSplit); getline(file, line); memcpy(lineBuf, line.c_str(), line.size()+1); lineSplit = split(lineBuf, ';', &splitLength);
			hitWallHealthPunish	 = std::stof(lineSplit[1], nullptr); free(lineSplit); getline(file, line); memcpy(lineBuf, line.c_str(), line.size()+1); lineSplit = split(lineBuf, ';', &splitLength);
			weight				 = std::stof(lineSplit[1], nullptr); free(lineSplit); getline(file, line); memcpy(lineBuf, line.c_str(), line.size()+1); lineSplit = split(lineBuf, ';', &splitLength);
			gravityForce		 = std::stof(lineSplit[1], nullptr); free(lineSplit); getline(file, line); memcpy(lineBuf, line.c_str(), line.size()+1); lineSplit = split(lineBuf, ';', &splitLength);
			boostIndex           = std::stoi(lineSplit[1], nullptr, 10); free(lineSplit);

			file.close();
		}
	}

	if (Car::models[vehicleID].size() > 0)
	{
		return;
	}

	#ifdef DEV_MODE
	std::fprintf(stdout, "Loading Car static models...\n");
	#endif

	std::string modelFolder = "";
	std::string modelName = "";
	switch (vehicleID)
	{
		case 0: modelFolder = "res/Models/Machines/BlueFalcon/";   modelName = "GX";           break;
		case 4: modelFolder = "res/Models/Machines/Arwing/";       modelName = "Arwing";       break;
		case 5: modelFolder = "res/Models/Machines/RedGazelle/";   modelName = "RedGazelle";   break;
		case 1: modelFolder = "res/Models/Machines/TwinNorita/";   modelName = "TwinNorita";   break;
		case 3: modelFolder = "res/Models/Machines/BlackBull/";    modelName = "BlackBull";    break;
		case 2: modelFolder = "res/Models/Machines/SonicPhantom/"; modelName = "SonicPhantom"; break;
		default: break;
	}
	loadModel(&Car::models[vehicleID], modelFolder, modelName);
}

void Car::deleteStaticModels()
{
	#ifdef DEV_MODE
	std::fprintf(stdout, "Deleting Car static models...\n");
	#endif

	for (int i = 0; i < 30; i++)
	{
		std::list<TexturedModel*>* e = &Car::models[i];
		if (e->size() > 0)
		{
			Entity::deleteModels(e);
		}
	}
}

bool Car::isVehicle()
{
	return true;
}
