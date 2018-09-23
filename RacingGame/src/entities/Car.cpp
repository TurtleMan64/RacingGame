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

#include <list>
#include <iostream>
#include <algorithm>
#include <fstream>

std::list<TexturedModel*> Car::models  [30];
std::list<EngineExhaust>  Car::exhausts[30];

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

void Car::step()
{
	canMoveTimer    = std::fmaxf(0.0f, canMoveTimer    - dt);
	boostDelayTimer = std::fmaxf(0.0f, boostDelayTimer - dt);
	slipTimer       = std::fmaxf(0.0f, slipTimer       - dt);

	setInputs();

	//Vector3f prevPos(&position);

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
		slipPunishScale = -slipPunishScale;
	}
	
	//turn due to slipping
	float xIn2 = -slipPower*(slipAngle/slipAngleMax)*dt;
	vel.scale(1 - slipPunishScale*(std::fabsf(xIn2))); //slow down
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
	Vector3f strafeDir = currNorm.cross(&vel);
	strafeDir.normalize();
	Vector3f strafeRAmount(&strafeDir);
	Vector3f strafeLAmount(&strafeDir);
	strafeRAmount.scale(-inputR);
	strafeLAmount.scale( inputL);
	Vector3f totalStrafe = (strafeRAmount + strafeLAmount);
	totalStrafe.scale(strafePercentage);
	totalStrafe.scale(vel.length());
	float strafeScale = std::fabsf(inputL-inputR);
	float strafePunish = 1.0f - strafeScale*(1.0f - strafeTerminalPunish);


	//Boost
	if (inputBoost && !inputBoostPrevious)
	{
		if (boostDelayTimer == 0.0f && onPlane)
		{
			AudioPlayer::play(2, getPosition());

			boostDelayTimer = boostDelayMax;

			float oldSpeed = vel.length();
			float newSpeed = oldSpeed + boostKick*(boostSpeed - oldSpeed);

			if (newSpeed > oldSpeed)
			{
				float ratio = newSpeed/oldSpeed;
				vel.x*=ratio;
				vel.y*=ratio;
				vel.z*=ratio;
			}
		}
	}

	float brakePunish = 1.0f;
	if (onPlane && currentTriangle->isBrake())
	{
		brakePunish = 0.5f;
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

		float newSpeed = Maths::approach(oldSpeed, terminal*strafePunish*brakePunish, terminalAccelGas, dt);

		float ratio = newSpeed/oldSpeed;
		vel.x*=ratio;
		vel.y*=ratio;
		vel.z*=ratio;
	}
	else if (inputBrake && onPlane)
	{
		float oldSpeed = vel.length();
		float newSpeed = Maths::approach(oldSpeed, 0, terminalAccelBrake/brakePunish, dt);
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
		float newSpeed = Maths::approach(oldSpeed, 0, terminalAccelCoast/brakePunish, dt);
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

		//if (vel.y > 0)
		//{
			//yIn = fminf(yIn, 0);
		//}

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
			vel.x = buf[0];
			vel.y = buf[1];
			vel.z = buf[2];
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


	Vector3f overallVel = vel + totalStrafe;

	//speed before adjusting
	float originalSpeed = vel.length();

	CollisionChecker::setCheckPlayer();
	if (CollisionChecker::checkCollision(getX(), getY(), getZ(), getX()+overallVel.x*dt, getY()+overallVel.y*dt, getZ()+overallVel.z*dt))
	{
		Vector3f* colNormal = &CollisionChecker::getCollideTriangle()->normal;

		if (onPlane  == false) //Air to ground
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
		else //Ground to a different triangle
		{
			//check if you can smoothly transition from previous triangle to this triangle
			float dotProduct = currNorm.dot(colNormal);
			if (dotProduct < smoothTransitionThreshold)
			{
				Vector3f bounceVec = Maths::bounceVector(&overallVel, colNormal, 1.0f);
				Vector3f newDir = bounceVec + bounceVec + overallVel;
				newDir.normalize();
				newDir.scale(originalSpeed*0.75f);
				vel.set(&newDir);

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
						if (dotProduct < smoothTransitionThreshold)
						{
							Vector3f bounceVec = Maths::bounceVector(&nextVel, colNormal, 1.0f);
							Vector3f newDir = bounceVec + bounceVec + nextVel;
							newDir.normalize();
							newDir.scale(originalSpeed*0.75f);
							vel.set(&newDir);

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

			if (dotProduct < smoothTransitionThreshold) //It's a wall, pretend the collision check didn't see it
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
			if (dotProduct < smoothTransitionThreshold)
			{
				CollisionChecker::falseAlarm();
				Vector3f bounceVec = Maths::bounceVector(&overallVel, colNormal, 1.0f);
				Vector3f newDir = bounceVec + bounceVec + overallVel;
				newDir.scale(0.25f);
				vel.set(&newDir);

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

		rotX = diff - slipAngle;
		rotY = yawAngle;
		rotZ = pitchAngle;
		rotRoll = 10*(inputL - inputR)*(vel.length()/terminalSpeed);

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

		rotX = 0;
		rotY = yawAngle;
		rotZ = pitchAngle+90;
		rotRoll = 10*(inputL - inputR)*(vel.length()/terminalSpeed);

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

		Vector3f perpen = vel.cross(&currNorm);

		float u3 = upDir.x;
		float v3 = upDir.y;
		float w3 = upDir.z;
		float x3 = atDir.x;
		float y3 = atDir.y;
		float z3 = atDir.z;
		float buf[3];
		Maths::rotatePoint(buf, 0, 0, 0, u3, v3, w3, x3, y3, z3, Maths::toRadians(-slipAngle));
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

	Global::gameCamera->setViewMatrixValues(&eye, &target, &up);

	Global::gameMainVehicleSpeed = (int)(overallVel.length()*3.46f);

	//Vector3f posDiffDelta = position - prevPos;
	//std::fprintf(stdout, "delta pos = %f\n\n", posDiffDelta.length()/dt);



	//Sound effect stuff

	if (onPlane && overallVel.length() > 10)
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

void Car::setInputs()
{
	inputGas   = Input::inputs.INPUT_JUMP;
	inputBrake = Input::inputs.INPUT_ACTION;
	inputBoost = Input::inputs.INPUT_SPECIAL;
	inputWheel = Input::inputs.INPUT_X;
	inputDive  = Input::inputs.INPUT_Y;
	inputL     = Input::inputs.INPUT_L2;
	inputR     = Input::inputs.INPUT_R2;

	inputWheelJerk = Input::inputs.INPUT_X - Input::inputs.INPUT_PREVIOUS_X;

	inputGasPrevious   = Input::inputs.INPUT_PREVIOUS_JUMP;
	inputBrakePrevious = Input::inputs.INPUT_PREVIOUS_ACTION;
	inputBoostPrevious = Input::inputs.INPUT_PREVIOUS_SPECIAL;

	if (canMoveTimer > 0.0f)
	{
		inputGas   = 0;
		inputBrake = 0;
		inputBoost = 0;
		inputWheel = 0;
		inputDive  = 0;
		inputL     = 0;
		inputR     = 0;

		inputWheelJerk = 0;

		inputGasPrevious   = 0;
		inputBrakePrevious = 0;
		inputBoostPrevious = 0;
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
			case 0: engineFilename = "res/Models/BlueFalcon/Engine.ini"; break;
			case 1: engineFilename = "res/Models/Arwing/Engine.ini";     break;
			default: break;
		}

		std::ifstream file(engineFilename);
		if (!file.is_open())
		{
			std::fprintf(stdout, "Error: Cannot load file '%s'\n", engineFilename.c_str());
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
		case 0: modelFolder = "res/Models/BlueFalcon/"; modelName = "BlueFalcon.obj"; break;
		case 1: modelFolder = "res/Models/Arwing/";     modelName = "Arwing.obj";     break;
		default: break;
	}
	loadObjModel(&Car::models[vehicleID], modelFolder, modelName);
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

