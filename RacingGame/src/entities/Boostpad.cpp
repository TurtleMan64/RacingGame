#include <glad/glad.h>

#include "entity.h"
#include "../models/models.h"
#include "../toolbox/vector.h"
#include "boostpad.h"
#include "../renderEngine/renderEngine.h"
#include "../objLoader/objLoader.h"
#include "../engineTester/main.h"
#include "../entities/car.h"
#include "../toolbox/maths.h"
#include "dummy.h"
#include "../entities/camera.h"
#include "../audio/audioplayer.h"
#include "../particles/particleresources.h"
#include "../particles/particle.h"
#include "../collision/collisionchecker.h"

#include <list>
#include <iostream>
#include <algorithm>
#include <cmath>

std::list<TexturedModel*> Boostpad::modelsPad;
std::list<TexturedModel*> Boostpad::modelsBolt;
CollisionModel* Boostpad::cmOriginal;

Boostpad::Boostpad()
{

}

Boostpad::Boostpad(float x, float y, float z, float xRot, float yRot, float zRot)
{
	position.x = x;
	position.y = y;
	position.z = z;
	rotX = xRot;
	rotY = yRot;
	rotZ = zRot;
	scale = 1;
	visible = true;
	playerIsIn = false;
	updateTransformationMatrix();

	bolt = new Dummy(&Boostpad::modelsBolt); INCR_NEW
	bolt->setVisible(true);
	bolt->setPosition(&position);
	bolt->setRotation(xRot, yRot, zRot, 0);
	bolt->updateTransformationMatrix();
	Main_addTransparentEntity(bolt);

	//cmTransformed = loadCollisionModel("Models/Boostpad/", "Collision");
	//CollisionChecker::addCollideModel(cmTransformed);
	//updateCollisionModel();
}

void Boostpad::step()
{
	if (abs(getX() - Global::gameCamera->eye.x) > ENTITY_RENDER_DIST)
	{
		setVisible(false);
		bolt->setVisible(false);
	}
	else
	{
		if (abs(getZ() - Global::gameCamera->eye.z) > ENTITY_RENDER_DIST)
		{
			setVisible(false);
			bolt->setVisible(false);
		}
		else
		{
			setVisible(true);
			bolt->setVisible(true);

			if (abs(getX() - Global::gameMainVehicle->getPosition()->x) < 10 &&
				abs(getZ() - Global::gameMainVehicle->getPosition()->z) < 10 &&
				abs(getY() - Global::gameMainVehicle->getPosition()->y) < 10)
			{
				Vector3f diff(
					getX()-Global::gameMainVehicle->getPosition()->x, 
					getY()-Global::gameMainVehicle->getPosition()->y,
					getZ()-Global::gameMainVehicle->getPosition()->z);

				if (diff.lengthSquared() < 5.5f*5.5f)
				{
					if (!playerIsIn)
					{
						AudioPlayer::play(0, getPosition());
						Global::gameMainVehicle->giveMeABoost();
					}

					playerIsIn = true;
				}
				else
				{
					playerIsIn = false;
				}
			}
			else
			{
				playerIsIn = false;
			}
		}
	}
}

std::list<TexturedModel*>* Boostpad::getModels()
{
	return &Boostpad::modelsPad;
}

void Boostpad::loadStaticModels()
{
	if (Boostpad::modelsPad.size() > 0)
	{
		return;
	}

	#ifdef DEV_MODE
	std::fprintf(stdout, "Loading Boostpad static models...\n");
	#endif

	loadObjModel(&Boostpad::modelsPad,  "res/Models/Boostpad/", "Pad.obj");
	loadObjModel(&Boostpad::modelsBolt, "res/Models/Boostpad/", "Bolt.obj");

	if (Boostpad::cmOriginal == nullptr)
	{
		//Boostpad::cmOriginal = loadCollisionModel("Models/Boostpad/", "Collision");
	}
}

void Boostpad::deleteStaticModels()
{
	#ifdef DEV_MODE
	std::fprintf(stdout, "Deleting Boostpad static models...\n");
	#endif

	Entity::deleteModels(&Boostpad::modelsPad);
	Entity::deleteModels(&Boostpad::modelsBolt);
	//Entity::deleteCollisionModel(&Boostpad::cmOriginal);
}
