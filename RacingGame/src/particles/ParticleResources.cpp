#include "particleresources.h"
#include "particletexture.h"
#include "../renderEngine/renderEngine.h"
#include "../engineTester/main.h"

std::vector<ParticleTexture*> ParticleResources::exhaustTextures;

ParticleTexture* ParticleResources::textureDust = nullptr;
ParticleTexture* ParticleResources::textureDirt = nullptr;
ParticleTexture* ParticleResources::textureSnowDrop = nullptr;
ParticleTexture* ParticleResources::textureDustCloud = nullptr;
ParticleTexture* ParticleResources::textureSnowball = nullptr;
ParticleTexture* ParticleResources::textureStar = nullptr;
ParticleTexture* ParticleResources::textureSparkleYellow = nullptr;
ParticleTexture* ParticleResources::textureSparkleGreen = nullptr;
ParticleTexture* ParticleResources::textureSparkleRed = nullptr;
ParticleTexture* ParticleResources::textureSparkleBlue = nullptr;
ParticleTexture* ParticleResources::textureSparkleLightBlue = nullptr;
ParticleTexture* ParticleResources::textureSparkleWhite = nullptr;
ParticleTexture* ParticleResources::textureWaterDrop = nullptr;
ParticleTexture* ParticleResources::textureWhiteTrail = nullptr;
ParticleTexture* ParticleResources::textureLightBlueTrail = nullptr;
ParticleTexture* ParticleResources::textureBlueTrail = nullptr;
ParticleTexture* ParticleResources::textureBlackTrail = nullptr;
ParticleTexture* ParticleResources::textureGrayTrail = nullptr;
ParticleTexture* ParticleResources::texturePinkTrail = nullptr;
ParticleTexture* ParticleResources::textureDarkGreenTrail = nullptr;
ParticleTexture* ParticleResources::textureOrangeTrail = nullptr;
ParticleTexture* ParticleResources::textureRedTrail = nullptr;
ParticleTexture* ParticleResources::textureSplash = nullptr;
ParticleTexture* ParticleResources::textureBubble = nullptr;
ParticleTexture* ParticleResources::textureExplosion1 = nullptr;
ParticleTexture* ParticleResources::textureExplosion2 = nullptr;
ParticleTexture* ParticleResources::textureExplosion3 = nullptr;
ParticleTexture* ParticleResources::textureBlackFade = nullptr;
ParticleTexture* ParticleResources::textureBlackFadeOut = nullptr;
ParticleTexture* ParticleResources::textureTear1 = nullptr;
ParticleTexture* ParticleResources::textureTear2 = nullptr;
ParticleTexture* ParticleResources::textureWhiteFadeOutAndIn = nullptr;
ParticleTexture* ParticleResources::textureBlackFadeOutAndIn = nullptr;
ParticleTexture* ParticleResources::textureInWater = nullptr;
ParticleTexture* ParticleResources::textureBlueLine = nullptr;

void ParticleResources::loadParticles()
{
	INCR_NEW exhaustTextures.push_back(new ParticleTexture(Loader::loadTexture("res/Images/Particles/ExhaustBlueFalcon.png"), 1, 0.2f, 1));
	INCR_NEW exhaustTextures.push_back(new ParticleTexture(Loader::loadTexture("res/Images/Particles/ExhaustArwing.png"), 1, 0.3f, 1));

	INCR_NEW textureSnowDrop          = new ParticleTexture(Loader::loadTexture("res/Images/Particles/SnowDropAtlas.png"), 2, 1.0f, 0);
	INCR_NEW textureDustCloud         = new ParticleTexture(Loader::loadTexture("res/Images/Particles/DustCloud.png"), 1, 0.2f, 0);
	INCR_NEW textureSnowball          = new ParticleTexture(Loader::loadTexture("res/Images/Particles/Snowball.png"), 1, 0.75f, 0);
	INCR_NEW textureStar              = new ParticleTexture(Loader::loadTexture("res/Images/Particles/Star.png"), 1, 1.0f, 0);
	INCR_NEW textureSparkleYellow     = new ParticleTexture(Loader::loadTexture("res/Images/Particles/SparkleYellow.png"), 1, 1.0f, 1);
	INCR_NEW textureSparkleGreen      = new ParticleTexture(Loader::loadTexture("res/Images/Particles/SparkleGreen.png"), 1, 1.0f, 1);
	INCR_NEW textureSparkleRed        = new ParticleTexture(Loader::loadTexture("res/Images/Particles/SparkleRed.png"), 1, 1.0f, 1);
	INCR_NEW textureSparkleBlue       = new ParticleTexture(Loader::loadTexture("res/Images/Particles/SparkleBlue.png"), 1, 1.0f, 1);
	INCR_NEW textureSparkleLightBlue  = new ParticleTexture(Loader::loadTexture("res/Images/Particles/SparkleLightBlue.png"), 1, 1.0f, 1);
	INCR_NEW textureSparkleWhite      = new ParticleTexture(Loader::loadTexture("res/Images/Particles/SparkleWhite.png"), 1, 1.0f, 1);
	INCR_NEW textureWaterDrop         = new ParticleTexture(Loader::loadTexture("res/Images/Particles/WaterDrop.png"), 1, 1, 0);
	INCR_NEW textureWhiteTrail        = new ParticleTexture(Loader::loadTexture("res/Images/Particles/SpTrailWhite.png"), 1, 0.3f, 1);
	INCR_NEW textureLightBlueTrail    = new ParticleTexture(Loader::loadTexture("res/Images/Particles/SpTrailLightBlue.png"), 1, 0.1f, 1);
	INCR_NEW textureBlueTrail         = new ParticleTexture(Loader::loadTexture("res/Images/Particles/SpTrailBlue.png"), 1, 0.1f, 1);
	INCR_NEW textureBlackTrail        = new ParticleTexture(Loader::loadTexture("res/Images/Particles/SpTrailBlack.png"), 1, 0.1f, 1);
	INCR_NEW textureGrayTrail         = new ParticleTexture(Loader::loadTexture("res/Images/Particles/SpTrailGray.png"), 1, 0.1f, 1);
	INCR_NEW texturePinkTrail         = new ParticleTexture(Loader::loadTexture("res/Images/Particles/SpTrailPink.png"), 1, 0.1f, 1);
	INCR_NEW textureDarkGreenTrail    = new ParticleTexture(Loader::loadTexture("res/Images/Particles/SpTrailDarkGreen.png"), 1, 0.1f, 1);
	INCR_NEW textureOrangeTrail       = new ParticleTexture(Loader::loadTexture("res/Images/Particles/SpTrailOrange.png"), 1, 0.3f, 1);
	INCR_NEW textureRedTrail          = new ParticleTexture(Loader::loadTexture("res/Images/Particles/SpTrailRed.png"), 1, 0.3f, 1);
	INCR_NEW textureDust              = new ParticleTexture(Loader::loadTexture("res/Images/Particles/DustAtlas.png"), 4, 0.2f, 0);
	INCR_NEW textureDirt              = new ParticleTexture(Loader::loadTexture("res/Images/Particles/DirtAtlas.png"), 4, 0.75f, 0);
	INCR_NEW textureSplash            = new ParticleTexture(Loader::loadTextureNoInterpolation("res/Images/Particles/SplashAtlas.png"), 4, 0.6f, 0);
	INCR_NEW textureBubble            = new ParticleTexture(Loader::loadTextureNoInterpolation("res/Images/Particles/BubbleInverseAtlas.png"), 4, 0.6f, 0);
	INCR_NEW textureExplosion1        = new ParticleTexture(Loader::loadTextureNoInterpolation("res/Images/Particles/Explosion1Atlas.png"), 4, 0.8f, 0);
	INCR_NEW textureExplosion2        = new ParticleTexture(Loader::loadTextureNoInterpolation("res/Images/Particles/Explosion2Atlas.png"), 4, 0.8f, 0);
	INCR_NEW textureExplosion3        = new ParticleTexture(Loader::loadTexture("res/Images/Particles/Explosion3Atlas.png"), 4, 0.8f, 0);
	INCR_NEW textureBlackFade         = new ParticleTexture(Loader::loadTexture("res/Images/Particles/BlackFadeAtlas.png"), 2, 1.0f, 0);
	INCR_NEW textureBlackFadeOut      = new ParticleTexture(Loader::loadTexture("res/Images/Particles/BlackFadeOutAtlas.png"), 2, 1.0f, 0);
	INCR_NEW textureTear1             = new ParticleTexture(Loader::loadTexture("res/Images/Particles/Tear1.png"), 1, 1.0f, 0);
	INCR_NEW textureTear2             = new ParticleTexture(Loader::loadTexture("res/Images/Particles/Tear2.png"), 1, 1.0f, 0);
	INCR_NEW textureWhiteFadeOutAndIn = new ParticleTexture(Loader::loadTexture("res/Images/Particles/WhiteFadeOutAndInAtlas.png"), 2, 1.0f, 0);
	INCR_NEW textureBlackFadeOutAndIn = new ParticleTexture(Loader::loadTexture("res/Images/Particles/BlackFadeOutAndInAtlas.png"), 2, 1.0f, 0);
	INCR_NEW textureInWater           = new ParticleTexture(Loader::loadTexture("res/Images/Particles/InWater.png"), 1, 0.1f, 1);
	INCR_NEW textureBlueLine          = new ParticleTexture(Loader::loadTexture("res/Images/Particles/BlueLine.png"), 1, 1.0f, 1);
}