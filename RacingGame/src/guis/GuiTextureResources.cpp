#include "guitextureresources.h"
#include "guitexture.h"
#include "../renderEngine/renderEngine.h"
#include "../engineTester/main.h"

GuiTexture* GuiTextureResources::textureRing        = nullptr;
GuiTexture* GuiTextureResources::textureBlueLine    = nullptr;
GuiTexture* GuiTextureResources::textureRankDisplay = nullptr;

void GuiTextureResources::loadGuiTextures()
{
	extern unsigned int SCR_WIDTH;
	extern unsigned int SCR_HEIGHT;

	float px = 1.0f/(SCR_WIDTH);  //1 pixel in x dimension
	float py = 1.0f/(SCR_HEIGHT); //1 pixel in y dimension

	const float w = 0.02f;   //width of a single text character
	const float o = 0.0008f; //horizontal offset to adjust for centered vs non centered

	INCR_NEW textureRing        = new GuiTexture(Loader::loadTexture("res/Images/Ring.png"),     o + 0.5f*w + 16*px, 0.0212f+48*py,  32*px, 32*py, 0 );
	INCR_NEW textureBlueLine    = new GuiTexture(Loader::loadTexture("res/Images/BlueLine.png"), 0.5f, 0.5f, 10*16*px, 10*128*py, 29.5f);
	INCR_NEW textureRankDisplay = new GuiTexture(0, 0.5f, 0.8f, 128*px, 128*py, 0);
}