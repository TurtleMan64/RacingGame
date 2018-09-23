//To get rid of fopen error
#ifdef _WIN32
#define _CRT_SECURE_NO_DEPRECATE
#endif

#include <AL/al.h>
#include <AL/alc.h>
#include <iostream>

#ifdef _WIN32
#include <windows.h>
#endif

#include <vorbis/vorbisfile.h>

#include "audiomaster.h"
#include "../toolbox/vector.h"
#include "source.h"
#include "audioplayer.h"
#include "../toolbox/maths.h"
#include "../engineTester/main.h"

ALCdevice*  AudioMaster::device = nullptr;
ALCcontext* AudioMaster::context = nullptr;


void AudioMaster::init()
{
	alDistanceModel(AL_LINEAR_DISTANCE_CLAMPED);
	alListenerf(AL_GAIN, 0.1f);

	AudioMaster::device = alcOpenDevice(nullptr);
	if (AudioMaster::device == nullptr)
	{
		fprintf(stderr, "no sound device\n");
		return;
	}

	AudioMaster::context = alcCreateContext(AudioMaster::device, nullptr);
	alcMakeContextCurrent(AudioMaster::context);
	if (AudioMaster::context == nullptr)
	{
		fprintf(stderr, "no sound context\n");
		return;
	}

	AudioPlayer::loadSettings();
	AudioPlayer::createSources();
	AudioPlayer::loadSoundEffects();
}

void AudioMaster::updateListenerData(Vector3f* eye, Vector3f* target, Vector3f* up, Vector3f* vel)
{
	Vector3f at(target);
	at = at - eye;
	at.normalize();

	Vector3f perpen = at.cross(up);

	float u3 = perpen.x;
	float v3 = perpen.y;
	float w3 = perpen.z;
	float x3 = at.x;
	float y3 = at.y;
	float z3 = at.z;
	float buf[3];
	Maths::rotatePoint(buf, 0, 0, 0, u3, v3, w3, x3, y3, z3, Maths::toRadians(90));

	Vector3f up2(buf[0], buf[1], buf[2]);
	up2.normalize();

	ALfloat listenerPos[] = { eye->x, eye->y, eye->z };
	ALfloat listenerVel[] = { vel->x/3.0f, vel->y/3.0f, vel->z/3.0f }; // over 3 to scale down doppler effect
	ALfloat listenerOri[] = { at.x, at.y, at.z, up2.x, up2.y, up2.z };

	alListenerfv(AL_POSITION,    listenerPos);
	alListenerfv(AL_VELOCITY,    listenerVel);
	alListenerfv(AL_ORIENTATION, listenerOri);
}

ALuint AudioMaster::loadOGG(const char* fileName)
{
	FILE* fp = nullptr;
	fp = fopen(fileName, "rb");

	if (fp == NULL)
	{
		fprintf(stdout, "Error when trying to open '%s'\n", fileName);
		return AL_NONE;
	}

	int endian = 0; //0 = little
	int bitStream;
	long bytes;
	char array[32768];

	std::vector<char> buf;

	ALuint frequency;
	ALenum format;

	vorbis_info* pInfo;
	OggVorbis_File oggFile;

	ov_open(fp, &oggFile, NULL, 0);

	pInfo = ov_info(&oggFile, -1);

	if (pInfo->channels == 1)
	{
		format = AL_FORMAT_MONO16;
	}
	else
	{
		format = AL_FORMAT_STEREO16;
	}

	frequency = pInfo->rate;

	do
	{
		bytes = ov_read(&oggFile, array, 32768, endian, 2, 1, &bitStream);
		buf.insert(buf.end(), array, array + bytes);
	}
	while (bytes > 0);

	ov_clear(&oggFile);

	ALuint buffer;
	alGenBuffers(1, &buffer);
	alBufferData(buffer, format, &buf[0], (ALsizei)buf.size(), frequency);

	buf.clear();
	buf.shrink_to_fit();

	return buffer;
}

ALuint AudioMaster::loadWAV(const char* fileName)
{
#ifdef _WIN32
	FILE* fp = nullptr;
	fp = fopen(fileName, "rb");

	if (fp == NULL)
	{
		fprintf(stdout, "Error when trying to open '%s'\n", fileName);
		fclose(fp);
		return AL_NONE;
	}

	char type[4];
	DWORD size, chunkSize;
	short formatType, channels;
	DWORD sampleRate, avgBytesPerSec;
	short bytesPerSample, bitsPerSample;
	DWORD dataSize;

	fread(type, sizeof(char), 4, fp);
	if (type[0] != 'R' ||
		type[1] != 'I' ||
		type[2] != 'F' ||
		type[3] != 'F')
	{
		fprintf(stderr, "No RIFF\n");
		fclose(fp);
		return 0;
	}

	fread(&size, sizeof(DWORD), 1, fp);
	fread(type, sizeof(char), 4, fp);
	if (type[0] != 'W' ||
		type[1] != 'A' ||
		type[2] != 'V' ||
		type[3] != 'E')
	{
		fprintf(stderr, "not WAVE\n");
		fclose(fp);
		return 0;
	}

	fread(type, sizeof(char), 4, fp);
	if (type[0] != 'f' ||
		type[1] != 'm' ||
		type[2] != 't' ||
		type[3] != ' ')
	{
		fprintf(stderr, "not fmt\n");
		fclose(fp);
		return 0;
	}

	fread(&chunkSize,      sizeof(DWORD), 1, fp);
	fread(&formatType,     sizeof(short), 1, fp);
	fread(&channels,       sizeof(short), 1, fp);
	fread(&sampleRate,     sizeof(DWORD), 1, fp);
	fread(&avgBytesPerSec, sizeof(DWORD), 1, fp);
	fread(&bytesPerSample, sizeof(short), 1, fp);
	fread(&bitsPerSample,  sizeof(short), 1, fp);

	fread(type, sizeof(char), 4, fp);
	if (type[0] != 'd' ||
		type[1] != 'a' ||
		type[2] != 't' ||
		type[3] != 'a')
	{
		fprintf(stderr, "Missing DATA\n");
		fclose(fp);
		return 0;
	}

	fread(&dataSize, sizeof(DWORD), 1, fp);

	unsigned char* buf = new unsigned char[dataSize]; INCR_NEW
	fread(buf, sizeof(BYTE), dataSize, fp);



	ALuint buffer;
	ALuint frequency = sampleRate;
	ALenum format = 0;

	alGenBuffers(1, &buffer);

	switch (bitsPerSample)
	{
	case 8:
		switch (channels)
		{
		case 1:
			format = AL_FORMAT_MONO8;
			break;

		case 2:
			format = AL_FORMAT_STEREO8;
			break;

		default:
			fprintf(stderr, "unknown sound format\n");
		}
		break;

	case 16:
		switch (channels)
		{
		case 1:
			format = AL_FORMAT_MONO16;
			break;

		case 2:
			format = AL_FORMAT_STEREO16;
			break;

		default:
			fprintf(stderr, "unknown sound format\n");
		}
		break;

	default:
		fprintf(stderr, "unknown sound format\n");
		break;
	}

	alBufferData(buffer, format, buf, dataSize, frequency);

	delete[] buf; INCR_DEL

	fclose(fp);

	return buffer;
#else
    return AL_NONE;
#endif
}

void AudioMaster::cleanUp()
{
	AudioPlayer::deleteSources();
	AudioPlayer::deleteBuffersSE();

	alcMakeContextCurrent(nullptr);
	alcDestroyContext(AudioMaster::context);
	alcCloseDevice(AudioMaster::device);
}