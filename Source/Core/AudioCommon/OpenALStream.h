// Copyright 2008 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#pragma once

#include <mutex>
#include <thread>

#include "AudioCommon/SoundStream.h"
#include "Common/Common.h"
#include "Common/Event.h"
#include "Core/Core.h"
#include "Core/HW/AudioInterface.h"
#include "Core/HW/SystemTimers.h"

#if defined HAVE_OPENAL && HAVE_OPENAL
#ifdef _WIN32
#include <OpenAL/include/al.h>
#include <OpenAL/include/alc.h>
#include <OpenAL/include/alext.h>
#elif defined __APPLE__
#include <OpenAL/al.h>
#include <OpenAL/alc.h>
#else
#include <AL/al.h>
#include <AL/alc.h>
#include <AL/alext.h>
#endif
#endif

class OpenALStream final : public SoundStream
{
#if defined HAVE_OPENAL && HAVE_OPENAL
public:
  OpenALStream() : uiSource(0) {}
  bool Start() override;
  void SoundLoop() override;
  void SetVolume(int volume) override;
  void Stop() override;
  void Clear(bool mute) override;
  void Update() override;

  static bool isValid() { return true; }
private:
  std::thread thread;
  Common::Flag m_run_thread;

  std::mutex m_pause_lock;
  Common::Event soundSyncEvent;

  ALuint uiSource;
  ALfloat fVolume;
#endif  // HAVE_OPENAL
};
