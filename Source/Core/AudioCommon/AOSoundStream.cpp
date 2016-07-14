// Copyright 2008 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include <chrono>
#include <cstring>
#include <thread>

#include "AudioCommon/AOSoundStream.h"
#include "AudioCommon/Mixer.h"
#include "Common/MsgHandler.h"
#include "Common/Thread.h"

#if defined(HAVE_AO) && HAVE_AO
#include <ao/ao.h>

void AOSound::SoundLoop()
{
  Common::SetCurrentThreadName("Audio thread - ao");

  short realtimeBuffer[1024];

  uint_32 numBytesToRender = 256;
  unsigned int samples_to_render = numBytesToRender >> 2;
  ao_initialize();
  int default_driver = ao_default_driver_id();
  ao_sample_format format;
  format.bits = 16;
  format.channels = 2;
  format.rate = m_mixer->GetSampleRate();
  format.byte_format = AO_FMT_LITTLE;

  ao_device* device = ao_open_live(default_driver, &format, nullptr /* no options */);
  if (!device)
  {
    PanicAlertT("AudioCommon: Error opening AO device.");
    ao_shutdown();
    return;
  }

  while (m_run_thread.IsSet())
  {
    if (!m_muted)
      m_mixer->Mix(realtimeBuffer, samples_to_render);
    else
      std::memset(realtimeBuffer, 0, numBytesToRender);
    // NOTE: ao_play is synchronous. It plays the buffer in entirety before it returns.
    ao_play(device, reinterpret_cast<char*>(realtimeBuffer), numBytesToRender);
  }

  ao_close(device);
  ao_shutdown();
}

bool AOSound::Start()
{
  m_run_thread.Set();
  thread = std::thread(&AOSound::SoundLoop, this);
  return true;
}

void AOSound::Stop()
{
  m_run_thread.Clear();
  thread.join();
}

#endif
