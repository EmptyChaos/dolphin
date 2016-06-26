// Copyright 2008 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include <algorithm>
#include <thread>
#include <unordered_set>

#include "AudioCommon/DPL2Decoder.h"
#include "AudioCommon/OpenALStream.h"
#include "AudioCommon/aldlist.h"
#include "Common/Logging/Log.h"
#include "Common/Thread.h"
#include "Core/ConfigManager.h"

#if defined HAVE_OPENAL && HAVE_OPENAL
#ifdef __APPLE__
// Avoid conflict with objc.h (on Windows, ST uses the system BOOL type, so this doesn't work)
#define BOOL SoundTouch_BOOL
#endif

#include <soundtouch/STTypes.h>
#include <soundtouch/SoundTouch.h>

#ifdef __APPLE__
#undef BOOL
#endif

// 16 bit Stereo
#define SFX_MAX_SOURCE 1
#define OAL_MAX_BUFFERS 30
#define OAL_MAX_SAMPLES 256
#define STEREO_CHANNELS 2
#define SURROUND_CHANNELS 6  // number of channels in surround mode
#define SIZE_SHORT 2
#define SIZE_FLOAT 4  // size of a float in bytes
#define FRAME_STEREO_SHORT (STEREO_CHANNELS * SIZE_SHORT)
#define FRAME_STEREO_FLOAT (STEREO_CHANNELS * SIZE_FLOAT)
#define FRAME_SURROUND_FLOAT (SURROUND_CHANNELS * SIZE_FLOAT)
#define FRAME_SURROUND_SHORT (SURROUND_CHANNELS * SIZE_SHORT)

#ifdef _WIN32
#pragma comment(lib, "openal32.lib")
#endif

//
// AyuanX: Spec says OpenAL1.1 is thread safe already
//
bool OpenALStream::Start()
{
  bool bReturn = false;

  ALDeviceList pDeviceList;
  if (pDeviceList.GetNumDevices())
  {
    char* defDevName = pDeviceList.GetDeviceName(pDeviceList.GetDefaultDevice());

    WARN_LOG(AUDIO, "Found OpenAL device %s", defDevName);

    ALCdevice* pDevice = alcOpenDevice(defDevName);
    if (pDevice)
    {
      ALCcontext* pContext = alcCreateContext(pDevice, nullptr);
      if (pContext)
      {
        // Used to determine an appropriate period size (2x period = total buffer size)
        // ALCint refresh;
        // alcGetIntegerv(pDevice, ALC_REFRESH, 1, &refresh);
        // period_size_in_millisec = 1000 / refresh;

        alcMakeContextCurrent(pContext);
        m_run_thread.Set();
        thread = std::thread(&OpenALStream::SoundLoop, this);
        bReturn = true;
      }
      else
      {
        alcCloseDevice(pDevice);
        PanicAlertT("OpenAL: can't create context for device %s", defDevName);
      }
    }
    else
    {
      PanicAlertT("OpenAL: can't open device %s", defDevName);
    }
  }
  else
  {
    PanicAlertT("OpenAL: can't find sound devices");
  }

  // Initialize DPL2 parameters
  DPL2Reset();

  return bReturn;
}

void OpenALStream::Stop()
{
  m_run_thread.Clear();
  // kick the thread if it's waiting
  soundSyncEvent.Set();

  thread.join();

  ALCcontext* pContext = alcGetCurrentContext();
  ALCdevice* pDevice = alcGetContextsDevice(pContext);

  alcMakeContextCurrent(nullptr);
  alcDestroyContext(pContext);
  alcCloseDevice(pDevice);
}

void OpenALStream::SetVolume(int volume)
{
  fVolume = (float)volume / 100.0f;

  if (uiSource)
    alSourcef(uiSource, AL_GAIN, fVolume);
}

void OpenALStream::Update()
{
  soundSyncEvent.Set();
}

void OpenALStream::Clear(bool mute)
{
  std::lock_guard<std::mutex> guard(m_pause_lock);
  m_muted = mute;

  if (m_muted)
  {
    alSourceStop(uiSource);
  }
  else
  {
    alSourcePlay(uiSource);
  }
}

void OpenALStream::SoundLoop()
{
  Common::SetCurrentThreadName("SoundStream - OpenAL");

  short realtimeBuffer[OAL_MAX_SAMPLES * STEREO_CHANNELS] = {0};
  soundtouch::SAMPLETYPE sampleBuffer[OAL_MAX_SAMPLES * SURROUND_CHANNELS] = {0};
  ALuint uiBuffers[OAL_MAX_BUFFERS] = {0};

  bool surround_capable = SConfig::GetInstance().bDPL2Decoder;
#if defined(__APPLE__)
  bool float32_capable = false;
  const ALenum AL_FORMAT_STEREO_FLOAT32 = 0;
  // OS X does not have the alext AL_FORMAT_51CHN32 yet.
  surround_capable = false;
  const ALenum AL_FORMAT_51CHN32 = 0;
  const ALenum AL_FORMAT_51CHN16 = 0;
#else
  bool float32_capable = true;

  // Checks if a X-Fi is being used. If it is, disable FLOAT32 support as this sound card has no
  // support for it even though it reports it does.
  if (strstr(alGetString(AL_RENDERER), "X-Fi"))
    float32_capable = false;
#endif

  u32 ulFrequency = m_mixer->GetSampleRate();
  // OpenAL requires a minimum of two buffers
  unsigned int numBuffers = std::min(OAL_MAX_BUFFERS, SConfig::GetInstance().iLatency + 2);
  std::unordered_set<ALuint> buffers_active(numBuffers * 2);
  buffers_active.max_load_factor(0.5f);

  // Generate some AL Buffers for streaming
  alGenBuffers(numBuffers, uiBuffers);
  // Generate a Source to playback the Buffers
  uiSource = 0;
  alGenSources(1, &uiSource);

  // Set the default sound volume as saved in the config file.
  alSourcef(uiSource, AL_GAIN, fVolume);

  // TODO: Error handling
  // ALenum err = alGetError();

  ALuint uiBufferTemp[OAL_MAX_BUFFERS] = {0};
  ALint iState = 0;

  soundtouch::SoundTouch soundTouch;
  soundTouch.setChannels(2);
  soundTouch.setSampleRate(ulFrequency);
  soundTouch.setTempo(1.0);
  soundTouch.setSetting(SETTING_USE_QUICKSEEK, 0);
  soundTouch.setSetting(SETTING_USE_AA_FILTER, 0);
  // SeekWindow = max(SEQUENCE_MS, OVERLAP_MS * 2)
  // Latency = max(tempo * (SeekWindow - OVERLAP_MS) + OVERLAP_MS, SeekWindow) + SEEKWINDOW_MS
  // SEQUENCE = 1, SEEKWINDOW = 28, OVERLAP = 12
  // e.g. Tempo = 2.0, max(2 * (48 - 12) + 12, 48) + 28 = 112ms latency
  // Tempo = 1.0, max(36, 48) + 28 = 76ms latency
  // Tempo = 0.5, max(18, 48) + 28 = 76ms latency
  // Recommended value for Overlap is 8ms.
  // Recommended value for SeekWindow is 15-25ms.
  // Minimum value for Sequence is 1ms.
  // NOTE: All above values are strictly for SoundTouch itself. It doesn't include the
  //   latency of getting data into an OpenAL buffer, and the OpenAL buffer making it
  //   through the system to the speakers. Minimum total buffer size seems to be 5ms
  //   for a fast system to prevent constant underflows with slower systems likely
  //   needing more. Determining latency from buffer submission to speakers would
  //   require the user to perform a listening test.
  //   CMixer doesn't let us query the amount of back-fill in the FIFOs so that's also
  //   additional untracked latency.
  soundTouch.setSetting(SETTING_SEQUENCE_MS, 1);
  soundTouch.setSetting(SETTING_SEEKWINDOW_MS, 28);
  soundTouch.setSetting(SETTING_OVERLAP_MS, 12);

  while (m_run_thread.IsSet())
  {
    // num_samples_to_render in this update - depends on SystemTimers::AUDIO_DMA_PERIOD.
    static constexpr u32 DMA_LENGTH = 32;
    static constexpr u32 AIS_SAMPLES_PER_SECOND = 48000;
    // NOTE: AudioInterface can be changed by the CPU at any time, it doesn't make
    //   sense to access it from here - the value is invalid the moment after we read it.
    // NOTE: Value is either 32 (AIDSampleRate = 48000) or 48 (32000).
    u32 num_samples_to_render =
        (AIS_SAMPLES_PER_SECOND * DMA_LENGTH) / AudioInterface::GetAIDSampleRate();

    unsigned int numSamples = (unsigned int)num_samples_to_render;
    unsigned int minSamples =
        surround_capable ? 240 : 32;  // DPL2 accepts 240 samples minimum (FWRDURATION)

    numSamples = (numSamples > OAL_MAX_SAMPLES) ? OAL_MAX_SAMPLES : numSamples;
    // NOTE: This is a race. The stream tries to pull audio from the mixer at random
    //   intervals and the mixer has no internal synchronization so we may pull nothing,
    //   or partial data from FIFOs. i.e. we'll pull AID before the AIS has been written
    //   causing a desync and increasing the DTK latency, etc. Under the worst case we
    //   can experience an oscillation where we pull 2ms of AID with 0ms of AIS, then
    //   2ms of AIS and 0ms of AID resulting in "half-speed" audio that sounds terrible
    //   because of the mixer's "padding" algorithm inserting a flat tone at a random
    //   pitch.
    numSamples = m_mixer->Mix(realtimeBuffer, numSamples, false);

    // Convert the samples from short to float
    float dest[OAL_MAX_SAMPLES * STEREO_CHANNELS];
    for (u32 i = 0; i < numSamples * STEREO_CHANNELS; ++i)
      dest[i] = (float)realtimeBuffer[i] / (1 << 15);

    double rate = m_mixer->GetCurrentSpeed();
    if (rate <= 0)
    {
      // NOTE: This is asynchronous. It won't be updated until some point later.
      Core::RequestRefreshInfo();
      rate = 1;
    }

    // SoundTouch can only operate between -99% and 1000% before it stops functioning
    // correctly and produces garbage.
    if (rate < 0.01 || rate > 10)
    {
      soundTouch.clear();
      continue;
    }
    soundTouch.setTempo(rate);
    // NOTE: SoundTouch computes output once the buffer reaches the input latency
    //   threshold (i.e. latency of 72ms means it begins processing when 72ms of
    //   input audio is queued). Conversion happens in putSamples so all parameters
    //   must be configured before risking crossing the boundary line.
    soundTouch.putSamples(dest, numSamples);

    if (!buffers_active.empty())
    {
      int num_buffers_done = 0;
      alGetSourcei(uiSource, AL_BUFFERS_PROCESSED, &num_buffers_done);
      if (num_buffers_done)
      {
        uiBufferTemp[0] = 0;
        alSourceUnqueueBuffers(uiSource, num_buffers_done, uiBufferTemp);
        if (uiBufferTemp[0])
        {
          for (int i = 0; i < num_buffers_done; ++i)
          {
            buffers_active.erase(uiBufferTemp[i]);
          }
        }
      }
    }
    if (buffers_active.size() == numBuffers)
    {
      // NOTE: This event is set by AID after it writes data. If AID is disabled
      //   so only DTK is used then this will never be set; thus, it tells us
      //   nothing about AIS/DTK. We have no way of knowing when or if the AIS/DTK
      //   has been written so we get to data race that system and pray it works.
      soundSyncEvent.Wait();
      continue;
    }

    // If we are 1000ms behind the emulator then just throw everything away.
    // 1sec latency is too far. This typically happens during boot when the
    // emulator is running at 1fps while the JIT compiles the first wave of
    // blocks.
    if (soundTouch.numSamples() > AIS_SAMPLES_PER_SECOND)
    {
      unsigned int samples = soundTouch.numSamples();
      unsigned int buffer_size_with_rate =
          static_cast<unsigned int>(numBuffers * OAL_MAX_SAMPLES / rate);
      if (buffer_size_with_rate < samples)
      {
        soundTouch.receiveSamples(soundTouch.numSamples() - buffer_size_with_rate);
        WARN_LOG(AUDIO, "Buffer discard: %u -> %u", samples, soundTouch.numSamples());
      }
    }
    else if (soundTouch.numSamples() < minSamples)
    {
      // NOTE: Do not try to sleep on soundSyncEvent, it will stall out for too
      //   long and underflow.
      continue;
    }

    while (soundTouch.numSamples() >= minSamples && buffers_active.size() != numBuffers)
    {
      unsigned int nSamples = soundTouch.receiveSamples(sampleBuffer, OAL_MAX_SAMPLES);

      ALuint empty_buffer = 0;
      for (unsigned int i = 0; i < numBuffers; ++i)
      {
        if (!buffers_active.count(uiBuffers[i]))
        {
          empty_buffer = uiBuffers[i];
          break;
        }
      }

      if (surround_capable)
      {
        float dpl2[OAL_MAX_SAMPLES * SURROUND_CHANNELS];
        DPL2Decode(sampleBuffer, nSamples, dpl2);

        // zero-out the subwoofer channel - DPL2Decode generates a pretty
        // good 5.0 but not a good 5.1 output.  Sadly there is not a 5.0
        // AL_FORMAT_50CHN32 to make this super-explicit.
        // DPL2Decode output: LEFTFRONT, RIGHTFRONT, CENTREFRONT, (sub), LEFTREAR, RIGHTREAR
        for (u32 i = 0; i < nSamples; ++i)
        {
          dpl2[i * SURROUND_CHANNELS + 3 /*sub/lfe*/] = 0.0f;
        }

        if (float32_capable)
        {
          alBufferData(empty_buffer, AL_FORMAT_51CHN32, dpl2, nSamples * FRAME_SURROUND_FLOAT,
                       ulFrequency);
        }
        else
        {
          short surround_short[OAL_MAX_SAMPLES * SURROUND_CHANNELS];
          for (u32 i = 0; i < nSamples * SURROUND_CHANNELS; ++i)
            surround_short[i] = (short)((float)dpl2[i] * (1 << 15));

          alBufferData(empty_buffer, AL_FORMAT_51CHN16, surround_short,
                       nSamples * FRAME_SURROUND_SHORT, ulFrequency);
        }

        ALenum err = alGetError();
        if (err == AL_INVALID_ENUM)
        {
          // 5.1 is not supported by the host, fallback to stereo
          WARN_LOG(AUDIO,
                   "Unable to set 5.1 surround mode.  Updating OpenAL Soft might fix this issue.");
          surround_capable = false;
        }
        else if (err != 0)
        {
          ERROR_LOG(AUDIO, "Error occurred while buffering data: %08x", err);
        }
      }
      else
      {
        if (float32_capable)
        {
          alBufferData(empty_buffer, AL_FORMAT_STEREO_FLOAT32, sampleBuffer,
                       nSamples * FRAME_STEREO_FLOAT, ulFrequency);
          ALenum err = alGetError();
          if (err == AL_INVALID_ENUM)
          {
            float32_capable = false;
          }
          else if (err != 0)
          {
            ERROR_LOG(AUDIO, "Error occurred while buffering float32 data: %08x", err);
          }
        }

        else
        {
          // Convert the samples from float to short
          short stereo[OAL_MAX_SAMPLES * STEREO_CHANNELS];
          for (u32 i = 0; i < nSamples * STEREO_CHANNELS; ++i)
            stereo[i] = (short)((float)sampleBuffer[i] * (1 << 15));

          alBufferData(empty_buffer, AL_FORMAT_STEREO16, stereo, nSamples * FRAME_STEREO_SHORT,
                       ulFrequency);
        }
      }

      alSourceQueueBuffers(uiSource, 1, &empty_buffer);
      ALenum err = alGetError();
      if (err == AL_NO_ERROR)
      {
        buffers_active.insert(empty_buffer);
      }
      else
      {
        ERROR_LOG(AUDIO, "Error queuing buffers: %08x", err);
      }
    }

    alGetSourcei(uiSource, AL_SOURCE_STATE, &iState);
    if (iState != AL_PLAYING)
    {
      // Buffer underrun occurred, resume playback
      std::lock_guard<std::mutex> pause_lock(m_pause_lock);
      // Wait until ALL buffers are full (i.e. maximum latency attained)
      // before beginning to play otherwise we'll probably just underflow
      // again immediately because each individual buffer is really small.
      if (!m_muted && buffers_active.size() == numBuffers)
      {
        alSourcePlay(uiSource);
        ALenum err = alGetError();
        if (err != 0)
        {
          ERROR_LOG(AUDIO, "Error occurred resuming playback: %08x", err);
        }
      }
    }
  }

  // Clean up buffers
  alSourceStop(uiSource);
  alSourceUnqueueBuffers(uiSource, numBuffers, uiBufferTemp);
  alSourcei(uiSource, AL_BUFFER, 0);
  alDeleteBuffers(numBuffers, uiBuffers);

  // Cleanup Source
  alDeleteSources(1, &uiSource);
  uiSource = 0;
}

#endif  // HAVE_OPENAL
