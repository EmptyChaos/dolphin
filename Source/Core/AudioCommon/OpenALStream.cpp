// Copyright 2008 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include <algorithm>
#include <bitset>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

#include "AudioCommon/DPL2Decoder.h"
#include "AudioCommon/OpenALStream.h"
#include "AudioCommon/aldlist.h"
#include "Common/Assert.h"
#include "Common/BitSet.h"
#include "Common/Logging/Log.h"
#include "Common/MathUtil.h"
#include "Common/ScopeGuard.h"
#include "Common/Thread.h"
#include "Core/ConfigManager.h"

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

#ifdef __APPLE__
// Avoid conflict with objc.h (on Windows, ST uses the system BOOL type, so this doesn't work)
#define BOOL SoundTouch_BOOL
#endif

#include <soundtouch/FIFOSampleBuffer.h>
#include <soundtouch/STTypes.h>
#include <soundtouch/SoundTouch.h>

#ifdef __APPLE__
#undef BOOL
#endif

#ifdef _WIN32
#pragma comment(lib, "openal32.lib")
#endif

// OS X lacks support for Surround and Float32.
#ifndef AL_FORMAT_51CHN16
#define AL_FORMAT_51CHN16 0
#endif
#ifndef AL_FORMAT_51CHN32
#define AL_FORMAT_51CHN32 0
#endif
#ifndef AL_FORMAT_STEREO_FLOAT32
#define AL_FORMAT_STEREO_FLOAT32 0
#endif

static constexpr int STEREO_CHANNELS = 2;
static constexpr int SURROUND51_CHANNELS = 6;
static constexpr ALsizei FRAME_STEREO_F32 = STEREO_CHANNELS * sizeof(float);
static constexpr ALsizei FRAME_STEREO_S16 = STEREO_CHANNELS * sizeof(s16);
static constexpr ALsizei FRAME_SURROUND51_F32 = SURROUND51_CHANNELS * sizeof(float);
static constexpr ALsizei FRAME_SURROUND51_S16 = SURROUND51_CHANNELS * sizeof(s16);

template <class T, class A>
void EnsureCapacity(std::vector<T, A>* vec, std::size_t units)
{
  if (vec->size() < units)
    vec->resize(units);
}

template <class T, class A>
void EnsureCapacityBytes(std::vector<T, A>* vec, std::size_t bytes)
{
  EnsureCapacity(vec, (bytes + sizeof(T) - 1) / sizeof(T));
}

// Architecture Notes:
//  We constantly move data out of CMixer into m_holding_fifo for later
//  processing. We don't put the data directly into SoundTouch because
//  SoundTouch converts audio immediately in putSamples(); we need to
//  hold the audio until we know the desired tempo setting then we dump
//  the relevant part of the holding FIFO into SoundTouch to get a
//  single OpenAL buffer of output.
//
//  We use a holding FIFO instead of just letting audio build up in the
//  CMixer because CMixer uses multiple ring buffers in the source
//  sample rate, this means there is no coherent way to determine the
//  *output* sample count. If we tried to guess the output sample count
//  then that would be invalidated by the sample rate being changed by
//  the CPU at any time. We need to get the audio out of CMixer in order
//  to render it immutable.
//
// Rate Control:
//  We infer the emulation speed from the size of the holding FIFO. This
//  is the most precise way to do it since we only care about audio lag,
//  not the emulation FPS. The method is to simply predict how much
//  audio we should have (i.e. at 100% speed the holding FIFO should
//  contain the same number of samples as the OpenAL buffer size due to
//  there being 1-1 time/size correspondence with audio samples) then
//  comparing that to the actual sample count. If the FIFO has more,
//  then emulation is over 100% and we need to compress the held samples
//  (time stretch to be shorter), if we have less, then we are sub-100%
//  and need to time stretch to be longer.
class OpenALStream::WorkerThread final
{
public:
  WorkerThread()
  {
    m_sound_touch.setChannels(2);
    m_sound_touch.setSetting(SETTING_USE_AA_FILTER, 0);  // We aren't resampling.
    m_sound_touch.setSetting(SETTING_USE_QUICKSEEK, 0);  // This is broken. It segfaults.
  }
  ~WorkerThread();

  bool Start(CMixer* mixer, unsigned int channels = STEREO_CHANNELS);

  void Pause(bool paused = true);
  void Play() { Pause(false); }
  void SetVolume(float volume);

private:
  using ALCcontextPtr = std::unique_ptr<ALCcontext, decltype(&alcDestroyContext)>;
  using ALCdevicePtr = std::unique_ptr<ALCdevice, decltype(&alcCloseDevice)>;

  enum class SoundTouchProcess
  {
    Normal,
    None,
    Failed
  };

  // [THREAD] Thread entrypoint.
  void RunLoop();

  // [THREAD] Checks if SConfig::iLatency changed and reconfigures state appropriately
  // May resize m_processing_buffer and change m_openal_buffer_size
  void PollForConfigurationChanges(bool force = false);

  // [THREAD] Move samples out of CMixer into m_holding_fifo (converts to float)
  void DrainMixer();

  // [THREAD] Converts Float32 Stereo into Float32 5.1 Surround using Dolby DPL2.
  // Output buffer must be (size * 3) bytes.
  ALsizei ConvertToSurround51(float* output, float* stereo_source, ALsizei size);

  // [THREAD] Converts the given array of float32 samples to the OpenAL input format.
  // Size is in bytes. Output is in m_format_conversion_buffer.
  // Return value is adjusted number of bytes.
  // Clobbers m_dpl2_buffer.
  ALsizei ConvertToOutputFormat(float* source, ALsizei size);

  // [THREAD] Configures SoundTouch to have the given processing latency.
  // Called during initial setup and whenever the SConfig::iLatency value changes.
  void ConfigureSoundTouch(std::chrono::milliseconds latency_ms);

  // [THREAD] Computes the latency in samples from the current SoundTouch settings.
  unsigned int ComputeSoundTouchLatency(double tempo = 1.0) const;

  // [THREAD] Configures the SoundTouch Tempo to produce output_frames from
  // input_frames. Includes accounting for latency increases as tempo increases.
  // e.g. in=500, out=1000 -> Tempo=0.5; in=500, out=100 -> Tempo=5.0
  // Returns Failed if SoundTouch cannot provide any useful output.
  // Returns None if there is too much output audio already queued for consumption.
  SoundTouchProcess SetSoundTouchTempo(u32 input_frames, u32 output_frames);

  // [THREAD] Reconfigures SoundTouch then moves m_holding_fifo into it.
  // Returns false if the required tempo to convert the holding buffer is out of
  // range for SoundTouch to process. SoundTouch will be emptied.
  // If it returns true then m_sound_touch.numSamples() will contain approximately
  // m_openal_buffer_size samples of output.
  // NOTE: m_holding_fifo may not be empty
  bool GenerateOutputBuffer();

  // [THREAD] Writes the given buffer into the given OpenAL buffer and queues it to
  // the source.
  // [LOCKED] Requires m_openal_lock
  void WriteOpenALBuffer(ALuint buffer, void* source, ALsizei bytes);

  // [THREAD] Checks OpenAL for completed buffers and updates the tracking variables.
  // [LOCKED] Requires m_openal_lock
  // Returns true if there are empty buffers that need refilling.
  // Returns false if all buffers are still active and being played.
  bool PollOpenALBufferStates();

  // [THREAD] Returns the next empty OpenAL buffer that needs to be filled.
  // Returns 0 if all buffers are still queued.
  ALuint GetNextEmptyOpenALBuffer();

  // [THREAD] Checks if OpenAL playback underrun and needs to be resumed.
  // [LOCKED] Requires m_openal_lock
  // NOTE: When an underrun happens, it will not actually resume playback
  //   until NUM_BUFFERS OpenAL buffers have been queued (See WriteOpenALBuffer)
  void CheckForUnderrun();

  // SoundTouch configuration parameters.
  static constexpr int ST_MIN_SEQUENCE = 1;
  static constexpr int ST_MIN_SEEKWINDOW = 15;
  static constexpr int ST_MAX_SEEKWINDOW = 25;
  static constexpr int ST_MIN_OVERLAP = 2;
  static constexpr int ST_MAX_OVERLAP = 8;
  static constexpr int ST_MIN_LATENCY = 19;
  static constexpr double ST_MIN_TEMPO = 0.01;  // SoundTouch's algorithm limits
  static constexpr double ST_MAX_TEMPO = 10.0;

  static constexpr int MAX_LATENCY_MS = 1000;
  static constexpr unsigned int NUM_BUFFERS = 2;

  // Required by DPL2. We multiply it by two because SoundTouch will not always
  // fill the buffer completely to full, it typically only manages around 90%.
  static constexpr int MIN_OPENAL_BUFFER_FRAMES = 240 * 2;

  // OpenAL lock protects OpenAL state (all usage of al* functions).
  // OpenAL is threadsafe but that just means that calling APIs from multiple
  // threads won't break things, not that the result will make sense at all.
  // The main problem is corrupting alGetError() since that is global, not TLS.
  std::mutex m_openal_lock;
  std::condition_variable m_unpaused_cvar;  // m_is_paused changes to false.

  std::thread m_thread;
  std::atomic<bool> m_thread_run{false};
  bool m_is_paused = false;  // Requires m_openal_lock

  ALCdevicePtr m_device{nullptr, alcCloseDevice};
  ALCcontextPtr m_context{nullptr, alcDestroyContext};
  CMixer* m_mixer = nullptr;

  ALuint m_source = 0;
  ALuint m_buffers[NUM_BUFFERS] = {0};
  std::bitset<NUM_BUFFERS> m_buffers_active;
  ALenum m_openal_format = 0;

  std::chrono::milliseconds m_last_latency{0};
  std::chrono::milliseconds m_last_canon_latency{0};
  std::chrono::milliseconds m_min_openal_buffer_ms{0};
  unsigned int m_openal_buffer_size = 0;  // Audio Frames

  soundtouch::SoundTouch m_sound_touch;

  soundtouch::FIFOSampleBuffer m_holding_fifo{STEREO_CHANNELS};
  std::vector<float> m_processing_buffer;
  std::vector<float> m_dpl2_buffer;
  std::vector<char> m_format_conversion_buffer;
};

// WorkerThread needs to be visible to instantiate unique_ptr's destructor.
OpenALStream::OpenALStream() = default;
OpenALStream::~OpenALStream() = default;

bool OpenALStream::Start()
{
  if (m_worker)
    return false;

  auto worker = std::make_unique<WorkerThread>();
  unsigned int channels =
      SConfig::GetInstance().bDPL2Decoder ? SURROUND51_CHANNELS : STEREO_CHANNELS;
  if (!worker->Start(m_mixer.get(), channels))
    return false;

  worker->SetVolume(SConfig::GetInstance().m_Volume / 100.f);

  m_worker = std::move(worker);
  return true;
}

void OpenALStream::Stop()
{
  m_worker.reset();
}

void OpenALStream::SetVolume(int volume)
{
  if (m_worker)
    m_worker->SetVolume(volume / 100.f);
}

void OpenALStream::Update()
{
  // Nothing.
}

void OpenALStream::Clear(bool mute)
{
  if (m_worker)
    m_worker->Pause(mute);
}

bool OpenALStream::WorkerThread::Start(CMixer* mixer, unsigned int num_channels)
{
  if (m_thread.joinable())
    return false;

  unsigned int sample_rate = mixer->GetSampleRate();

  ALDeviceList device_list;
  if (!device_list.GetNumDevices())
  {
    PanicAlertT("OpenAL: Can't find any sound devices");
    return false;
  }

  const char* default_device_name = device_list.GetDeviceName(device_list.GetDefaultDevice());
  WARN_LOG(AUDIO, "Found OpenAL device \"%s\"", default_device_name);

  ALCdevicePtr device{alcOpenDevice(default_device_name), &alcCloseDevice};
  if (!device)
  {
    PanicAlertT("OpenAL: Can't open device \"%s\" (%08X)", default_device_name,
                alcGetError(nullptr));
    return false;
  }

  // NOTE: This actually makes more sense to do the other way around, i.e.
  //  use the output mix frequency on our internal CMixer to minimize the
  //  amount of resampling passes that need to happen.
  ALCint attr_list[] = {ALC_FREQUENCY, static_cast<ALCint>(sample_rate), 0};

  ALCcontextPtr context{alcCreateContext(device.get(), attr_list), &alcDestroyContext};
  if (!context)
  {
    context.reset(alcCreateContext(device.get(), nullptr));
    if (!context)
    {
      PanicAlertT("OpenAL: Can't create context for device \"%s\" (%08X)", default_device_name,
                  alcGetError(device.get()));
      return false;
    }
    ALCint freq = 0;
    alcGetIntegerv(device.get(), ALC_FREQUENCY, 1, &freq);
    WARN_LOG(AUDIO, "OpenAL: Could not set Device frequency. Using default frequency %d.", freq);
  }

  if (!alcMakeContextCurrent(context.get()))
  {
    PanicAlertT("OpenAL: Failed to make context current: %08X", alcGetError(nullptr));
    return false;
  }
  Common::ScopeGuard context_reset([] { alcMakeContextCurrent(nullptr); });

  alGenBuffers(NUM_BUFFERS, m_buffers);
  ALenum err = alGetError();
  if (err != AL_NO_ERROR)
  {
    PanicAlertT("OpenAL: Could not create audio buffers: %08X", err);
    return false;
  }
  Common::ScopeGuard buffer_release([&] {
    alDeleteBuffers(NUM_BUFFERS, m_buffers);
    std::fill_n(m_buffers, NUM_BUFFERS, 0);
  });

  alGenSources(1, &m_source);
  err = alGetError();
  if (err != AL_NO_ERROR)
  {
    PanicAlertT("OpenAL: Could not create audio source: %08X", err);
    return false;
  }
  Common::ScopeGuard source_release([&] {
    alDeleteSources(1, &m_source);
    m_source = 0;
  });

  // Feature Tests.
  // NOTE: Keep these sorted from most preferred to least preferred.
  bool supports_float32 = false;
  m_openal_format = 0;
#if AL_FORMAT_STEREO_FLOAT32
  {
    std::string renderer{alGetString(AL_RENDERER)};

    // Checks if an X-Fi is being used. If it is, disable FLOAT32 support as this
    // sound card has no support for it even though it reports it does.
    supports_float32 = renderer.find("X-Fi") == std::string::npos;
    WARN_LOG(AUDIO, "OpenAL Renderer: %s (Float32: %s)", renderer.c_str(),
             supports_float32 ? "yes" : "no");
  }
#endif
#if AL_FORMAT_51CHN32
  if (!m_openal_format && supports_float32 && num_channels >= SURROUND51_CHANNELS)
  {
    float silence[32 * SURROUND51_CHANNELS] = {0.f};
    alBufferData(m_buffers[0], AL_FORMAT_51CHN32, silence, sizeof(silence), sample_rate);
    if (alGetError() != AL_INVALID_ENUM)
      m_openal_format = AL_FORMAT_51CHN32;
    else
      WARN_LOG(AUDIO, "OpenAL: Unable to render 5.1 Surround in Float32");
  }
#endif
#if AL_FORMAT_51CHN16
  if (!m_openal_format && num_channels >= SURROUND51_CHANNELS)
  {
    short silence[32 * SURROUND51_CHANNELS] = {0};
    alBufferData(m_buffers[0], AL_FORMAT_51CHN16, silence, sizeof(silence), sample_rate);
    if (alGetError() != AL_INVALID_ENUM)
      m_openal_format = AL_FORMAT_51CHN16;
    else
      WARN_LOG(AUDIO, "OpenAL: Unable to render 5.1 Surround");
  }
#endif
#if AL_FORMAT_STEREO_FLOAT32
  if (!m_openal_format && supports_float32)
  {
    float silence[32 * STEREO_CHANNELS] = {0.f};
    alBufferData(m_buffers[0], AL_FORMAT_STEREO_FLOAT32, silence, sizeof(silence), sample_rate);
    if (alGetError() != AL_INVALID_ENUM)
      m_openal_format = AL_FORMAT_STEREO_FLOAT32;
    else
      WARN_LOG(AUDIO, "OpenAL: Unable to render Float32 samples");
  }
#endif
  if (!m_openal_format)
    m_openal_format = AL_FORMAT_STEREO16;

  // Log the OpenAL interface latency since it affects the user configurable latency.
  {
    ALCint refresh = 1;
    alcGetIntegerv(device.get(), ALC_REFRESH, 1, &refresh);
    WARN_LOG(AUDIO, "OpenAL: Refresh %dHz. Minimum OpenAL Latency: %.2f", refresh,
             1000.0 / refresh);
  }

  // Initialize DPL2
  // TODO: DPL2 Encoder should be a class so we can just create an instance of it.
  DPL2Reset();

  // Actually start the thread.
  m_mixer = mixer;
  m_min_openal_buffer_ms = std::chrono::milliseconds(
      (MIN_OPENAL_BUFFER_FRAMES * NUM_BUFFERS * 1000 + sample_rate - 1) / sample_rate);
  m_sound_touch.setSampleRate(sample_rate);
  m_thread_run.store(true, std::memory_order_relaxed);
  m_thread = std::thread(&WorkerThread::RunLoop, this);

  // Release ownership of everything, it's now owned by the class instance and
  // will be cleaned up by Stop().
  source_release.Dismiss();
  buffer_release.Dismiss();
  context_reset.Dismiss();
  m_context = std::move(context);
  m_device = std::move(device);

  return true;
}

OpenALStream::WorkerThread::~WorkerThread()
{
  if (!m_thread.joinable())
    return;

  m_thread_run.store(false, std::memory_order_relaxed);
  {
    std::lock_guard<std::mutex> openal_lock(m_openal_lock);
    m_unpaused_cvar.notify_one();
  }
  m_thread.join();

  // Release OpenAL resources.
  alSourceStop(m_source);
  alSourcei(m_source, AL_BUFFER, 0);
  alDeleteBuffers(NUM_BUFFERS, m_buffers);
  alDeleteSources(1, &m_source);

  // Release OpenAL context.
  alcMakeContextCurrent(nullptr);
  // Destructors for unique_ptr destroy context and device.
}

void OpenALStream::WorkerThread::Pause(bool paused)
{
  std::lock_guard<std::mutex> guard(m_openal_lock);
  if (m_is_paused == paused)
    return;

  m_is_paused = paused;
  if (paused)
  {
    alSourceStop(m_source);
  }
  else
  {
    alSourcePlay(m_source);
    ALenum err = alGetError();
    if (err != AL_NO_ERROR)
      ERROR_LOG(AUDIO, "OpenAL: Source failed to resume playing: %08X", err);
    m_unpaused_cvar.notify_one();
  }
}

void OpenALStream::WorkerThread::SetVolume(float volume)
{
  std::lock_guard<std::mutex> guard(m_openal_lock);
  alSourcef(m_source, AL_GAIN, MathUtil::Clamp(volume, 0.f, 1.f));
}

void OpenALStream::WorkerThread::RunLoop()
{
  Common::SetCurrentThreadName("Audio: OpenALStream");
  PollForConfigurationChanges(true);

  while (m_thread_run.load(std::memory_order_relaxed))
  {
    // Transfer CMixer buffer into m_holding_fifo
    DrainMixer();

    // Check for latency configuration changes
    PollForConfigurationChanges();

    // Begin OpenAL update.
    std::unique_lock<std::mutex> openal_lock(m_openal_lock);

    // We start by checking if we are paused and suspend execution.
    if (m_is_paused)
    {
      m_unpaused_cvar.wait(openal_lock, [&] {
        return !m_thread_run.load(std::memory_order_relaxed) || !m_is_paused;
      });
      continue;
    }

    // Check if OpenAL has finished any buffers.
    if (!PollOpenALBufferStates())
    {
      openal_lock.unlock();
      // All buffers are still playing so we just sleep and try again later.
      // We sleep for 1/4 of a buffer length (we need to keep polling for completion)
      // or 8ms (1/2 frame@60FPS), whichever is smaller.
      // The reason for the 8ms limit is because we need to keep draining CMixer at
      // regular intervals. 8ms@48kHz@1000% = 3840 samples, CMixer's buffer limit
      // is 4096 samples.
      static constexpr std::intmax_t SLEEP_LIMIT_MS = 8;
      std::intmax_t sleep_time =
          m_openal_buffer_size * std::milli::den / (4 * m_mixer->GetSampleRate());
      std::this_thread::sleep_for(std::chrono::milliseconds(std::min(sleep_time, SLEEP_LIMIT_MS)));
      continue;
    }

    // We have empty buffers, so fill one.
    openal_lock.unlock();

    // Run the pending audio through SoundTouch. This can take a while, if the latency is too low
    // for the Host CPU then we may underrun due to the computation latency.
    ALsizei buffer_size = 0;
    if (GenerateOutputBuffer())
    {
      // Transfer out of SoundTouch into the processing buffer
      buffer_size = m_sound_touch.receiveSamples(m_processing_buffer.data(), m_openal_buffer_size);
      buffer_size *= FRAME_STEREO_F32;
    }
    else
    {
      // SoundTouch failed to produce usable output so we'll just play silence instead.
      std::fill(m_processing_buffer.begin(), m_processing_buffer.end(), 0.f);
      buffer_size = m_openal_buffer_size * FRAME_STEREO_F32;
    }
    _assert_(buffer_size / FRAME_STEREO_F32 >= MIN_OPENAL_BUFFER_FRAMES);

    // Do we need a format conversion?
    void* buffer = m_processing_buffer.data();
    if (m_openal_format != AL_FORMAT_STEREO_FLOAT32)
    {
      buffer_size = ConvertToOutputFormat(m_processing_buffer.data(), buffer_size);
      buffer = m_format_conversion_buffer.data();
    }

    // Transfer the buffer to OpenAL
    openal_lock.lock();
    WriteOpenALBuffer(GetNextEmptyOpenALBuffer(), buffer, buffer_size);
    CheckForUnderrun();
  }
}

void OpenALStream::WorkerThread::PollForConfigurationChanges(bool force)
{
  using std::chrono::milliseconds;

  milliseconds sconfig_latency(SConfig::GetInstance().iLatency);
  if (!force && sconfig_latency == m_last_latency)
    return;
  m_last_latency = sconfig_latency;

  // The minimum OpenAL buffer latency has to be more than ALC_REFRESH since
  // we only get "buffer finished" updates at the polling interval.
  // e.g. Refresh of 50Hz with 10ms buffers means we play 20ms (2 buffers)
  // before we learn that the first buffer has finished. Since NUM_BUFFERS
  // is 2, that is going to guarantee we underrun constantly.
  milliseconds min_latency_ms(ST_MIN_LATENCY);
  milliseconds min_openal_buffer_ms;
  {
    ALCint refresh = 50;  // Hz
    alcGetIntegerv(m_device.get(), ALC_REFRESH, 1, &refresh);
    min_openal_buffer_ms = milliseconds((std::milli::den * NUM_BUFFERS + refresh - 1) / refresh);
    min_openal_buffer_ms = std::max(min_openal_buffer_ms, m_min_openal_buffer_ms);
    min_latency_ms += min_openal_buffer_ms;
  }

  // Adjust configured latency according to the limits.
  milliseconds latency_ms =
      MathUtil::Clamp(m_last_latency, min_latency_ms, milliseconds(MAX_LATENCY_MS));
  if (latency_ms != m_last_latency)
  {
    NOTICE_LOG(AUDIO, "OpenAL: Latency setting out of range: %d -> %d",
               static_cast<int>(m_last_latency.count()), static_cast<int>(latency_ms.count()));
  }
  if (latency_ms == m_last_canon_latency)
    return;
  m_last_canon_latency = latency_ms;

  // iLatency is opaque so we arbitrarily break it in half and give half
  // to SoundTouch and the rest to OpenAL. This isn't ideal since increasing
  // SoundTouch's latency also increases the computational load.
  // Special Case: If half of the latency is less than the minimum OpenAL buffer
  //   size then we guarantee OpenAL gets what it needs and the remainder goes
  //   to SoundTouch instead.
  milliseconds half_latency_ms = latency_ms / 2;
  milliseconds sound_touch_ms = half_latency_ms;
  if (half_latency_ms < min_openal_buffer_ms)
    sound_touch_ms = latency_ms - min_openal_buffer_ms;

  ConfigureSoundTouch(sound_touch_ms);

  unsigned int latency_frames =
      static_cast<unsigned int>(latency_ms.count() * m_mixer->GetSampleRate() / std::milli::den);
  m_openal_buffer_size = (latency_frames - ComputeSoundTouchLatency()) / NUM_BUFFERS;
  m_processing_buffer.clear();  // So we don't memcpy
  m_processing_buffer.resize(m_openal_buffer_size * STEREO_CHANNELS);
}

void OpenALStream::WorkerThread::DrainMixer()
{
  // Drain CMixer into our holding FIFO until we run out of audio to pull,
  // or we hit the size limit.
  const unsigned int FIFO_LIMIT =
      static_cast<unsigned int>(ST_MAX_TEMPO) * NUM_BUFFERS * m_openal_buffer_size;
  static constexpr unsigned int NUM_PASS_FRAMES = 512;
  unsigned int num_frames = NUM_PASS_FRAMES;
  while (num_frames == NUM_PASS_FRAMES && m_holding_fifo.numSamples() < FIFO_LIMIT)
  {
    short samples[NUM_PASS_FRAMES * STEREO_CHANNELS];
    num_frames = m_mixer->MixAvailable(samples, NUM_PASS_FRAMES);

    // Convert short to float.
    // NOTE: This loop is auto-vectorized by MSVC.
    float* out_buffer = m_holding_fifo.ptrEnd(num_frames);
    for (unsigned int i = 0; i < num_frames * STEREO_CHANNELS; ++i)
    {
      out_buffer[i] = samples[i] / static_cast<float>(1 << 15);
    }
    m_holding_fifo.putSamples(num_frames);
  }

  // Resynchronize the Mixer if it becomes desynced (due to sample rate changes).
  // We use 4ms of lag as a fudge since different CPU side systems provide samples
  // in different sized chunks.
  if (m_mixer->GetFIFOLag() > m_mixer->GetSampleRate() * 4 / 1000)
    m_mixer->Resynchronize();
}

ALsizei OpenALStream::WorkerThread::ConvertToSurround51(float* output, float* stereo_source,
                                                        ALsizei size)
{
  DPL2Decode(stereo_source, size / FRAME_STEREO_F32, output);
  size *= 3;  // FRAME_SURROUND51_F32 / FRAME_STEREO_F32

  // zero-out the subwoofer channel - DPL2Decode generates a pretty
  // good 5.0 but not a good 5.1 output.
  // DPL2Decode output: LEFTFRONT, RIGHTFRONT, CENTREFRONT, (sub), LEFTREAR, RIGHTREAR
  static constexpr int LFE_CHANNEL = 3;
  for (ALsizei i = 0; i < size / FRAME_SURROUND51_F32; ++i)
  {
    output[i * SURROUND51_CHANNELS + LFE_CHANNEL] = 0.f;
  }

  return size;
}

ALsizei OpenALStream::WorkerThread::ConvertToOutputFormat(float* source, ALsizei size)
{
  switch (m_openal_format)
  {
  case AL_FORMAT_51CHN32:  // Decode with Dolby DPL2, 2CH_F32 -> 51CH_F32
    EnsureCapacityBytes(&m_format_conversion_buffer, size * 3);
    return ConvertToSurround51(reinterpret_cast<float*>(m_format_conversion_buffer.data()), source,
                               size);

  case AL_FORMAT_STEREO_FLOAT32:  // memcpy
    EnsureCapacityBytes(&m_format_conversion_buffer, size);
    std::copy_n(source, size / sizeof(float),
                reinterpret_cast<float*>(m_format_conversion_buffer.data()));
    return size;

  case AL_FORMAT_51CHN16:  // Convert 2CH_F32 -> 51CH_F32, then convert F32 -> S16
    EnsureCapacityBytes(&m_dpl2_buffer, size * 3);
    size = ConvertToSurround51(m_dpl2_buffer.data(), source, size);
    source = m_dpl2_buffer.data();
  // fallthrough

  case AL_FORMAT_STEREO16:  // Convert F32 -> S16
  {
    ALsizei num_samples = size / sizeof(float);
    size = num_samples * sizeof(s16);
    EnsureCapacityBytes(&m_format_conversion_buffer, size);
    s16* output = reinterpret_cast<s16*>(m_format_conversion_buffer.data());
    for (ALsizei i = 0; i < num_samples; ++i)
      output[i] = static_cast<s16>(MathUtil::Clamp(source[i] * (1 << 15), -32768.f, 32767.f));
    return size;
  }

  default:
    _assert_msg_(AUDIO, false, "The OpenAL format is not implemented. Fix ConvertToOutputFormat.");
    return 0;
  }
}

void OpenALStream::WorkerThread::ConfigureSoundTouch(std::chrono::milliseconds latency)
{
  // This function essentially works backwards to configure SoundTouch from the
  // desired processing latency which makes things somewhat complicated. The
  // basic idea is that Overlap has the biggest quality effect, then Seeking,
  // then Sequence is just gravy, so we tune parameters in that order according
  // to the latency we have available to work with.
  //
  // Internal Latency calculations extracted from inside SoundTouch:
  // SeekWindow = max(SEQUENCE_MS, OVERLAP_MS * 2)
  // ChunkSize = Tempo * (SeekWindow - OVERLAP_MS)
  // Latency = max(ChunkSize + OVERLAP_MS, SeekWindow) + SEEKWINDOW_MS
  //
  // The calculations are shown in MS for convenience, actual latency values
  // are computed in samples. A notable property is that tempo <= 1.0 has a
  // constant latency, but tempo > 1.0 has increasing latency in proportion
  // to the tempo value. ChunkSize is the amount of data pulled from the
  // latency input buffer per pass, the ChunkSize is always determined by
  // the Tempo so the size of SoundTouch's output can sawtooth at high
  // tempo values (produce too much, then nothing due to being underfilled).
  using std::chrono::milliseconds;

  // Minimum: max(1 * (max(1, 2 * 2) - 2) + 2, max(1, 2 * 2)) + 15 = 19ms
  if (latency < milliseconds(42))
  {
    _assert_(latency > milliseconds(18));
    m_sound_touch.setSetting(SETTING_SEQUENCE_MS, ST_MIN_SEQUENCE);
    if (latency < milliseconds(32))  // Minimum 19
    {
      // max(1 * (max(1, 2 * 2) - 2) + 2, max(1, 2 * 2)) + 15 = 19ms
      // max(1 * (max(1, 4 * 2) - 4) + 4, max(1, 4 * 2)) + 15 = 23ms
      // max(1 * (max(1, 6 * 2) - 6) + 6, max(1, 6 * 2)) + 15 = 27ms
      // max(1 * (max(1, 8 * 2) - 8) + 8, max(1, 8 * 2)) + 15 = 31ms
      // NOTE: This only accepts odd latency values. Even ones alias down.
      //   20 -> 19, 22 -> 21, 24 -> 23, etc.
      milliseconds overlap = (latency - milliseconds(15)) / 2;
      m_sound_touch.setSetting(SETTING_SEEKWINDOW_MS, ST_MIN_SEEKWINDOW);
      m_sound_touch.setSetting(SETTING_OVERLAP_MS, static_cast<int>(overlap.count()));
    }
    else
    {
      // max(1 * (max(1, 8 * 2) - 8) + 8, max(1, 8 * 2)) + 16 = 32ms
      // max(1 * (max(1, 8 * 2) - 8) + 8, max(1, 8 * 2)) + 25 = 41ms
      milliseconds seek_window = latency - milliseconds(16);
      m_sound_touch.setSetting(SETTING_OVERLAP_MS, ST_MAX_OVERLAP);
      m_sound_touch.setSetting(SETTING_SEEKWINDOW_MS, static_cast<int>(seek_window.count()));
    }
  }
  else
  {
    // Because the ChunkSize causes sawtoothing of the output size (i.e.
    // SoundTouch will overprocess the input buffer producing too much
    // output on one pass then none on the next because of insufficient
    // input latency) we need to control the sequence size so that it
    // does not explode out of control in proportion to the OpenAL buffer
    // size.
    // max(1 * (17 - 8) + 8, 17) + 25 = 42ms (latency = 365ms) [ChunkSize10 = 90ms; OpenAL = 182ms]
    // max(1 * (23 - 8) + 8, 23) + 25 = 48ms (latency = 500ms) [ChunkSize10 = 150ms; OpenAL = 250ms]
    milliseconds sequence =
        (latency - milliseconds(42 - 17)) / static_cast<u32>(ST_MAX_TEMPO * NUM_BUFFERS);
    m_sound_touch.setSetting(SETTING_SEEKWINDOW_MS, ST_MAX_SEEKWINDOW);
    m_sound_touch.setSetting(SETTING_OVERLAP_MS, ST_MAX_OVERLAP);
    m_sound_touch.setSetting(SETTING_SEQUENCE_MS, std::max(static_cast<int>(sequence.count()), 1));
  }
}

unsigned int OpenALStream::WorkerThread::ComputeSoundTouchLatency(double tempo) const
{
  _assert_(tempo > 0.0);

  // u64 to implicitly perform all calculations using wider arithmetic.
  u64 sample_rate = m_mixer->GetSampleRate();
  u32 overlap = static_cast<u32>(m_sound_touch.getSetting(SETTING_OVERLAP_MS) * sample_rate / 1000);
  u32 sequence =
      static_cast<u32>(m_sound_touch.getSetting(SETTING_SEQUENCE_MS) * sample_rate / 1000);
  u32 seek_window =
      static_cast<u32>(m_sound_touch.getSetting(SETTING_SEEKWINDOW_MS) * sample_rate / 1000);

  // See ConfigureSoundTouch for the explanation of this formula.
  u32 internal_seek_window = std::max(sequence, overlap * 2);
  u32 latency = static_cast<u32>(tempo * (internal_seek_window - overlap) + overlap);
  return std::max(latency, internal_seek_window) + seek_window;
}

auto OpenALStream::WorkerThread::SetSoundTouchTempo(u32 input_frames, u32 output_frames)
    -> SoundTouchProcess
{
  // Include the current latency samples that are queued in SoundTouch's input.
  u32 total_input = m_sound_touch.numUnprocessedSamples() + input_frames;
  // We also include any left-over output from the previous buffer if it
  // generated slightly too much.
  u32 adjusted_output = m_sound_touch.numSamples();
  if (adjusted_output >= output_frames)
  {
    // Caused by ChunkSize being too big and producing too many samples
    // per execution. This should never happen, we took pains to prevent it
    // in ConfigureSoundTouch().
    ERROR_LOG(AUDIO, "OpenAL: Latency spike. Too many excess samples: %u (%u)", adjusted_output,
              output_frames);
    return SoundTouchProcess::None;
  }
  adjusted_output = output_frames - adjusted_output;

  // Formulas:
  // SeekWindow = max(SEQUENCE, OVERLAP * 2)
  // ChunkSize = Tempo * (SeekWindow - OVERLAP)
  // Latency = max(ChunkSize + OVERLAP, SeekWindow) + SEEKWINDOW
  // [a] Latency = ChunkSize + OVERLAP + SEEKWINDOW
  // [b] Latency = SeekWindow + SEEKWINDOW
  // Tempo = (in - Latency) / out
  //
  // We'll expand the max() function and solve separately.
  // [a] Tempo = (in - (Tempo * (SeekWindow - OVERLAP) + OVERLAP + SEEKWINDOW)) / out
  // [a] Tempo = (in - Tempo * SeekWindow + Tempo * OVERLAP - OVERLAP - SEEKWINDOW) / out
  // [a] Tempo * out = in - Tempo * SeekWindow + Tempo * OVERLAP - OVERLAP - SEEKWINDOW
  // [a] Tempo * (out + SeekWindow - OVERLAP) = in - OVERLAP - SEEKWINDOW
  // [a] Tempo = (in - OVERLAP - SEEKWINDOW) / (out + SeekWindow - OVERLAP)
  // [b] Tempo = (in - SeekWindow - SEEKWINDOW) / out
  u64 sample_rate = m_mixer->GetSampleRate();
  s64 overlap = m_sound_touch.getSetting(SETTING_OVERLAP_MS) * sample_rate / 1000;
  s64 sequence = m_sound_touch.getSetting(SETTING_SEQUENCE_MS) * sample_rate / 1000;
  s64 seek_window = m_sound_touch.getSetting(SETTING_SEEKWINDOW_MS) * sample_rate / 1000;
  s64 internal_seek_window = std::max(sequence, overlap * 2);

  double tempo_a = static_cast<double>(total_input - overlap - seek_window + 1) /
                   static_cast<double>(adjusted_output + internal_seek_window - overlap);
  double tempo_b =
      static_cast<double>(total_input - internal_seek_window - seek_window + 1) / adjusted_output;

  // We now need to select which of the two tempo functions to use.
  // Below 1.0, tempo_b is the correct one. Above 1.0, tempo_a is used.
  // The functions intersect at 1.0 where tempo_b continues upward at a
  // sharper slope than tempo_a.
  double tempo = std::min(tempo_a, tempo_b);
  NOTICE_LOG(AUDIO, "OpenAL: Tempo is %.2f (a = %.2f, b = %.2f)", tempo, tempo_a, tempo_b);

  // We can get negative tempos if the input is abysmally tiny, or zero.
  // i.e. the emulator has stalled for too long and we've run out of audio.
  if (tempo <= 0.0)
    return SoundTouchProcess::None;

  // If the required tempo is outside of SoundTouch's range then we can't do
  // anything with it.
  if (tempo < ST_MIN_TEMPO || tempo > ST_MAX_TEMPO)
  {
    m_sound_touch.setTempo(MathUtil::Clamp(tempo, ST_MIN_TEMPO, ST_MAX_TEMPO));
    return SoundTouchProcess::Failed;
  }
  m_sound_touch.setTempo(tempo);
  return SoundTouchProcess::Normal;
}

bool OpenALStream::WorkerThread::GenerateOutputBuffer()
{
  // We need to slice our holding FIFO to partition the audio evenly across
  // all the buffers. We also need to consider the currently playing buffer
  // since the holding FIFO will contain audio roughly equal to the amount of
  // depletion from the playing buffer (i.e. current buffer is 50% played,
  // the holding FIFO will have 1.5 audio buffers in it).
  u32 num_active_buffers = static_cast<u32>(m_buffers_active.count());
  u32 reserve = 0;
  if (num_active_buffers)
  {
    // Get the play cursor for the playing buffer.
    // This works by offset from head of alSourceQueueBuffers. Since
    // we are constantly pulling buffers out and cycling them, the
    // play cursor will always be somewhere in or near the first
    // buffer.
    ALint offset = 0;
    alGetSourcei(m_source, AL_SAMPLE_OFFSET, &offset);
    reserve = std::min<u32>(offset, m_openal_buffer_size);
  }
  u32 num_inactive_buffers = NUM_BUFFERS - num_active_buffers;
  _assert_(num_inactive_buffers);
  // NOTE: This is fixed point using base m_openal_buffer_size as the fixed base.
  u32 holding_fifo_slice =
      static_cast<u32>(static_cast<u64>(m_holding_fifo.numSamples()) * m_openal_buffer_size /
                       (static_cast<u64>(num_inactive_buffers) * m_openal_buffer_size + reserve));

  // Feed the holding FIFO through SoundTouch now.
  // We always do this unless explicitly instructed not to in order to keep
  // SoundTouch's internal state up to date so we can resume playback after
  // a failure at any time without crackle.
  // NOTE: SoundTouch is complex and potentially slow (in the millisecond range).
  auto success = SetSoundTouchTempo(holding_fifo_slice, m_openal_buffer_size);
  if (success != SoundTouchProcess::None)
  {
    m_sound_touch.putSamples(m_holding_fifo.ptrBegin(), holding_fifo_slice);
    m_holding_fifo.receiveSamples(holding_fifo_slice);
  }
  if (success == SoundTouchProcess::Failed)
  {
    // If SoundTouch cannot process the current audio then what we got after
    // processing is junk so throw it away.
    m_sound_touch.adjustAmountOfSamples(0);
    return false;
  }
  NOTICE_LOG(AUDIO, "Samples In: %u, Out: %u", holding_fifo_slice, m_sound_touch.numSamples());
  // If SoundTouch is not primed and we don't have enough audio to prime it
  // then we can "succeed" but produce no output. This usually happens when
  // recovering from a frame stall (i.e. shader stutter that took longer than
  // our buffer latency to clear so we ran out of audio to use)
  return m_sound_touch.numSamples() >= MIN_OPENAL_BUFFER_FRAMES;
}

void OpenALStream::WorkerThread::WriteOpenALBuffer(ALuint buffer, void* source, ALsizei bytes)
{
  // NOTE: m_openal_lock is held
  _assert_(buffer != 0);

  // Submit data to OpenAL (it makes an internal copy)
  alBufferData(buffer, m_openal_format, source, bytes, m_mixer->GetSampleRate());
  ALenum err = alGetError();
  if (err != AL_NO_ERROR)
  {
    ERROR_LOG(AUDIO, "OpenAL: Failed to submit buffer data: %08X", err);
    return;
  }

  // Queue buffer for playback
  alSourceQueueBuffers(m_source, 1, &buffer);
  err = alGetError();
  if (err != AL_NO_ERROR)
  {
    ERROR_LOG(AUDIO, "OpenAL: Failed to attach buffer to source: %08X", err);
    return;
  }

  // Mark buffer active
  for (unsigned int i = 0; i < NUM_BUFFERS; ++i)
  {
    if (m_buffers[i] == buffer)
    {
      m_buffers_active[i] = true;
      break;
    }
  }
}

bool OpenALStream::WorkerThread::PollOpenALBufferStates()
{
  // NOTE: m_openal_lock is held

  ALint num_buffers_done = 0;
  alGetSourcei(m_source, AL_BUFFERS_PROCESSED, &num_buffers_done);
  if (!num_buffers_done)
    return !m_buffers_active.all();

  ALuint unqueued_buffers[NUM_BUFFERS] = {0};
  alSourceUnqueueBuffers(m_source, num_buffers_done, unqueued_buffers);
  if (!unqueued_buffers[0])
    return !m_buffers_active.all();

  // Clear active flags for all buffers we got back
  for (ALint i = 0; i < num_buffers_done; ++i)
  {
    for (unsigned int j = 0; j < NUM_BUFFERS; ++j)
    {
      if (m_buffers[j] == unqueued_buffers[i])
      {
        m_buffers_active[j] = false;
        break;
      }
    }
  }
  return true;
}

ALuint OpenALStream::WorkerThread::GetNextEmptyOpenALBuffer()
{
  static_assert(NUM_BUFFERS <= sizeof(u32) * 8, "Too many bits to use LeastSignificantSetBit");
  int idx = LeastSignificantSetBit(static_cast<u32>(~m_buffers_active.to_ulong()));
  if (idx < 0 || static_cast<unsigned int>(idx) >= NUM_BUFFERS)
    return 0;
  return m_buffers[idx];
}

void OpenALStream::WorkerThread::CheckForUnderrun()
{
  // NOTE: m_openal_lock is held
  if (m_is_paused)
    return;

  ALint state = AL_PLAYING;
  alGetSourcei(m_source, AL_SOURCE_STATE, &state);
  // Wait until ALL buffers are full (i.e. maximum latency attained)
  // before beginning to play otherwise we'll probably just underrun
  // again immediately because each individual buffer is small.
  if (state != AL_PLAYING && m_buffers_active.all())
  {
    // Buffer underrun occurred, resume playback
    alSourcePlay(m_source);
    ALenum err = alGetError();
    if (err != AL_NO_ERROR)
      ERROR_LOG(AUDIO, "OpenAL: Error occurred resuming playback: %08x", err);
    WARN_LOG(
        AUDIO,
        "OpenAL: Buffer has underrun. Latency may be too low for current hardware or system load.");
  }
}

#endif  // HAVE_OPENAL
