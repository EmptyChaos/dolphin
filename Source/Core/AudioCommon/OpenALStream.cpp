// Copyright 2008 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include <algorithm>
#include <array>
#include <chrono>
#include <thread>
#include <utility>
#include <vector>

#include "AudioCommon/DPL2Decoder.h"
#include "AudioCommon/OpenALStream.h"
#include "AudioCommon/aldlist.h"
#include "Common/Assert.h"
#include "Common/Event.h"
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
#include <soundtouch/TDStretch.h>

#ifdef __APPLE__
#undef BOOL
#endif

#ifdef _WIN32
#pragma comment(lib, "openal32.lib")
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
    m_td_stretch->setChannels(STEREO_CHANNELS);
    m_td_stretch->enableQuickSeek(false);  // This is broken. It segfaults.
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
  // May change m_openal_buffer_size
  void PollForConfigurationChanges(bool force = false);

  // [THREAD] Checks if we've been sent a control message and updates the state.
  void PollForControlMessages();

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
  // Returns false if it fails to produce an entire buffer of output.
  // If it returns true then m_td_stretch->numSamples() will contain approximately
  // m_openal_buffer_size samples of output.
  // NOTE: m_holding_fifo may not be empty
  bool GenerateOutputBuffer();

  // [THREAD] Writes the given buffer into the given OpenAL buffer and queues it to
  // the source.
  void WriteOpenALBuffer(ALuint buffer, void* source, ALsizei bytes);

  // [THREAD] Checks OpenAL for completed buffers and updates the tracking variables.
  // Returns true if there are empty buffers that need refilling.
  // Returns false if all buffers are still active and being played.
  bool PollOpenALBufferStates();

  // [THREAD] Returns the next empty OpenAL buffer that needs to be filled.
  // Returns 0 if all buffers are still queued.
  ALuint GetNextEmptyOpenALBuffer();

  // [THREAD] Checks if OpenAL playback underrun and needs to be resumed.
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

  // OpenAL is "threadsafe" but not actually designed properly for use on
  // multiple threads because alGetError() is global state associated with
  // the current context. If we call OpenAL functions on multiple threads
  // then the resulting error codes are useless junk (Windows uses TLS for
  // GetLastError(), and most C libraries do the same for errno). We must
  // therefore only use OpenAL on one thread so the state remains sane and
  // use a message passing system to ask the thread to do things.
  std::atomic<float> m_msg_volume{1.f};
  std::atomic<bool> m_msg_pause{false};
  std::atomic<unsigned int> m_msg_version{0};  // Lock-free message passing
  unsigned int m_msg_version_last_check = 0;   // [THREAD] Used to detect new messages
  Common::Event m_unpaused_event;

  std::thread m_thread;
  std::atomic<bool> m_thread_run{false};
  bool m_is_paused = false;  // [THREAD]

  ALCdevicePtr m_device{nullptr, alcCloseDevice};
  ALCcontextPtr m_context{nullptr, alcDestroyContext};
  CMixer* m_mixer = nullptr;

  ALuint m_source = 0;
  std::array<ALuint, NUM_BUFFERS> m_idle_buffers{};
  unsigned int m_idle_buffers_end = 0;
  ALenum m_openal_format = 0;

  std::chrono::milliseconds m_last_latency{0};
  std::chrono::milliseconds m_last_canon_latency{0};
  std::chrono::milliseconds m_min_openal_buffer_ms{0};  // Total across NUM_BUFFERS
  unsigned int m_openal_buffer_size = 0;                // Audio Frames

  std::unique_ptr<soundtouch::TDStretch> m_td_stretch{soundtouch::TDStretch::newInstance()};

  soundtouch::FIFOSampleBuffer m_holding_fifo{STEREO_CHANNELS};
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
  NOTICE_LOG(AUDIO, "Found OpenAL device \"%s\"", default_device_name);

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
  auto context_reset = Common::MakeScopeGuard([] { alcMakeContextCurrent(nullptr); });

  alGenBuffers(NUM_BUFFERS, m_idle_buffers.data());
  ALenum err = alGetError();
  if (err != AL_NO_ERROR)
  {
    PanicAlertT("OpenAL: Could not create audio buffers: %08X", err);
    return false;
  }
  auto buffer_release = Common::MakeScopeGuard([&] {
    alDeleteBuffers(NUM_BUFFERS, m_idle_buffers.data());
    std::fill_n(m_idle_buffers.data(), NUM_BUFFERS, 0);
  });

  alGenSources(1, &m_source);
  err = alGetError();
  if (err != AL_NO_ERROR)
  {
    PanicAlertT("OpenAL: Could not create audio source: %08X", err);
    return false;
  }
  auto source_release = Common::MakeScopeGuard([&] {
    alDeleteSources(1, &m_source);
    m_source = 0;
  });

  // Feature Tests.
  // NOTE: Keep these sorted from most preferred to least preferred.
  bool supports_float32 = false;
  m_openal_format = 0;
#ifdef AL_EXT_float32
  {
    std::string renderer{alGetString(AL_RENDERER)};

    if (alIsExtensionPresent("AL_EXT_float32"))
    {
      // Checks if an X-Fi is being used. If it is, disable FLOAT32 support as this
      // sound card has no support for it even though it reports it does.
      supports_float32 = renderer.find("X-Fi") == std::string::npos;
    }
    NOTICE_LOG(AUDIO, "OpenAL Renderer: %s (Float32: %s)", renderer.c_str(),
               supports_float32 ? "yes" : "no");
  }
#endif
#ifdef AL_EXT_MCFORMATS
  if (alIsExtensionPresent("AL_EXT_MCFORMATS"))
  {
    if (!m_openal_format && supports_float32 && num_channels >= SURROUND51_CHANNELS)
    {
      float silence[32 * SURROUND51_CHANNELS] = {0.f};
      alBufferData(m_idle_buffers[0], AL_FORMAT_51CHN32, silence, sizeof(silence), sample_rate);
      if (alGetError() != AL_INVALID_ENUM)
        m_openal_format = AL_FORMAT_51CHN32;
      else
        WARN_LOG(AUDIO, "OpenAL: Unable to render 5.1 Surround in Float32");
    }
    if (!m_openal_format && num_channels >= SURROUND51_CHANNELS)
    {
      short silence[32 * SURROUND51_CHANNELS] = {0};
      alBufferData(m_idle_buffers[0], AL_FORMAT_51CHN16, silence, sizeof(silence), sample_rate);
      if (alGetError() != AL_INVALID_ENUM)
        m_openal_format = AL_FORMAT_51CHN16;
      else
        WARN_LOG(AUDIO, "OpenAL: Unable to render 5.1 Surround");
    }
  }
#endif
#ifdef AL_EXT_float32
  if (!m_openal_format && supports_float32)
  {
    float silence[32 * STEREO_CHANNELS] = {0.f};
    alBufferData(m_idle_buffers[0], AL_FORMAT_STEREO_FLOAT32, silence, sizeof(silence),
                 sample_rate);
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
    NOTICE_LOG(AUDIO, "OpenAL: Refresh %dHz. Minimum OpenAL Latency: %.2f", refresh,
               1000.0 * NUM_BUFFERS / refresh);
  }

  // Initialize DPL2
  // TODO: DPL2 Encoder should be a class so we can just create an instance of it.
  DPL2Reset();

  // Actually start the thread.
  m_mixer = mixer;
  m_min_openal_buffer_ms = std::chrono::milliseconds(
      (MIN_OPENAL_BUFFER_FRAMES * NUM_BUFFERS * 1000 + sample_rate - 1) / sample_rate);
  m_idle_buffers_end = NUM_BUFFERS;
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
  m_unpaused_event.Set();
  m_thread.join();

  // Release OpenAL resources.
  alSourceStop(m_source);
  if (m_idle_buffers_end != NUM_BUFFERS)
    alSourceUnqueueBuffers(m_source, NUM_BUFFERS - m_idle_buffers_end,
                           &m_idle_buffers[m_idle_buffers_end]);
  alSourcei(m_source, AL_BUFFER, 0);
  alDeleteBuffers(NUM_BUFFERS, m_idle_buffers.data());
  alDeleteSources(1, &m_source);

  // Release OpenAL context.
  alcMakeContextCurrent(nullptr);
  // Destructors for unique_ptr destroy context and device.
}

void OpenALStream::WorkerThread::Pause(bool paused)
{
  m_msg_pause.store(paused, std::memory_order_relaxed);
  std::atomic_thread_fence(std::memory_order_release);
  m_msg_version.fetch_add(1, std::memory_order_relaxed);
  if (!paused)
    m_unpaused_event.Set();
}

void OpenALStream::WorkerThread::SetVolume(float volume)
{
  m_msg_volume.store(MathUtil::Clamp(volume, 0.f, 1.f), std::memory_order_relaxed);
  std::atomic_thread_fence(std::memory_order_release);
  m_msg_version.fetch_add(1, std::memory_order_relaxed);
}

void OpenALStream::WorkerThread::RunLoop()
{
  Common::SetCurrentThreadName("Audio: OpenALStream");
  std::vector<float> silence_buffer;
  PollForConfigurationChanges(true);

  while (m_thread_run.load(std::memory_order_relaxed))
  {
    // Transfer CMixer buffer into m_holding_fifo
    DrainMixer();

    // Check for latency configuration changes
    PollForConfigurationChanges();

    // Check for messages
    PollForControlMessages();
    if (m_is_paused)
    {
      m_unpaused_event.Wait();
      continue;
    }

    // Check if OpenAL has finished any buffers.
    if (!PollOpenALBufferStates())
    {
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
    // Run the pending audio through SoundTouch. This can take a while, if the latency is too low
    // for the Host CPU then we may underrun due to the computation latency.
    float* float_buffer = nullptr;
    ALsizei buffer_size = 0;
    unsigned int td_stretch_frames = 0;
    if (GenerateOutputBuffer())
    {
      // Point the buffer at SoundTouch's output
      td_stretch_frames = std::min(m_td_stretch->numSamples(), m_openal_buffer_size);
      float_buffer = m_td_stretch->getOutput()->ptrBegin();
      buffer_size = td_stretch_frames * FRAME_STEREO_F32;
    }
    else
    {
      // SoundTouch failed to produce usable output so we'll just play silence instead.
      WARN_LOG(AUDIO,
               "OpenAL: Input underrun (CPU stall or latency too low for current framelimit)");
      EnsureCapacity(&silence_buffer, m_openal_buffer_size * STEREO_CHANNELS);
      float_buffer = silence_buffer.data();
      buffer_size = m_openal_buffer_size * FRAME_STEREO_F32;
    }
    _assert_(buffer_size / FRAME_STEREO_F32 >= MIN_OPENAL_BUFFER_FRAMES / 2);

    // Do we need a format conversion?
    void* buffer = float_buffer;
#ifdef AL_EXT_float32
    if (m_openal_format != AL_FORMAT_STEREO_FLOAT32)
#endif
    {
      buffer_size = ConvertToOutputFormat(float_buffer, buffer_size);
      buffer = m_format_conversion_buffer.data();
    }

    // Transfer the buffer to OpenAL
    WriteOpenALBuffer(GetNextEmptyOpenALBuffer(), buffer, buffer_size);
    CheckForUnderrun();
    if (td_stretch_frames)
      m_td_stretch->receiveSamples(td_stretch_frames);
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
}

void OpenALStream::WorkerThread::PollForControlMessages()
{
  unsigned int version = m_msg_version.load(std::memory_order_relaxed);
  if (version == m_msg_version_last_check)
    return;
  m_msg_version_last_check = version;
  std::atomic_thread_fence(std::memory_order_acquire);

  // Apply volume
  alSourcef(m_source, AL_GAIN, m_msg_volume.load(std::memory_order_relaxed));

  // Apply Play/Pause
  bool make_paused = m_msg_pause.load(std::memory_order_relaxed);
  if (make_paused != m_is_paused)
  {
    m_is_paused = make_paused;
    if (make_paused)
    {
      alSourceStop(m_source);
    }
    else
    {
      alSourcePlay(m_source);
      ALenum err = alGetError();
      if (err != AL_NO_ERROR)
        ERROR_LOG(AUDIO, "OpenAL: Failed to resume playback of source: %08X", err);
    }
  }
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
#ifdef AL_EXT_MCFORMATS
  case AL_FORMAT_51CHN32:  // Decode with Dolby DPL2, 2CH_F32 -> 51CH_F32
    EnsureCapacityBytes(&m_format_conversion_buffer, size * 3);
    return ConvertToSurround51(reinterpret_cast<float*>(m_format_conversion_buffer.data()), source,
                               size);
#endif

#ifdef AL_EXT_float32
  case AL_FORMAT_STEREO_FLOAT32:  // memcpy
    EnsureCapacityBytes(&m_format_conversion_buffer, size);
    std::copy_n(source, size / sizeof(float),
                reinterpret_cast<float*>(m_format_conversion_buffer.data()));
    return size;
#endif

#ifdef AL_EXT_MCFORMATS
  case AL_FORMAT_51CHN16:  // Convert 2CH_F32 -> 51CH_F32, then convert F32 -> S16
    EnsureCapacityBytes(&m_dpl2_buffer, size * 3);
    size = ConvertToSurround51(m_dpl2_buffer.data(), source, size);
    source = m_dpl2_buffer.data();
// fallthrough
#endif

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
    if (latency < milliseconds(32))  // Minimum 19
    {
      // max(1 * (max(1, 2 * 2) - 2) + 2, max(1, 2 * 2)) + 15 = 19ms
      // max(1 * (max(1, 4 * 2) - 4) + 4, max(1, 4 * 2)) + 15 = 23ms
      // max(1 * (max(1, 6 * 2) - 6) + 6, max(1, 6 * 2)) + 15 = 27ms
      // max(1 * (max(1, 8 * 2) - 8) + 8, max(1, 8 * 2)) + 15 = 31ms
      // NOTE: This only accepts odd latency values. Even ones alias down.
      //   20 -> 19, 22 -> 21, 24 -> 23, etc.
      milliseconds overlap = (latency - milliseconds(15)) / 2;
      m_td_stretch->setParameters(m_mixer->GetSampleRate(), ST_MIN_SEQUENCE, ST_MIN_SEEKWINDOW,
                                  static_cast<int>(overlap.count()));
    }
    else
    {
      // max(1 * (max(1, 8 * 2) - 8) + 8, max(1, 8 * 2)) + 16 = 32ms
      // max(1 * (max(1, 8 * 2) - 8) + 8, max(1, 8 * 2)) + 25 = 41ms
      milliseconds seek_window = latency - milliseconds(16);
      m_td_stretch->setParameters(m_mixer->GetSampleRate(), ST_MIN_SEQUENCE,
                                  static_cast<int>(seek_window.count()), ST_MAX_OVERLAP);
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
    m_td_stretch->setParameters(m_mixer->GetSampleRate(),
                                std::max(static_cast<int>(sequence.count()), ST_MIN_SEQUENCE),
                                ST_MAX_SEEKWINDOW, ST_MAX_OVERLAP);
  }
}

unsigned int OpenALStream::WorkerThread::ComputeSoundTouchLatency(double tempo) const
{
  _assert_(tempo > 0.0);

  int sample_rate = 0;
  int overlap = 0;
  int sequence = 0;
  int seek_window = 0;
  m_td_stretch->getParameters(&sample_rate, &sequence, &seek_window, &overlap);
  overlap = static_cast<int>(static_cast<u64>(overlap) * sample_rate / 1000);
  sequence = static_cast<int>(static_cast<u64>(sequence) * sample_rate / 1000);
  seek_window = static_cast<int>(static_cast<u64>(seek_window) * sample_rate / 1000);

  // See ConfigureSoundTouch for the explanation of this formula.
  u32 internal_seek_window = std::max(sequence, overlap * 2);
  u32 latency = static_cast<u32>(tempo * (internal_seek_window - overlap) + overlap);
  return std::max(latency, internal_seek_window) + seek_window;
}

auto OpenALStream::WorkerThread::SetSoundTouchTempo(u32 input_frames, u32 output_frames)
    -> SoundTouchProcess
{
  // Include the current latency samples that are queued in SoundTouch's input.
  u32 total_input = m_td_stretch->getInput()->numSamples() + input_frames;
  // We also include any left-over output from the previous buffer if it
  // generated slightly too much.
  u32 adjusted_output = m_td_stretch->numSamples();
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
  int sample_rate = 0;
  int sequence = 0;
  int seek_window = 0;
  int overlap = 0;
  m_td_stretch->getParameters(&sample_rate, &sequence, &seek_window, &overlap);
  sequence = static_cast<int>(static_cast<u64>(sequence) * sample_rate / 1000);
  seek_window = static_cast<int>(static_cast<u64>(seek_window) * sample_rate / 1000);
  overlap = static_cast<int>(static_cast<u64>(overlap) * sample_rate / 1000);
  int internal_seek_window = std::max(sequence, overlap * 2);

  double tempo_a =
      static_cast<double>(static_cast<s64>(total_input) - overlap - seek_window + 1) /
      static_cast<double>(static_cast<s64>(adjusted_output) + internal_seek_window - overlap);
  double tempo_b =
      static_cast<double>(static_cast<s64>(total_input) - internal_seek_window - seek_window + 1) /
      adjusted_output;

  // We now need to select which of the two tempo functions to use.
  // Below 1.0, tempo_b is the correct one. Above 1.0, tempo_a is used.
  // The functions intersect at 1.0 where tempo_b continues upward at a
  // sharper slope than tempo_a.
  double tempo = std::min(tempo_a, tempo_b);
  DEBUG_LOG(AUDIO, "OpenAL: Tempo is %.2f (a = %.2f, b = %.2f)", tempo, tempo_a, tempo_b);

  // We can get negative tempos if the input is abysmally tiny, or zero.
  // i.e. the emulator has stalled for too long and we've run out of audio.
  if (tempo <= 0.0)
    return SoundTouchProcess::None;

  // If the required tempo is outside of SoundTouch's range then we can't do
  // anything with it.
  if (tempo < ST_MIN_TEMPO || tempo > ST_MAX_TEMPO)
  {
    m_td_stretch->setTempo(MathUtil::Clamp(tempo, ST_MIN_TEMPO, ST_MAX_TEMPO));
    return SoundTouchProcess::Failed;
  }
  m_td_stretch->setTempo(tempo);
  return SoundTouchProcess::Normal;
}

bool OpenALStream::WorkerThread::GenerateOutputBuffer()
{
  // We need to slice our holding FIFO to partition the audio evenly across
  // all the buffers. We also need to consider the currently playing buffer
  // since the holding FIFO will contain audio roughly equal to the amount of
  // depletion from the playing buffer (i.e. current buffer is 50% played,
  // the holding FIFO will have 1.5 audio buffers in it).
  u32 num_active_buffers = static_cast<u32>(NUM_BUFFERS - m_idle_buffers_end);
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
  // NOTE: This is fixed point using base m_openal_buffer_size as the fixed base.
  u32 holding_fifo_slice =
      static_cast<u32>(static_cast<u64>(m_holding_fifo.numSamples()) * m_openal_buffer_size /
                       (static_cast<u64>(m_idle_buffers_end) * m_openal_buffer_size + reserve));

  // Feed the holding FIFO through SoundTouch now.
  // We always do this unless explicitly instructed not to in order to keep
  // SoundTouch's internal state up to date so we can resume playback after
  // a failure at any time without crackle.
  // NOTE: SoundTouch is complex and potentially slow (in the millisecond range).
  auto success = SetSoundTouchTempo(holding_fifo_slice, m_openal_buffer_size);
  if (success != SoundTouchProcess::None)
  {
    m_td_stretch->putSamples(m_holding_fifo.ptrBegin(), holding_fifo_slice);
    m_holding_fifo.receiveSamples(holding_fifo_slice);
  }
  if (success == SoundTouchProcess::Failed)
  {
    // If SoundTouch cannot process the current audio then what we got after
    // processing is junk so throw it away.
    m_td_stretch->adjustAmountOfSamples(0);
    return false;
  }
  DEBUG_LOG(AUDIO, "Samples In: %u, Out: %u", holding_fifo_slice, m_td_stretch->numSamples());
  // If SoundTouch is not primed and we don't have enough audio to prime it
  // then we can "succeed" but produce no output. This usually happens when
  // recovering from a frame stall (i.e. shader stutter that took longer than
  // our buffer latency to clear so we ran out of audio to use)
  return m_td_stretch->numSamples() >= MIN_OPENAL_BUFFER_FRAMES;
}

void OpenALStream::WorkerThread::WriteOpenALBuffer(ALuint buffer, void* source, ALsizei bytes)
{
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
}

bool OpenALStream::WorkerThread::PollOpenALBufferStates()
{
  ALint num_buffers_done = 0;
  alGetSourcei(m_source, AL_BUFFERS_PROCESSED, &num_buffers_done);
  if (num_buffers_done)
  {
    alSourceUnqueueBuffers(m_source, num_buffers_done, &m_idle_buffers[m_idle_buffers_end]);
    if (m_idle_buffers[m_idle_buffers_end])
      m_idle_buffers_end += num_buffers_done;
  }
  return m_idle_buffers_end != 0;
}

ALuint OpenALStream::WorkerThread::GetNextEmptyOpenALBuffer()
{
  if (!m_idle_buffers_end)
    return 0;
  ALuint buffer = m_idle_buffers[--m_idle_buffers_end];
  m_idle_buffers[m_idle_buffers_end] = 0;
  return buffer;
}

void OpenALStream::WorkerThread::CheckForUnderrun()
{
  if (m_is_paused)
    return;

  ALint state = AL_PLAYING;
  alGetSourcei(m_source, AL_SOURCE_STATE, &state);
  // Wait until ALL buffers are full (i.e. maximum latency attained)
  // before beginning to play otherwise we'll probably just underrun
  // again immediately because each individual buffer is small.
  if (state != AL_PLAYING && !m_idle_buffers_end)
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
