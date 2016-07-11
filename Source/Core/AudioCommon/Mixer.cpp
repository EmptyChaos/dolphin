// Copyright 2008 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include <algorithm>

#include "AudioCommon/Mixer.h"
#include "Common/CommonFuncs.h"
#include "Common/CommonTypes.h"
#include "Common/Logging/Log.h"
#include "Common/MathUtil.h"
#include "Core/ConfigManager.h"

CMixer::CMixer(unsigned int BackendSampleRate) : m_sampleRate(BackendSampleRate)
{
  INFO_LOG(AUDIO_INTERFACE, "Mixer is initialized");
}

CMixer::~CMixer()
{
}

unsigned int CMixer::MixerFifo::AvailableSamples() const
{
  u32 read_idx = m_indexR.load(std::memory_order_relaxed);
  u32 write_idx = m_indexW.load(std::memory_order_relaxed);

  double ratio = static_cast<double>(m_mixer->GetSampleRate()) / m_input_sample_rate;

  // The Mix function has a latency of 1 sample so we need to subtract one from the
  // distance. m_frac is the fixed point offset between the 1 latency sample and the
  // next one, so we need to include the fractional offset in the calculation to
  // determine the amount left; we would rather undershoot than overshoot since
  // overshooting means we end up with padding garbage, undershooting just means
  // we have 1-2 samples of additional latency.
  unsigned int distance = std::max<u32>((write_idx - read_idx) / 2, 1) - 1;
  return distance ?
             static_cast<unsigned int>(ratio * (distance - m_frac / static_cast<double>(1 << 16))) :
             0;
}

// Executed from sound stream thread
unsigned int CMixer::MixerFifo::Mix(short* samples, unsigned int numSamples,
                                    bool consider_framelimit)
{
  unsigned int currentSample = 0;

  // Cache access in non-volatile variable
  // This is the only function changing the read value, so it's safe to
  // cache it locally although it's written here.
  // The writing pointer will be modified outside, but it will only increase,
  // so we will just ignore new written data while interpolating.
  // Without this cache, the compiler wouldn't be allowed to optimize the
  // interpolation loop.
  u32 indexR = m_indexR.load(std::memory_order_relaxed);
  u32 indexW = m_indexW.load(std::memory_order_acquire);  // acquire m_buffer

  // If the pipe is empty then skip everything. The only thing that will happen is
  // that we'll fill the buffer with padding samples which is usually bad.
  if (indexW - indexR <= 2)
    return 0;

  u32 ratio = static_cast<u32>(m_input_sample_rate * UINT64_C(65536) / m_mixer->GetSampleRate());
  if (consider_framelimit)
  {
    u32 low_waterwark = m_input_sample_rate * SConfig::GetInstance().iTimingVariance / 1000;
    low_waterwark = std::min(low_waterwark, MAX_SAMPLES / 2);

    float numLeft = static_cast<float>((indexW - indexR) / 2);
    m_numLeftI = (numLeft + m_numLeftI * (CONTROL_AVG - 1)) / CONTROL_AVG;
    float offset = (m_numLeftI - low_waterwark) * CONTROL_FACTOR;
    if (offset > MAX_FREQ_SHIFT)
      offset = MAX_FREQ_SHIFT;
    if (offset < -MAX_FREQ_SHIFT)
      offset = -MAX_FREQ_SHIFT;

    float emulationspeed = SConfig::GetInstance().m_EmulationSpeed;
    float aid_sample_rate = m_input_sample_rate + offset;
    if (emulationspeed > 0.0f)
    {
      aid_sample_rate = aid_sample_rate * emulationspeed;
    }

    ratio = static_cast<u32>(65536.f * aid_sample_rate / m_mixer->GetSampleRate());
  }

  s32 lvolume = m_LVolume.load(std::memory_order_relaxed);
  s32 rvolume = m_RVolume.load(std::memory_order_relaxed);

  // render numleft sample pairs to samples[]
  // advance indexR with sample position
  // remember fractional offset

  // TODO: consider a higher-quality resampling algorithm.
  for (; currentSample < numSamples * 2 && (indexW - indexR) > 2; currentSample += 2)
  {
    u32 indexR2 = indexR + 2;  // next sample

    s16 l1 = Common::swap16(m_buffer[indexR & INDEX_MASK]);   // current
    s16 l2 = Common::swap16(m_buffer[indexR2 & INDEX_MASK]);  // next
    int sampleL = ((l1 << 16) + (l2 - l1) * (u16)m_frac) >> 16;
    sampleL = (sampleL * lvolume) >> 8;
    sampleL += samples[currentSample + 1];
    samples[currentSample + 1] = MathUtil::Clamp(sampleL, -32767, 32767);

    s16 r1 = Common::swap16(m_buffer[(indexR + 1) & INDEX_MASK]);   // current
    s16 r2 = Common::swap16(m_buffer[(indexR2 + 1) & INDEX_MASK]);  // next
    int sampleR = ((r1 << 16) + (r2 - r1) * (u16)m_frac) >> 16;
    sampleR = (sampleR * rvolume) >> 8;
    sampleR += samples[currentSample];
    samples[currentSample] = MathUtil::Clamp(sampleR, -32767, 32767);

    m_frac += ratio;
    indexR += 2 * (u16)(m_frac >> 16);
    m_frac &= 0xffff;
  }

  unsigned int frames_written = currentSample / 2;

  // Padding
  short s[2];
  s[0] = Common::swap16(m_buffer[(indexR - 1) & INDEX_MASK]);
  s[1] = Common::swap16(m_buffer[(indexR - 2) & INDEX_MASK]);
  s[0] = (s[0] * rvolume) >> 8;
  s[1] = (s[1] * lvolume) >> 8;
  for (; currentSample < numSamples * 2; currentSample += 2)
  {
    int sampleR = MathUtil::Clamp(s[0] + samples[currentSample + 0], -32767, 32767);
    int sampleL = MathUtil::Clamp(s[1] + samples[currentSample + 1], -32767, 32767);

    samples[currentSample + 0] = sampleR;
    samples[currentSample + 1] = sampleL;
  }

  // Flush cached variable
  // Memory Order: We need release ordering here so that index updates strictly
  //   happen after we are finished copying from the FIFO rather than at any
  //   arbitrary point decided by CPU/Compiler reordering.
  m_indexR.store(indexR, std::memory_order_release);

  return frames_written;
}

void CMixer::MixerFifo::Purge(unsigned int amount)
{
  static constexpr unsigned int CHUNK_SIZE = 512;
  short buffer[CHUNK_SIZE];
  while (amount)
  {
    unsigned int cycle = std::min(amount, CHUNK_SIZE / 2);
    if (Mix(buffer, cycle, false) < cycle)
      break;
    amount -= cycle;
  }
}

void CMixer::MixerFifo::Purge()
{
  m_indexR.store(m_indexW.load(std::memory_order_relaxed), std::memory_order_relaxed);
  m_numLeftI = 0.f;
  m_frac = 0;
}

unsigned int CMixer::Mix(short* samples, unsigned int num_samples, bool consider_framelimit)
{
  if (!samples || !num_samples)
    return 0;

  std::fill_n(samples, num_samples * 2, 0);

  unsigned int dma_written = m_dma_mixer.Mix(samples, num_samples, consider_framelimit);
  unsigned int dtk_written = m_streaming_mixer.Mix(samples, num_samples, consider_framelimit);
  unsigned int wii_written = m_wiimote_speaker_mixer.Mix(samples, num_samples, consider_framelimit);
  return std::max({dma_written, dtk_written, wii_written});
}

unsigned int CMixer::MixAvailable(short* samples, unsigned int num_samples)
{
  if (!samples || !num_samples)
    return 0;

  std::fill_n(samples, num_samples * 2, 0);

  // Find out how many samples we have available and pull that many at most.
  // The primary condition we have to deal with is a race between DMA and DTK
  // where we try to mix after the DMA has been written but before the DTK
  // CoreTiming callback has added the DTK audio or vice versa. As such, we
  // must block forward progress if either the DMA or DTK is empty. [Those
  // FIFOs are always written by the CPU, it writes silence if it has nothing
  // else to put in it so we can safely rely on these 2 FIFOs to run in
  // lock-step reliably as long as we force synchronization here]
  // NOTE: A desync can happen if the input sample rate on the FIFOs are
  //   changed. This requires a Resynchronize() to fix.
  unsigned int dma_available = m_dma_mixer.AvailableSamples();
  unsigned int dtk_available = m_streaming_mixer.AvailableSamples();
  unsigned int fetch_size = std::min({num_samples, dma_available, dtk_available});
  if (!fetch_size)
    return 0;

  m_dma_mixer.Mix(samples, fetch_size, false);
  m_streaming_mixer.Mix(samples, fetch_size, false);
  m_wiimote_speaker_mixer.Mix(samples, fetch_size, false);
  return fetch_size;
}

void CMixer::Resynchronize()
{
  // Threshold is so we don't purge audio which doesn't need to resynchronized
  // because we're mid-CPU cycle and it hasn't updated one of the FIFOs yet.
  // NOTE: DTK is written in 3.5ms chunks. DMA is written in 8 sample chunks.
  //   This inter-system knowledge is unfortunate but would require embedding
  //   timing information in the audio stream to work without it.
  const unsigned int THRESHOLD = m_sampleRate * 4 / 1000;  // 4ms of samples
  unsigned int dma_available = m_dma_mixer.AvailableSamples();
  unsigned int dtk_available = m_streaming_mixer.AvailableSamples();

  if (dma_available > dtk_available + THRESHOLD)
  {
    m_dma_mixer.Purge(dma_available - dtk_available - THRESHOLD);
    m_wiimote_speaker_mixer.Purge();
  }
  else if (dma_available + THRESHOLD < dtk_available)
  {
    m_streaming_mixer.Purge(dtk_available - dma_available - THRESHOLD);
    m_wiimote_speaker_mixer.Purge();
  }
}

unsigned int CMixer::GetFIFOLag() const
{
  // FIFO Lag is usually caused by the CPU Thread changing sample rates to a lower one
  // i.e. 48000 -> 32000. This leaves us with 16000 or so samples which will be played
  // back at 2/3 speed acting as 500ms of lag.
  // SoundStreams that care about this lag can fix it by calling Resynchronize().
  unsigned int dma_available = m_dma_mixer.AvailableSamples();
  unsigned int dtk_available = m_streaming_mixer.AvailableSamples();
  return std::max(dma_available, dtk_available) - std::min(dma_available, dtk_available);
}

void CMixer::MixerFifo::PushSamples(const short* samples, unsigned int num_samples)
{
  if (!samples || !num_samples)
    return;

  // num_samples is frames (stereo pairs), convert to individual samples
  num_samples *= 2;

  // Cache access in non-volatile variable
  u32 indexW = m_indexW.load(std::memory_order_relaxed);
  u32 indexR = m_indexR.load(std::memory_order_acquire);  // acquire m_buffer

  // [---{xxxxxxx}-----]
  //     ^\  ^  /^\ ^ /
  //     |   |   |  remaining_samples
  //     |   |   indexW
  //     |   distance_samples
  //     indexR
  static constexpr unsigned int BUFFER_SIZE = std::tuple_size<decltype(m_buffer)>::value;
  unsigned int remaining_samples = BUFFER_SIZE - (indexW & INDEX_MASK);
  unsigned int distance_samples = indexW - indexR;

  // Check if we have enough free space
  if (distance_samples == BUFFER_SIZE)
    return;
  // We only write as many samples as will fit if there are too many,
  // the rest get dropped (This should never happen unless the SoundStream
  // broke and stopped removing data from the FIFO).
  num_samples = std::min(num_samples, BUFFER_SIZE - distance_samples);

  // Actual re-sampling work has been moved to sound thread
  // to alleviate the workload on main thread
  if (remaining_samples)
  {
    std::copy_n(samples, std::min(remaining_samples, num_samples),
                m_buffer.begin() + (indexW & INDEX_MASK));
  }
  if (remaining_samples < num_samples)
  {
    // overflow (i.e. wrap around and put the rest at the start)
    std::copy(samples + remaining_samples, samples + num_samples, m_buffer.begin());
  }

  m_indexW.store(indexW + num_samples, std::memory_order_release);  // release m_buffer
}

void CMixer::PushSamples(const short* samples, unsigned int num_samples)
{
  m_dma_mixer.PushSamples(samples, num_samples);
  int sample_rate = m_dma_mixer.GetInputSampleRate();
  if (m_log_dsp_audio)
    m_wave_writer_dsp.AddStereoSamplesBE(samples, num_samples, sample_rate);
}

void CMixer::PushStreamingSamples(const short* samples, unsigned int num_samples)
{
  m_streaming_mixer.PushSamples(samples, num_samples);
  int sample_rate = m_streaming_mixer.GetInputSampleRate();
  if (m_log_dtk_audio)
    m_wave_writer_dtk.AddStereoSamplesBE(samples, num_samples, sample_rate);
}

void CMixer::PushWiimoteSpeakerSamples(const short* samples, unsigned int num_samples,
                                       unsigned int sample_rate)
{
  short samples_stereo[MAX_SAMPLES * 2];

  if (num_samples < MAX_SAMPLES)
  {
    m_wiimote_speaker_mixer.SetInputSampleRate(sample_rate);

    for (unsigned int i = 0; i < num_samples; ++i)
    {
      samples_stereo[i * 2] = Common::swap16(samples[i]);
      samples_stereo[i * 2 + 1] = Common::swap16(samples[i]);
    }

    m_wiimote_speaker_mixer.PushSamples(samples_stereo, num_samples);
  }
}

void CMixer::SetDMAInputSampleRate(unsigned int rate)
{
  m_dma_mixer.SetInputSampleRate(rate);
}

void CMixer::SetStreamInputSampleRate(unsigned int rate)
{
  m_streaming_mixer.SetInputSampleRate(rate);
}

void CMixer::SetStreamingVolume(unsigned int lvolume, unsigned int rvolume)
{
  m_streaming_mixer.SetVolume(lvolume, rvolume);
}

void CMixer::SetWiimoteSpeakerVolume(unsigned int lvolume, unsigned int rvolume)
{
  m_wiimote_speaker_mixer.SetVolume(lvolume, rvolume);
}

void CMixer::StartLogDTKAudio(const std::string& filename)
{
  if (!m_log_dtk_audio)
  {
    m_log_dtk_audio = true;
    m_wave_writer_dtk.Start(filename, m_streaming_mixer.GetInputSampleRate());
    m_wave_writer_dtk.SetSkipSilence(false);
    NOTICE_LOG(AUDIO, "Starting DTK Audio logging");
  }
  else
  {
    WARN_LOG(AUDIO, "DTK Audio logging has already been started");
  }
}

void CMixer::StopLogDTKAudio()
{
  if (m_log_dtk_audio)
  {
    m_log_dtk_audio = false;
    m_wave_writer_dtk.Stop();
    NOTICE_LOG(AUDIO, "Stopping DTK Audio logging");
  }
  else
  {
    WARN_LOG(AUDIO, "DTK Audio logging has already been stopped");
  }
}

void CMixer::StartLogDSPAudio(const std::string& filename)
{
  if (!m_log_dsp_audio)
  {
    m_log_dsp_audio = true;
    m_wave_writer_dsp.Start(filename, m_dma_mixer.GetInputSampleRate());
    m_wave_writer_dsp.SetSkipSilence(false);
    NOTICE_LOG(AUDIO, "Starting DSP Audio logging");
  }
  else
  {
    WARN_LOG(AUDIO, "DSP Audio logging has already been started");
  }
}

void CMixer::StopLogDSPAudio()
{
  if (m_log_dsp_audio)
  {
    m_log_dsp_audio = false;
    m_wave_writer_dsp.Stop();
    NOTICE_LOG(AUDIO, "Stopping DSP Audio logging");
  }
  else
  {
    WARN_LOG(AUDIO, "DSP Audio logging has already been stopped");
  }
}

void CMixer::MixerFifo::SetInputSampleRate(unsigned int rate)
{
  // Any existing samples are now invalidated because the sample rate is wrong.
  // Ideally we would actually synchronize the sound stream by inserting the
  // sample rate change into the FIFO as a message so that existing audio can
  // continue to be processed correctly until it catches up but that requires a
  // rewrite.
  m_input_sample_rate = rate;
}

unsigned int CMixer::MixerFifo::GetInputSampleRate() const
{
  return m_input_sample_rate;
}

void CMixer::MixerFifo::SetVolume(unsigned int lvolume, unsigned int rvolume)
{
  // Memory Order: We don't want to use a lock so there's no sane order here.
  //   Even if we use acquire/release/seq_cst, that only guarantees that the
  //   SoundStream would see "as new or newer", never an exact pair.
  // Ultimately it doesn't matter because nothing about this is synchronized;
  // we have no idea what SoundStream is doing, it may be 40ms behind the CPU
  // so we're altering the volume of an arbitrary track that may be different
  // than intended.
  m_LVolume.store(lvolume + (lvolume >> 7), std::memory_order_relaxed);
  m_RVolume.store(rvolume + (rvolume >> 7), std::memory_order_relaxed);
}
