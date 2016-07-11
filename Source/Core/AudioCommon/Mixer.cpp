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
  u32 write_idx = m_indexW.load(std::memory_order_acquire);  // Acquire m_control_messages

  double ratio = static_cast<double>(m_mixer->GetSampleRate()) / m_ss_sample_rate;

  u32 stop_point =
      m_control_messages.Empty() ? write_idx : m_control_messages.Front().attached_index;

  // The Mix function has a latency of 1 sample so we need to subtract one from the
  // distance. m_frac is the fixed point offset between the 1 latency sample and the
  // next one, so we need to include the fractional offset in the calculation to
  // determine the amount left; we would rather undershoot than overshoot since
  // overshooting means we end up with padding garbage, undershooting just means
  // we have 1-2 samples of additional latency.
  unsigned int distance = std::max<u32>((stop_point - read_idx) / 2, 1) - 1;
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
  u32 raw_indexW = m_indexW.load(std::memory_order_acquire);  // acquire m_buffer

  // If the pipe is empty then skip everything. The only thing that will happen is
  // that we'll fill the buffer with padding samples which is usually bad.
  if (raw_indexW - indexR <= 2)
    return 0;

  // If there's a control message pending then we can only process up to that position
  // before we have to stop and apply the changes.
  u32 indexW = m_control_messages.Empty() ? raw_indexW : m_control_messages.Front().attached_index;

  u32 ratio = static_cast<u32>(m_ss_sample_rate * UINT64_C(65536) / m_mixer->GetSampleRate());
  if (consider_framelimit)
  {
    u32 low_waterwark = m_ss_sample_rate * SConfig::GetInstance().iTimingVariance / 1000;
    low_waterwark = std::min(low_waterwark, MAX_SAMPLES / 2);

    float numLeft = static_cast<float>((indexW - indexR) / 2);
    m_numLeftI = (numLeft + m_numLeftI * (CONTROL_AVG - 1)) / CONTROL_AVG;
    float offset = (m_numLeftI - low_waterwark) * CONTROL_FACTOR;
    if (offset > MAX_FREQ_SHIFT)
      offset = MAX_FREQ_SHIFT;
    if (offset < -MAX_FREQ_SHIFT)
      offset = -MAX_FREQ_SHIFT;

    float emulationspeed = SConfig::GetInstance().m_EmulationSpeed;
    float aid_sample_rate = m_ss_sample_rate + offset;
    if (emulationspeed > 0.0f)
    {
      aid_sample_rate = aid_sample_rate * emulationspeed;
    }

    ratio = static_cast<u32>(65536.f * aid_sample_rate / m_mixer->GetSampleRate());
  }

  // render numleft sample pairs to samples[]
  // advance indexR with sample position
  // remember fractional offset

  // TODO: consider a higher-quality resampling algorithm.
  for (; currentSample < numSamples * 2 && (indexW - indexR) > 2; currentSample += 2)
  {
    u32 indexR2 = indexR + 2;  // next sample

    // NOTE: Input is in Right-Left order. Output must be Left-Right.
    //   This makes things somewhat confusing as Channel0 is Right, not Left.
    s16 l1 = Common::swap16(m_buffer[indexR & INDEX_MASK]);   // current
    s16 l2 = Common::swap16(m_buffer[indexR2 & INDEX_MASK]);  // next
    int sampleR = ((l1 << 16) + (l2 - l1) * (u16)m_frac) >> 16;
    sampleR = (sampleR * m_ss_channel0_volume) >> 8;
    sampleR += samples[currentSample + 1];
    samples[currentSample + 1] = MathUtil::Clamp(sampleR, -32767, 32767);

    s16 r1 = Common::swap16(m_buffer[(indexR + 1) & INDEX_MASK]);   // current
    s16 r2 = Common::swap16(m_buffer[(indexR2 + 1) & INDEX_MASK]);  // next
    int sampleL = ((r1 << 16) + (r2 - r1) * (u16)m_frac) >> 16;
    sampleL = (sampleL * m_ss_channel1_volume) >> 8;
    sampleL += samples[currentSample];
    samples[currentSample] = MathUtil::Clamp(sampleL, -32767, 32767);

    m_frac += ratio;
    indexR += 2 * (u16)(m_frac >> 16);
    m_frac &= 0xffff;
  }

  // Prepare the padding samples before we release control of the ring buffer.
  // We are pre-swapping the order (Channel 1->0, 0->1)
  int pad_samples[2];
  pad_samples[0] = static_cast<s16>(Common::swap16(m_buffer[(indexR - 1) & INDEX_MASK]));
  pad_samples[1] = static_cast<s16>(Common::swap16(m_buffer[(indexR - 2) & INDEX_MASK]));
  pad_samples[0] = (pad_samples[0] * m_ss_channel1_volume) >> 8;
  pad_samples[1] = (pad_samples[1] * m_ss_channel0_volume) >> 8;

  // Flush cached variable
  // Memory Order: We need release ordering here so that index updates strictly
  //   happen after we are finished copying from the FIFO rather than at any
  //   arbitrary point decided by CPU/Compiler reordering.
  m_indexR.store(indexR, std::memory_order_release);

  unsigned int frames_written = currentSample / 2;

  // If there was a message and we are at the point stated in the message
  // then apply the state changes.
  if (raw_indexW != indexW && indexW - indexR <= 2)
  {
    const ControlMessage& pkt = m_control_messages.Front();
    m_ss_channel0_volume = pkt.channel0_volume;
    m_ss_channel1_volume = pkt.channel1_volume;
    m_ss_sample_rate = pkt.sample_rate;
    m_control_messages.Pop();

    // If we need to supply more audio to our caller then we will be forced
    // to recurse and try again. This is simpler than creating a wrapper loop
    // which is almost never going to be used anyway.
    if (frames_written < numSamples)
      return frames_written +
             Mix(&samples[currentSample], numSamples - frames_written, consider_framelimit);
  }

  // Padding
  for (; currentSample < numSamples * 2; ++currentSample)
  {
    samples[currentSample] =
        MathUtil::Clamp(samples[currentSample] + pad_samples[currentSample & 1], -32768, 32767);
  }

  return frames_written;
}

bool CMixer::MixerFifo::UpdateControl()
{
  if (m_control_messages.Empty())
    return false;

  const ControlMessage& msg = m_control_messages.Front();
  u32 read_idx = m_indexR.load(std::memory_order_relaxed);
  // 4 instead of 2 because of the resampler latency RoundUp((1 + m_frac) * 2)
  if (msg.attached_index - read_idx > 4)
    return false;

  m_ss_channel0_volume = msg.channel0_volume;
  m_ss_channel1_volume = msg.channel1_volume;
  m_ss_sample_rate = msg.sample_rate;
  m_control_messages.Pop();
  return true;
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

  // Find out how many samples we have available and pull that many at most.
  // The primary condition we have to deal with is a race between DMA and DTK
  // where we try to mix after the DMA has been written but before the DTK
  // CoreTiming callback has added the DTK audio or vice versa. As such, we
  // must block forward progress if either the DMA or DTK is empty. [Those
  // FIFOs are always written by the CPU, it writes silence if it has nothing
  // else to put in it so we can safely rely on these 2 FIFOs to run in
  // lock-step reliably as long as we force synchronization here]
  unsigned int fetch_size = 0;
  do
  {
    unsigned int dma_available = m_dma_mixer.AvailableSamples();
    unsigned int dtk_available = m_streaming_mixer.AvailableSamples();
    fetch_size = std::min({num_samples, dma_available, dtk_available});
    // NOTE: Bitwise-OR is intentional, we always want to run both functions.
  } while (!fetch_size && (m_dma_mixer.UpdateControl() | m_streaming_mixer.UpdateControl()));

  std::fill_n(samples, num_samples * 2, 0);
  if (!fetch_size)
    return 0;

  m_dma_mixer.Mix(samples, fetch_size, false);
  m_streaming_mixer.Mix(samples, fetch_size, false);
  m_wiimote_speaker_mixer.Mix(samples, fetch_size, false);
  return fetch_size;
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
  // IMPORTANT: It's possible to get (indexW & INDEX_MASK) == (indexR & INDEX_MASK)
  //   where indexW != indexR. i.e. indexW - indexR == BUFFER_SIZE. This is unusual
  //   for most ring buffer implementations, it's possible here because the cursors
  //   are much wider than needed to store an index so the extra bits can be used to
  //   disambiguate the overlapping cursor positions.
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

  if (m_cpu_pending_message)
  {
    m_control_messages.Push(
        ControlMessage{indexW, m_cpu_channel0_volume, m_cpu_channel1_volume, m_cpu_sample_rate});
    m_cpu_pending_message = false;
  }

  m_indexW.store(indexW + num_samples, std::memory_order_release);  // release m_buffer
}

void CMixer::PushSamples(const short* samples, unsigned int num_samples)
{
  m_dma_mixer.PushSamples(samples, num_samples);
  int sample_rate = m_dma_mixer.GetInputSampleRateCPU();
  if (m_log_dsp_audio)
    m_wave_writer_dsp.AddStereoSamplesBE(samples, num_samples, sample_rate);
}

void CMixer::PushStreamingSamples(const short* samples, unsigned int num_samples)
{
  m_streaming_mixer.PushSamples(samples, num_samples);
  int sample_rate = m_streaming_mixer.GetInputSampleRateCPU();
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

void CMixer::SetStreamingVolume(unsigned int vol0, unsigned int vol1)
{
  m_streaming_mixer.SetVolume(vol0, vol1);
}

void CMixer::SetWiimoteSpeakerVolume(unsigned int vol0, unsigned int vol1)
{
  m_wiimote_speaker_mixer.SetVolume(vol0, vol1);
}

void CMixer::StartLogDTKAudio(const std::string& filename)
{
  if (!m_log_dtk_audio)
  {
    m_log_dtk_audio = true;
    m_wave_writer_dtk.Start(filename, m_streaming_mixer.GetInputSampleRateCPU());
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
    m_wave_writer_dsp.Start(filename, m_dma_mixer.GetInputSampleRateCPU());
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
  if (rate == m_cpu_sample_rate)
    return;

  m_cpu_sample_rate = rate;
  m_cpu_pending_message = true;
}

void CMixer::MixerFifo::SetVolume(unsigned int vol0, unsigned int vol1)
{
  vol0 += (vol0 >> 7);
  vol1 += (vol1 >> 7);
  if (vol0 == m_cpu_channel0_volume && vol1 == m_cpu_channel1_volume)
    return;

  m_cpu_channel0_volume = vol0;
  m_cpu_channel1_volume = vol1;
  m_cpu_pending_message = true;
}
