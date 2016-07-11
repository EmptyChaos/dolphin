// Copyright 2009 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#pragma once

#include <array>
#include <atomic>

#include "AudioCommon/WaveFile.h"
#include "Common/CommonTypes.h"
#include "Common/FifoQueue.h"

class CMixer final
{
public:
  explicit CMixer(unsigned int BackendSampleRate);
  ~CMixer();

  // Called from audio threads
  unsigned int Mix(short* samples, unsigned int numSamples, bool consider_framelimit = true);
  unsigned int MixAvailable(short* samples, unsigned int numSamples);  // Ignores framelimit

  // Called from main thread
  void PushSamples(const short* samples, unsigned int num_samples);
  void PushStreamingSamples(const short* samples, unsigned int num_samples);
  void PushWiimoteSpeakerSamples(const short* samples, unsigned int num_samples,
                                 unsigned int sample_rate);
  unsigned int GetSampleRate() const { return m_sampleRate; }
  void SetDMAInputSampleRate(unsigned int rate);
  void SetStreamInputSampleRate(unsigned int rate);
  void SetStreamingVolume(unsigned int lvolume, unsigned int rvolume);
  void SetWiimoteSpeakerVolume(unsigned int lvolume, unsigned int rvolume);

  void StartLogDTKAudio(const std::string& filename);
  void StopLogDTKAudio();

  void StartLogDSPAudio(const std::string& filename);
  void StopLogDSPAudio();

private:
  static constexpr u32 MAX_SAMPLES = 1024 * 4;  // 128 ms
  static constexpr u32 INDEX_MASK = MAX_SAMPLES * 2 - 1;
  static constexpr int MAX_FREQ_SHIFT = 200;  // Per 32000 Hz
  static constexpr float CONTROL_FACTOR = 0.2f;
  static constexpr u32 CONTROL_AVG = 32;  // In freq_shift per FIFO size offset

  class MixerFifo final
  {
  public:
    MixerFifo(CMixer* mixer, unsigned sample_rate)
        : m_mixer(mixer), m_cpu_sample_rate(sample_rate), m_ss_sample_rate(sample_rate)
    {
    }
    void PushSamples(const short* samples, unsigned int num_samples);
    unsigned int Mix(short* samples, unsigned int numSamples, bool consider_framelimit = true);
    bool UpdateControl();
    unsigned int AvailableSamples() const;
    void SetInputSampleRate(unsigned int rate);
    unsigned int GetInputSampleRateSoundStream() const { return m_ss_sample_rate; }
    unsigned int GetInputSampleRateCPU() const { return m_cpu_sample_rate; }
    void SetVolume(unsigned int lvolume, unsigned int rvolume);

  private:
    struct ControlMessage
    {
      // Ring Buffer coordinate where the message needs to be processed.
      u32 attached_index;

      s32 left_volume;
      s32 right_volume;
      unsigned int sample_rate;
    };

    CMixer* m_mixer;
    std::array<short, MAX_SAMPLES * 2> m_buffer{};
    Common::FifoQueue<ControlMessage, false> m_control_messages;

    // Volume ranges from 0-256
    // SoundStream side
    unsigned int m_ss_sample_rate;
    s32 m_ss_left_volume{256};
    s32 m_ss_right_volume{256};
    float m_numLeftI = 0.0f;
    u32 m_frac = 0;

    // CPU side
    s32 m_cpu_left_volume{256};
    s32 m_cpu_right_volume{256};
    unsigned int m_cpu_sample_rate;
    bool m_cpu_pending_message{false};

    // Ring Buffer implementation.
    std::atomic<u32> m_indexW{0};
    std::atomic<u32> m_indexR{0};
  };
  MixerFifo m_dma_mixer{this, 32000};
  MixerFifo m_streaming_mixer{this, 48000};
  MixerFifo m_wiimote_speaker_mixer{this, 3000};
  unsigned int m_sampleRate;

  WaveFileWriter m_wave_writer_dtk;
  WaveFileWriter m_wave_writer_dsp;

  bool m_log_dtk_audio = false;
  bool m_log_dsp_audio = false;
};
