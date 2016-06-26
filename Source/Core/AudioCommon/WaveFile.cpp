// Copyright 2008 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include <algorithm>
#include <array>
#include <string>

#include "AudioCommon/WaveFile.h"
#include "Common/CommonTypes.h"
#include "Common/MsgHandler.h"
#include "Core/ConfigManager.h"

WaveFileWriter::WaveFileWriter()
{
}

WaveFileWriter::~WaveFileWriter()
{
  if (file)
    Stop();
}

bool WaveFileWriter::Start(const std::string& filename, unsigned int HLESampleRate)
{
  // Check if the file is already open
  if (file)
  {
    PanicAlertT("The file %s was already open, the file header will not be written.",
                filename.c_str());
    return false;
  }

  file.Open(filename, "wb");
  if (!file)
  {
    PanicAlertT("The file %s could not be opened for writing. Please check if it's already opened "
                "by another program.",
                filename.c_str());
    return false;
  }

  audio_size = 0;

  if (basename.empty())
    SplitPath(filename, nullptr, &basename, nullptr);

  current_sample_rate = HLESampleRate;

  // -----------------
  // Write file header
  // -----------------
  Write4("RIFF");
  Write(100 * 1000 * 1000);  // write big value in case the file gets truncated
  Write4("WAVE");
  Write4("fmt ");

  Write(16);          // size of fmt block
  Write(0x00020001);  // two channels, uncompressed

  const u32 sample_rate = HLESampleRate;
  Write(sample_rate);
  Write(sample_rate * 2 * 2);  // two channels, 16bit

  Write(0x00100004);
  Write4("data");
  Write(100 * 1000 * 1000 - 32);

  // We are now at offset 44
  if (file.Tell() != 44)
    PanicAlert("Wrong offset: %lld", (long long)file.Tell());

  return true;
}

void WaveFileWriter::Stop()
{
  // u32 file_size = (u32)ftello(file);
  file.Seek(4, SEEK_SET);
  Write(audio_size + 36);

  file.Seek(40, SEEK_SET);
  Write(audio_size);

  file.Close();
}

void WaveFileWriter::Write(u32 value)
{
  file.WriteArray(&value, 1);
}

void WaveFileWriter::Write4(const char* ptr)
{
  file.WriteBytes(ptr, 4);
}

void WaveFileWriter::AddStereoSamplesBE(const short* sample_data, u32 count, int sample_rate)
{
  static constexpr u32 STEREO_CHANNELS = 2;
  if (!file)
    PanicAlertT("WaveFileWriter - file not open.");

  if (skip_silence && std::all_of(sample_data, sample_data + count * STEREO_CHANNELS,
                                  [](short sample) { return !sample; }))
    return;

  if (sample_rate != current_sample_rate)
  {
    Stop();
    file_index++;
    std::stringstream filename;
    filename << File::GetUserPath(D_DUMPAUDIO_IDX) << basename << file_index << ".wav";
    Start(filename.str(), sample_rate);
    current_sample_rate = sample_rate;
  }

  static constexpr u32 NUM_PASS_FRAMES = 2048;
  std::array<short, NUM_PASS_FRAMES * STEREO_CHANNELS> buffer;

  for (u32 i = 0; i < count;)
  {
    u32 pass = std::min(NUM_PASS_FRAMES, count - i);
    for (u32 end = i + pass; i < end; ++i)
    {
      // Swap RL -> LR. Convert from BE.
      buffer[STEREO_CHANNELS * i] = Common::swap16(sample_data[STEREO_CHANNELS * i + 1]);
      buffer[STEREO_CHANNELS * i + 1] = Common::swap16(sample_data[STEREO_CHANNELS * i]);
    }

    if (!file.WriteBytes(buffer.data(), pass * sizeof(short) * STEREO_CHANNELS))
    {
      file.Clear();
      break;
    }
    audio_size += pass;
  }
}
