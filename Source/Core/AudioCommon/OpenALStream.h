// Copyright 2008 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#pragma once

#include <memory>

#include "AudioCommon/SoundStream.h"
#include "Common/Common.h"

class OpenALStream final : public SoundStream
{
#if defined HAVE_OPENAL && HAVE_OPENAL
public:
  OpenALStream();
  ~OpenALStream();

  bool Start() override;
  void SetVolume(int volume) override;
  void Stop() override;
  void Clear(bool mute) override;

  static bool isValid() { return true; }
private:
  class WorkerThread;

  std::unique_ptr<WorkerThread> m_worker;
#endif  // HAVE_OPENAL
};
