// Copyright 2008 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#pragma once

#include <thread>

#include "AudioCommon/SoundStream.h"
#include "Common/Flag.h"

class AOSound final : public SoundStream
{
#if defined(HAVE_AO) && HAVE_AO
  std::thread thread;
  Common::Flag m_run_thread;

public:
  bool Start() override;
  void SoundLoop() override;
  void Stop() override;

  static bool isValid() { return true; }
#endif
};
