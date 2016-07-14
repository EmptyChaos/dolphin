// Copyright 2008 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include "AudioCommon/NullSoundStream.h"

// This does literally nothing.
// CMixer will accept audio until its internal buffer is full at which point it will just reject new
// audio being added. Audio dumping happens on the CPU thread so there's no reason to ever remove
// any data from CMixer.

void NullSound::SoundLoop()
{
}

bool NullSound::Start()
{
  return true;
}

void NullSound::SetVolume(int volume)
{
}

void NullSound::Clear(bool mute)
{
  m_muted = mute;
}

void NullSound::Stop()
{
}
