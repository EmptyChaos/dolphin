// Copyright 2008 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include "Core/HLE/HLE_Misc.h"
#include "Common/CommonTypes.h"
#include "Common/Logging/Log.h"
#include "Common/MsgHandler.h"
#include "Core/GeckoCode.h"
#include "Core/HW/CPU.h"
#include "Core/Host.h"
#include "Core/PowerPC/PowerPC.h"

namespace HLE_Misc
{
// If you just want to kill a function, one of the three following are usually appropriate.
// According to the PPC ABI, the return value is always in r3.
void UnimplementedFunction()
{
  NPC = LR;
}

// If you want a function to panic, you can rename it PanicAlert :p
// Don't know if this is worth keeping.
void HLEPanicAlert()
{
  ::PanicAlert("HLE: PanicAlert %08x", LR);
  NPC = LR;
}

void HBReload()
{
  // There isn't much we can do. Just stop cleanly.
  CPU::Break();
  Host_Message(WM_USER_STOP);
}

void GeckoCodeHandlerICacheFlush()
{
  // Work around the codehandler not properly invalidating the icache, but
  // only the first few frames.
  // (Project M uses a conditional to only apply patches after something has
  // been read into memory, or such, so we do the first 5 frames.  More
  // robust alternative would be to actually detect memory writes, but that
  // would be even uglier.)
  u32 gch_gameid = PowerPC::HostRead_U32(Gecko::INSTALLER_BASE_ADDRESS);
  if (gch_gameid - Gecko::MAGIC_GAMEID == 5)
  {
    return;
  }
  else if (gch_gameid - Gecko::MAGIC_GAMEID > 5)
  {
    gch_gameid = Gecko::MAGIC_GAMEID;
  }
  PowerPC::HostWrite_U32(gch_gameid + 1, Gecko::INSTALLER_BASE_ADDRESS);

  PowerPC::ppcState.iCache.Reset();
}

// _write function in the Gecko Code Handler. (HLE_HOOK_START)
// This hook is needed because Dolphin retains compiled JIT blocks more aggressively than the
// real CPU instruction cache on the GC/Wii. This means cheats that modify instructions differently
// over time (i.e. button conditional codes) don't work in Dolphin because the JIT ignores changes.
void GeckoWriteICacheFlush()
{
  // See _write in Gecko OS codehandleronly.s
  // r3 contains the pointer offset, r12 contains the base pointer,
  // r5 contains the subtype, r4 contains the second code word,
  // r15 contains the code list pointer (points to next code line after current one).
  // CR7 contains the execution enable state (Equal = on, NotEqual = off)
  if (!GetCRBit(30))  // CR7[EQ]
    return;

  u32 pointer = GPR(12) + GPR(3);
  u32 length = 0;
  switch (GPR(5))  // Subtype is 3bits (0-7)
  {
  case 0:  // _write816 (u8 ), Y times [00XXXXXX YYYY00ZZ]
  case 1:  // _write816 (u16), Y times [02XXXXXX YYYYZZZZ]
    length = (GPR(4) >> 16) << GPR(5);
    break;

  case 2:  // _write32 [04XXXXXX ZZZZZZZZ]
    length = 4;
    break;

  case 3:  // _write_string [06XXXXXX YYYYYYYY AAAAAAAA BBBBBBBB CCCCCCCC ...]
    length = GPR(4);
    break;

  case 4:  // _write_serial [08XXXXXX YYYYYYYY TNNNZZZZ VVVVVVVV]
  {
    // "Write Serial" is a non-contiguous pattern fill. The instruction is double length.
    // NNN is the iteration count, ZZZZ is the stride.
    u32 line2_addr = PowerPC::HostRead_U32(GPR(15));
    u32 iterations = (line2_addr >> 16) & 0xFFF;
    u32 stride = line2_addr & 0xFFFF;
    while (iterations--)
    {
      PowerPC::ppcState.iCache.Invalidate(pointer);
      pointer += stride;
    }
    return;
  }

  default:
    return;
  }

  // Align length to cache lines
  u32 iterations = static_cast<u32>(static_cast<u64>(length) + (pointer & 31) + 31) / 32;
  for (; iterations--; pointer += 32)
  {
    PowerPC::ppcState.iCache.Invalidate(pointer);
  }
}

// Because Dolphin messes around with the CPU state instead of patching the game binary, we
// need a way to branch into the GCH from an arbitrary PC address. Branching is easy, returning
// back is the hard part. This HLE function acts as a trampoline that restores the original LR, SP,
// and PC before the magic, invisible BL instruction happened.
void GeckoReturnTrampoline()
{
  // Stack frame is built in GeckoCode.cpp, Gecko::RunCodeHandler.
  u32 SP = GPR(1);
  GPR(1) = PowerPC::HostRead_U32(SP + 8);
  NPC = PowerPC::HostRead_U32(SP + 12);
  LR = PowerPC::HostRead_U32(SP + 16);
  PowerPC::ExpandCR(PowerPC::HostRead_U32(SP + 20));
  for (int i = 0; i < 14; ++i)
  {
    riPS0(i) = PowerPC::HostRead_U64(SP + 24 + 2 * i * sizeof(u64));
    riPS1(i) = PowerPC::HostRead_U64(SP + 24 + (2 * i + 1) * sizeof(u64));
  }
}
}
