// Copyright 2008 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#pragma once

#include <string>

#include "Common/CommonTypes.h"
#include "Core/PowerPC/PowerPC.h"

namespace HLE
{
enum HookType
{
  HLE_HOOK_START = 0,    // Hook the beginning of the function and execute the function afterwards
  HLE_HOOK_REPLACE = 1,  // Replace the function with the HLE version
  HLE_HOOK_NONE = 2,     // Do not hook the function
};

enum HookFlag
{
  HLE_TYPE_GENERIC = 0,  // Miscellaneous function
  HLE_TYPE_DEBUG = 1,    // Debug output function
  HLE_TYPE_FIXED = 2,    // An arbitrary hook mapped to a fixed address instead of a symbol
};

void PatchFunctions();
void Clear();

// Returns the physical address of the hook (i.e. independent of the current MSR.IR setting)
PowerPC::TranslateResult Patch(u32 pc, const char* func_name);
void PatchPhysical(u32 pc, const char* func_name);
u32 UnPatch(const std::string& patch_name);
// UnPatch takes a PHYSICAL address (return value of Patch)
bool UnPatch(u32 paddr, const std::string& name = {});
void Execute(u32 current_PC, u32 function_idx);

u32 GetFunctionIndex(u32 em_address);
int GetFunctionTypeByIndex(u32 index);
int GetFunctionFlagsByIndex(u32 index);

bool IsEnabled(int flags);
}
