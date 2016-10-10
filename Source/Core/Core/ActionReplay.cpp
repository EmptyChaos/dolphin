// Copyright 2008 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

// -----------------------------------------------------------------------------------------
// Partial Action Replay code system implementation.
// Will never be able to support some AR codes - specifically those that patch the running
// Action Replay engine itself - yes they do exist!!!
// Action Replay actually is a small virtual machine with a limited number of commands.
// It probably is Turing complete - but what does that matter when AR codes can write
// actual PowerPC code...
// -----------------------------------------------------------------------------------------

// -------------------------------------------------------------------------------------------------------------
// Code Types:
// (Unconditional) Normal Codes (0): this one has subtypes inside
// (Conditional) Normal Codes (1 - 7): these just compare values and set the line skip info
// Zero Codes: any code with no address.  These codes are used to do special operations like memory
// copy, etc
// -------------------------------------------------------------------------------------------------------------

#include <algorithm>
#include <cstring>
#include <iterator>
#include <mutex>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "Common/Assert.h"
#include "Common/CommonTypes.h"
#include "Common/IniFile.h"
#include "Common/Logging/Log.h"
#include "Common/MsgHandler.h"
#include "Common/StringUtil.h"

#include "Core/ARDecrypt.h"
#include "Core/ActionReplay.h"
#include "Core/ConfigManager.h"
#include "Core/Core.h"
#include "Core/PowerPC/PowerPC.h"

namespace ActionReplay
{
enum
{
  // Conditional Codes
  CONDITIONAL_EQUAL = 0x01,
  CONDITIONAL_NOT_EQUAL = 0x02,
  CONDITIONAL_LESS_THAN_SIGNED = 0x03,
  CONDITIONAL_GREATER_THAN_SIGNED = 0x04,
  CONDITIONAL_LESS_THAN_UNSIGNED = 0x05,
  CONDITIONAL_GREATER_THAN_UNSIGNED = 0x06,
  CONDITIONAL_AND = 0x07,  // bitwise AND

  // Conditional Line Counts
  CONDITIONAL_ONE_LINE = 0x00,
  CONDITIONAL_TWO_LINES = 0x01,
  CONDITIONAL_DISABLE_EXECUTION = 0x02,
  CONDITIONAL_ABORT_CODE = 0x03,
};

enum DataType
{
  DATATYPE_8BIT = 0x00,
  DATATYPE_16BIT = 0x01,
  DATATYPE_32BIT = 0x02,
  DATATYPE_32BIT_FLOAT = 0x03,
};

// General lock. Protects codes list and internal log.
static std::mutex s_lock;
static std::vector<ARCode> s_active_codes;
static std::vector<std::string> s_internal_log;
static bool s_use_internal_log = false;
static u32 s_code_list_version = 0;
static u32 s_last_code_list_version = 0;

struct ARAddr
{
  union {
    u32 hex;
    struct
    {
      u32 gcaddr : 25;
      u32 size : 2;
      u32 type : 3;
      u32 subtype : 2;
    };
  };

  explicit ARAddr(const u32 addr) : hex(addr) {}
  u32 GCAddress() const { return gcaddr | 0x80000000; }
  u32 MMIOAddress(bool wii) const
  {
    return (gcaddr & 0x00FFFFFF) | (wii ? 0xCD000000 : 0xCC000000);
  }
  u32 ControlBits() const { return (hex >> 24) & 0xFE; }
  DataType GetDataType() const { return static_cast<DataType>(size); }
  u32 GetDataSizeBytes() const { return 1 << std::min(GetDataType(), DATATYPE_32BIT); }
  operator u32() const { return hex; }
  const char* SizeAsString(bool allow_float = true) const
  {
    switch (size)
    {
    case DATATYPE_8BIT:
      return "U8";
    case DATATYPE_16BIT:
      return "U16";
    case DATATYPE_32BIT_FLOAT:
      if (allow_float)
        return "F32";
    // fallthrough
    case DATATYPE_32BIT:
      return "U32";
    }
    // Unreachable
    return "";
  }
};

// ----------------------
// AR Remote Functions
void ApplyCodes(const std::vector<ARCode>& codes)
{
  std::lock_guard<std::mutex> guard(s_lock);
  s_active_codes.clear();
  ++s_code_list_version;
  if (SConfig::GetInstance().bEnableCheats)
  {
    s_active_codes.reserve(codes.size());
    std::copy_if(codes.begin(), codes.end(), std::back_inserter(s_active_codes),
                 [](const ARCode& code) { return code.active && !code.ops.empty(); });
  }
  s_active_codes.shrink_to_fit();
}

void AddCode(ARCode code)
{
  if (!SConfig::GetInstance().bEnableCheats)
    return;

  if (code.active)
  {
    std::lock_guard<std::mutex> guard(s_lock);
    s_active_codes.emplace_back(std::move(code));
  }
}

void LoadAndApplyCodes(const IniFile& global_ini, const IniFile& local_ini)
{
  ApplyCodes(LoadCodes(global_ini, local_ini));
}

// Parses the Action Replay section of a game ini file.
std::vector<ARCode> LoadCodes(const IniFile& global_ini, const IniFile& local_ini)
{
  std::vector<ARCode> codes;

  std::unordered_set<std::string> enabled_names;
  {
    std::vector<std::string> enabled_lines;
    local_ini.GetLines("ActionReplay_Enabled", &enabled_lines);
    for (const std::string& line : enabled_lines)
    {
      if (line.size() != 0 && line[0] == '$')
      {
        std::string name = line.substr(1, line.size() - 1);
        enabled_names.insert(name);
      }
    }
  }

  const IniFile* inis[2] = {&global_ini, &local_ini};
  for (const IniFile* ini : inis)
  {
    std::vector<std::string> lines;
    std::vector<std::string> encrypted_lines;
    ARCode current_code;

    ini->GetLines("ActionReplay", &lines);

    for (const std::string& line : lines)
    {
      if (line.empty())
      {
        continue;
      }

      std::vector<std::string> pieces;

      // Check if the line is a name of the code
      if (line[0] == '$')
      {
        if (current_code.ops.size())
        {
          codes.push_back(current_code);
          current_code.ops.clear();
        }
        if (encrypted_lines.size())
        {
          DecryptARCode(encrypted_lines, &current_code.ops);
          codes.push_back(current_code);
          current_code.ops.clear();
          encrypted_lines.clear();
        }

        current_code.name = line.substr(1, line.size() - 1);
        current_code.active = enabled_names.find(current_code.name) != enabled_names.end();
        current_code.user_defined = (ini == &local_ini);
      }
      else
      {
        SplitString(line, ' ', pieces);

        // Check if the AR code is decrypted
        if (pieces.size() == 2 && pieces[0].size() == 8 && pieces[1].size() == 8)
        {
          AREntry op;
          bool success_addr = TryParse(std::string("0x") + pieces[0], &op.cmd_addr);
          bool success_val = TryParse(std::string("0x") + pieces[1], &op.value);

          if (success_addr && success_val)
          {
            current_code.ops.push_back(op);
          }
          else
          {
            PanicAlertT("Action Replay Error: invalid AR code line: %s", line.c_str());

            if (!success_addr)
              PanicAlertT("The address is invalid");

            if (!success_val)
              PanicAlertT("The value is invalid");
          }
        }
        else
        {
          SplitString(line, '-', pieces);
          if (pieces.size() == 3 && pieces[0].size() == 4 && pieces[1].size() == 4 &&
              pieces[2].size() == 5)
          {
            // Encrypted AR code
            // Decryption is done in "blocks", so we must push blocks into a vector,
            // then send to decrypt when a new block is encountered, or if it's the last block.
            encrypted_lines.emplace_back(pieces[0] + pieces[1] + pieces[2]);
          }
        }
      }
    }

    // Handle the last code correctly.
    if (current_code.ops.size())
    {
      codes.push_back(current_code);
    }
    if (encrypted_lines.size())
    {
      DecryptARCode(encrypted_lines, &current_code.ops);
      codes.push_back(current_code);
    }
  }

  return codes;
}

void SaveCodes(IniFile* local_ini, const std::vector<ARCode>& codes)
{
  std::vector<std::string> lines;
  std::vector<std::string> enabled_lines;
  for (const ActionReplay::ARCode& code : codes)
  {
    if (code.active)
      enabled_lines.emplace_back("$" + code.name);

    if (code.user_defined)
    {
      lines.emplace_back("$" + code.name);
      for (const ActionReplay::AREntry& op : code.ops)
      {
        lines.emplace_back(StringFromFormat("%08X %08X", op.cmd_addr, op.value));
      }
    }
  }
  local_ini->SetLines("ActionReplay_Enabled", enabled_lines);
  local_ini->SetLines("ActionReplay", lines);
}

void EnableSelfLogging(bool enable)
{
  std::lock_guard<std::mutex> guard(s_lock);
  s_use_internal_log = enable;
}

std::vector<std::string> GetSelfLog()
{
  std::lock_guard<std::mutex> guard(s_lock);
  return s_internal_log;
}

void ClearSelfLog()
{
  std::lock_guard<std::mutex> guard(s_lock);
  s_internal_log.clear();
}

bool IsSelfLogging()
{
  std::lock_guard<std::mutex> guard(s_lock);
  return s_use_internal_log;
}

// ----------------------
// Code Functions
enum class Instruction
{
  // Unrecognized code
  Unknown,

  // -------------
  // ZERO CODES
  // Address field is zero. Key feature is that the codes may take up 2 rows instead of 1.

  // 00000000 00000000
  // Exit all codes
  AbortExecution,

  // 00000000 40000000
  // ??? Turn on the Execution Enable register (disable instruction skipping)
  UnlockExecution,

  // 00000000 60000000
  // ??? Clear Execution Enable register (Skip instructions until enabled again)
  LockExecution,

  // 00000000 8XXXXXXX : X = 0x80000000 + (address & 0x01FFFFFF), size = 1 << ((X >> 25) & 3)
  // Y1Y2Y3Y4 Z1Z2Z3Z4 : Y = value, Z1 = (value += Z1), Z2 = iteration count,
  //                     Z3Z4 = (address += size * Z3Z4)
  StridedMemFill,

  // 00000000 8XXXXXXX : See Memset [Memcpy is "Memset with Size = 3"]
  // YYYYYYYY 0000ZZZZ : Y = Source Address, Z = bytes to copy
  // memcpy((U8*)destination address, (U8*)source address, bytes to copy)
  Memcpy,

  // 00000000 8XXXXXXX : See Memset [Memcpy is "Memset with Size = 3"]
  // YYYYYYYY 0100ZZZZ : Y = Source Pointer, Z = bytes to copy
  // memcpy((U8*)(*(U32*)destination address), (U8*)(*(U32*)source address), bytes to copy)
  MemcpyIndirect,

  // --------------
  // STANDARD CODES
  //
  // struct Address { subtype : 2; type : 3; size : 2; address : 25; }
  // real_address = address | 0x80000000
  // XXXXXXXX Y1Y2Y3Y4 : X = struct Address

  // [type = 0, subtype = 0, size = 0]
  //   memset((U8*)address, Y4, Y1Y2Y3)
  // [type = 0, subtype = 0, size = 1]
  //   for (size_t i = 0; i < Y1Y2; ++i) { *(U16*)(address + i * 2) = Y3Y4; }
  Memset,

  // [type = 0, subtype = 0, size = 2] *(U32*)address = Y1Y2Y3Y4
  // [type = 0, subtype = 0, size = 3] *(U32*)address = Y1Y2Y3Y4
  WriteImmediate,

  // [type = 0, subtype = 1, size = 0] *(U8*)(*(U32*)(address + Y1Y2Y3)) = Y4
  // [type = 0, subtype = 1, size = 1] *(U16*)(*(U32*)(address + Y1Y2)) = Y3Y4
  // [type = 0, subtype = 1, size = 2] *(U32*)(*(U32*)address) = Y1Y2Y3Y4
  WriteIndirect,

  // [type = 0, subtype = 2, size = 0] *(U8*)address += Y4
  // [type = 0, subtype = 2, size = 1] *(U16*)address += Y3Y4
  // [type = 0, subtype = 2, size = 2] *(U32*)address += Y1Y2Y3Y4
  // [type = 0, subtype = 2, size = 3] *(float*)address += (float)Y1Y2Y3Y4
  Addition,

  // type = 0, subtype = 3, size = 2 : Master Code [Not supported]
  // C4XXXXXX Y1Y2Y3Y4
  // Y4 = ID, 0 = Run once during boot, 1-15 = Run every time (when?)
  //      Only one ID0 is allowed, adjacent master codes with same ID are skipped after first one.
  // Y3 = Number of instructions to run per call from this hook
  // (Y2 & 3) = Hook Type
  // Y1 = Encryption Seed
  // X = Hook location, usage depends on Hook Type
  MasterCode,

  // [type = 0, subtype = 3, size = 3, raw_address >= 0x01000000] WriteMMIO_U32
  //   *(U32*)(raw_address + MMIO_BASE) = Y1Y2Y3Y4
  // [type = 0, subtype = 3, size = 3, raw_address < 0x01000000] WriteMMIO_U16
  //   *(U16*)(raw_address + MMIO_BASE) = Y3Y4
  // This is not supported.
  WriteMMIO,

  // ----------------
  // COMPARISON CODES
  //
  // Comparison codes come in 4 types:
  //   Protect next line (subtype = 0)
  //   Protect next 2 lines (subtype = 1)
  //   Clear Execution Enable Register (subtype = 2)
  //   Exit code (subtype = 3)

  // [type = 1, subtype/size = <Any>] *(U32*)address == Y1Y2Y3Y4
  // [type = 2, subtype/size = <Any>] *(U32*)address != Y1Y2Y3Y4
  // [type = 3, subtype/size = <Any>] *(S32*)address < (S32)Y1Y2Y3Y4 (signed)
  // [type = 4, subtype/size = <Any>] *(S32*)address > (S32)Y1Y2Y3Y4
  // [type = 5, subtype/size = <Any>] *(U32*)address < (U32)Y1Y2Y3Y4 (unsigned)
  // [type = 6, subtype/size = <Any>] *(U32*)address > (U32)Y1Y2Y3Y4
  // [type = 7, subtype/size = <Any>] (*(U32*)address & Y1Y2Y3Y4) != 0
  CompareAndBranch,
};

// This class is modeled in the style of a Virtual Machine since the Action Replay itself seems
// to work this way (so do Gecko Codes more or less).
class ExecutionEngine
{
public:
  enum class Logging
  {
    None,
    InfoLog,
    InternalLog,
    All
  };

  explicit ExecutionEngine(const ARCode& code, Logging log_mode = Logging::None,
                           bool flush_icache = false)
      : m_code(code), m_use_log(log_mode), m_flush_icache(flush_icache)
  {
  }

  bool Execute()
  {
    LogInfo("Action Replay Code: %s", m_code.name.c_str());
    LogInfo("Number of Lines: %zu", m_code.ops.size());

    m_ARPC = m_ARNPC = 0;
    while (m_ARPC < m_code.ops.size())
    {
      auto insn = AnalyzeInstruction(m_ARPC);
      m_ARNPC = m_ARPC + insn.lines;
      if (ShouldExecute(insn.op))
      {
        const ARAddr addr{m_code.ops[m_ARPC].cmd_addr};
        const u32 data = m_code.ops[m_ARPC].value;

        LogInfo("--- Running Code: %08x %08x ---", addr.hex, data);
        if (!(this->*insn.handler)(addr, data))
        {
          LogInfo("Error on Line: %zu", m_ARPC + 1);
          ERROR_LOG(ACTIONREPLAY, "Code %s: Line %zu failed (%08X %08X)", m_code.name.c_str(),
                    m_ARPC + 1, addr.hex, data);
          return false;
        }
      }
      m_ARPC = m_ARNPC;
    }
    return true;
  }

  void TransferInternalLog(std::vector<std::string>* out_log)
  {
    out_log->reserve(out_log->size() + m_log.size() + 1);
    std::move(m_log.begin(), m_log.end(), std::back_inserter(*out_log));
    m_log.clear();
    out_log->emplace_back("\n");
  }

private:
  using InstructionFunc = bool (ExecutionEngine::*)(ARAddr, u32);

  struct InstructionAnalysis
  {
    Instruction op;
    InstructionFunc handler;
    u32 lines;
  };

  // Instruction Decoder
  InstructionAnalysis AnalyzeInstruction(std::size_t address) const;
  bool ShouldExecute(Instruction insn) const
  {
    return m_execution_enabled || insn == Instruction::UnlockExecution ||
           insn == Instruction::Unknown;
  }

  // Helper functions
  bool IsSelfModificationCode(u32 pointer, u32 length) const;
  bool CheckPointer(u32 pointer, u32 length = 4) const;
  std::pair<bool, u32> ReadPointer(u32 pointer) const;
  bool DoWriteToRAM(u32 address, u32 value, DataType type);
  bool DoMemcpy(u32 dest, u32 source, u32 length);
  bool CompareValues(const u32 val1, const u32 val2, const int type) const;
  void LogInfo(const char* format, ...) const;

  // Action Replay Instructions
  bool UnknownInstruction(ARAddr, u32);
  bool WriteImmediate(ARAddr, u32);
  bool Memset(ARAddr, u32);
  bool WriteMMIO(ARAddr, u32);
  bool WriteIndirect(ARAddr, u32);
  bool Addition(ARAddr, u32);
  bool CompareAndBranch(ARAddr, u32);
  bool MasterCode(ARAddr, u32);
  bool AbortExecution(ARAddr, u32);
  bool LockExecution(ARAddr, u32);
  bool UnlockExecution(ARAddr, u32);
  bool StridedMemFill(ARAddr, u32);
  bool Memcpy(ARAddr, u32);
  bool MemcpyIndirect(ARAddr, u32);

  mutable std::vector<std::string> m_log;
  Logging m_use_log = Logging::None;

  const ARCode& m_code;
  std::size_t m_ARPC = 0;           // Action Replay Program Counter
  std::size_t m_ARNPC = 0;          // Action Replay Next Program Counter
  bool m_execution_enabled = true;  // Execution Enable Register

  // ICache is only flushed the first time ARCodes run unless the code contains a conditional.
  // Conditionals require constant flushing since they often change PPC instructions over time
  // (e.g. button codes).
  bool m_flush_icache = false;
};

void ExecutionEngine::LogInfo(const char* format, ...) const
{
  if (m_use_log == Logging::None)
    return;

  va_list args;
  va_start(args, format);
  std::string s = StringFromFormatV(format, args);
  if (m_use_log != Logging::InternalLog)
    INFO_LOG(ACTIONREPLAY, "%s", s.c_str());
  va_end(args);

  if (m_use_log != Logging::InfoLog)
  {
    s += '\n';
    m_log.push_back(std::move(s));
  }
}

ExecutionEngine::InstructionAnalysis ExecutionEngine::AnalyzeInstruction(std::size_t idx) const
{
  const ARAddr addr{m_code.ops[idx].cmd_addr};

  if (addr.hex == 0)  // Zero Code
  {
    const ARAddr zero_addr{m_code.ops[idx].value};
    switch (zero_addr.ControlBits())
    {
    // 00000000 00000000 [type = 0, subtype = 0, size = 0]
    // Exit all codes
    case 0x00:
      if (zero_addr.hex == 0)
        return {Instruction::AbortExecution, &ExecutionEngine::AbortExecution, 1};
      break;

    // 00000000 40000000 [type = 0, subtype = 1, size = 0]
    // ??? Resume executing codes
    case 0x40:
      if (zero_addr.gcaddr == 0)
        return {Instruction::UnlockExecution, &ExecutionEngine::UnlockExecution, 1};
      break;

    // 00000000 60000000 [type = 4, subtype = 1, size = 0]
    // ??? Disable executing code until unlocked
    case 0x60:
      if (zero_addr.gcaddr == 0)
        return {Instruction::LockExecution, &ExecutionEngine::LockExecution, 1};
      break;

    case 0x80:  // StridedMemFill [type = 0, subtype = 2, size = 0 (u8 )]
    case 0x82:  // StridedMemFill [type = 0, subtype = 2, size = 1 (u16)]
    case 0x84:  // StridedMemFill [type = 0, subtype = 2, size = 2 (u32)]
    case 0x86:  // Memcpy [type = 0, subtype = 2, size = 3]
    {
      if (idx + 1 == m_code.ops.size())
      {
        LogInfo("Memcpy/StridedMemFill second code line is missing");
        PanicAlertT("Action Replay Error: [%s] Memcpy/StridedMemFill missing second line.",
                    m_code.name.c_str());
        break;
      }
      if (zero_addr.ControlBits() != 0x86)
        return {Instruction::StridedMemFill, &ExecutionEngine::StridedMemFill, 2};

      // Memcpy control bits
      // 0x0000 = Memcpy, 0xYY00 = MemcpyIndirect, non-zero low byte is an error
      const u32 control_bits = m_code.ops[idx + 1].value >> 16;
      if (!(control_bits & 0x00FF))
      {
        if (control_bits & 0xFF00)
          return {Instruction::MemcpyIndirect, &ExecutionEngine::MemcpyIndirect, 2};

        return {Instruction::Memcpy, &ExecutionEngine::Memcpy, 2};
      }
      return {Instruction::Unknown, &ExecutionEngine::UnknownInstruction, 2};
    }

    default:
      break;
    }
  }
  else if (addr.type == 0)  // Normal Code
  {
    switch (addr.subtype)
    {
    case 0:  // Write to RAM
      if (addr.size < DATATYPE_32BIT)
        return {Instruction::Memset, &ExecutionEngine::Memset, 1};
      return {Instruction::WriteImmediate, &ExecutionEngine::WriteImmediate, 1};

    case 1:  // Write to RAM through pointer
      return {Instruction::WriteIndirect, &ExecutionEngine::WriteIndirect, 1};

    case 2:  // Addition
      return {Instruction::Addition, &ExecutionEngine::Addition, 1};

    case 3:  // Master Code / MMIO Write
      if (addr.size == DATATYPE_32BIT_FLOAT)
        return {Instruction::WriteMMIO, &ExecutionEngine::WriteMMIO, 1};
      else if (addr.size == DATATYPE_32BIT)
        return {Instruction::MasterCode, &ExecutionEngine::MasterCode, 1};
      break;

    default:  // Unreachable
      break;
    }
  }
  else  // Comparison Code
  {
    return {Instruction::CompareAndBranch, &ExecutionEngine::CompareAndBranch, 1};
  }

  return {Instruction::Unknown, &ExecutionEngine::UnknownInstruction, 1};
}

static bool PointerRangeIntersect(u32 a, u32 alen, u32 b, u32 blen)
{
  u32 rebased = b - a;
  u32 rebased_end = rebased + blen;
  // rebased_end becomes less than rebased due to wraparound
  return rebased < alen || rebased_end < rebased;
}

inline bool ExecutionEngine::IsSelfModificationCode(u32 pointer, u32 length) const
{
  // Actually this is where the Gecko Code Handler is, since AR is simulated.
  static constexpr u32 AR_BASE = 0x80001800;
  static constexpr u32 AR_END = 0x80003000;
  static constexpr u32 AR_LENGTH = AR_END - AR_BASE;

  if (PointerRangeIntersect(AR_BASE, AR_LENGTH, pointer, length))
  {
    LogInfo(
        "This Action Replay simulator does not support codes that modify Action Replay itself.");
    PanicAlertT("Action Replay Error: [%s] This simulator does not support codes that modify "
                "the Action Replay itself.",
                m_code.name.c_str());
    return true;
  }
  return false;
}

bool ExecutionEngine::CheckPointer(u32 pointer, u32 length) const
{
  if (!PowerPC::HostIsRAMAddress(pointer, length))
  {
    LogInfo("Invalid Pointer Address: %08X -> %08X", pointer, pointer + length);
    PanicAlertT("Action Replay Error: [%s] Invalid Pointer %08X (%u bytes)", m_code.name.c_str(),
                pointer, length);
    return false;
  }

  if (IsSelfModificationCode(pointer, length))
  {
    LogInfo("Self Modification Address: %08X -> %08X", pointer, pointer + length);
    return false;
  }

  return true;
}

inline std::pair<bool, u32> ExecutionEngine::ReadPointer(u32 pointer) const
{
  if (!CheckPointer(pointer))
    return {false, 0};

  return {true, PowerPC::HostRead_U32(pointer)};
}

inline bool ExecutionEngine::DoWriteToRAM(u32 address, u32 value, DataType type)
{
  if (!CheckPointer(address, 1 << std::min(type, DATATYPE_32BIT)))
    return false;

  switch (type)
  {
  case DATATYPE_8BIT:
    LogInfo("Write %02X to %08X", value & 0xFF, address);
    PowerPC::HostWrite_U8(static_cast<u8>(value), address);
    break;

  case DATATYPE_16BIT:
    LogInfo("Write %04X to %08X", value & 0xFFFF, address);
    PowerPC::HostWrite_U16(static_cast<u16>(value), address);
    break;

  case DATATYPE_32BIT:
  case DATATYPE_32BIT_FLOAT:
    LogInfo("Write %08X to %08X", value, address);
    PowerPC::HostWrite_U32(value, address);
    break;

  default:  // Unreachable
    _assert_(false);
    return false;
  }

  if (m_flush_icache)
  {
    LogInfo("Invalidate Instruction Cache: %08X", address & ~31U);
    PowerPC::ppcState.iCache.Invalidate(address);
  }

  return true;
}

static void InvalidateICache(u32 pointer, u32 length)
{
  // Round up to next cache line
  length = (length + 31) & ~UINT32_C(31);
  for (u32 offset = 0; offset < length; offset += 32)
    PowerPC::ppcState.iCache.Invalidate(pointer + offset);
}

inline bool ExecutionEngine::DoMemcpy(u32 dest, u32 source, u32 length)
{
  if (!CheckPointer(dest, length) || !CheckPointer(source, length))
    return false;

  LogInfo("Copy [%08X, %08X) to [%08X, %08X)", source, source + length, dest, dest + length);

  // NOTE: It'd be nice if there was a PowerPC::Memcpy function that only calls TranslateAddress
  //   as needed (once per page).
  for (u32 offset = 0; offset < length; ++offset)
  {
    u8 byte = PowerPC::HostRead_U8(source + offset);
    PowerPC::HostWrite_U8(byte, dest + offset);
  }

  if (m_flush_icache)
  {
    LogInfo("Invalidate Instruction Cache: %08X -> %08X", dest & ~31U, (dest + length) & ~31U);
    InvalidateICache(dest, length);
  }
  return true;
}

bool ExecutionEngine::UnknownInstruction(ARAddr cmd, u32 data)
{
  LogInfo("Unknown Instruction: %08X %08X", cmd, data);
  PanicAlertT("Action Replay Error: [%s] Unknown Code %08X %08X", cmd, data);
  return false;
}

bool ExecutionEngine::WriteImmediate(ARAddr addr, u32 data)
{
  LogInfo("WriteImmediate %s", addr.SizeAsString(false));
  return DoWriteToRAM(addr.GCAddress(), data, addr.GetDataType());
}

bool ExecutionEngine::Memset(ARAddr addr, u32 data)
{
  u32 start = addr.GCAddress();
  u32 end = start;

  LogInfo("Memset (Write %s repeatedly)", addr.SizeAsString(false));

  switch (addr.size)
  {
  case DATATYPE_8BIT:
  {
    u32 address = start;
    end = address + (data >> 8);
    if (!CheckPointer(start, end - start))
      return false;

    u8 byte = static_cast<u8>(data);
    LogInfo("--------");
    for (; address < end; ++address)
    {
      PowerPC::HostWrite_U8(byte, address);
      LogInfo("Wrote %02X to %08X", byte, address);
    }
    LogInfo("--------");
  }
  break;

  case DATATYPE_16BIT:
  {
    u32 address = start;
    end = address + (data >> 16) * sizeof(u16);
    if (!CheckPointer(start, end - start))
      return false;

    u16 halfword = static_cast<u16>(data);
    LogInfo("--------");
    for (; address < end; address += sizeof(u16))
    {
      PowerPC::HostWrite_U16(halfword, address);
      LogInfo("Wrote %04X to %08X", halfword, address);
    }
    LogInfo("--------");
  }
  break;

  default:  // Unreachable
    _assert_(false);
    break;
  }

  if (m_flush_icache)
  {
    LogInfo("Invalidate Instruction Cache: %08X -> %08X", start, end);
    InvalidateICache(start, end - start);
  }

  return true;
}

bool ExecutionEngine::WriteMMIO(ARAddr addr, u32 data)
{
  u32 io_addr = addr.MMIOAddress(SConfig::GetInstance().bWii);
  if (addr.gcaddr >= 0x01000000)
  {
    LogInfo("MMIO Write U32: Attempt to write %08X to %08X", data, io_addr);
  }
  else
  {
    LogInfo("MMIO Write U16: Attempt to write %04X to %08X", data & 0xFFFF, io_addr);
  }
  PanicAlertT("Action Replay Error: [%s] Write to MMIO is not supported", m_code.name.c_str());
  return false;
}

bool ExecutionEngine::WriteIndirect(ARAddr addr, u32 data)
{
  LogInfo("WriteIndirect %s", addr.SizeAsString(false));

  // Resolve pointer to pointer: *(T**)addr.GCAddress()
  const auto target = ReadPointer(addr.GCAddress());
  if (!target.first)
    return false;

  // Apply offset
  u32 ptr = target.second;
  switch (addr.size)
  {
  case DATATYPE_8BIT:
    ptr += data >> 8;  // 24bit offset
    break;

  case DATATYPE_16BIT:
    ptr += data >> 16;  // 16bit offset
    break;

  case DATATYPE_32BIT:
  case DATATYPE_32BIT_FLOAT:
    break;  // 0bit offset

  default:  // Unreachable
    _assert_(false);
    return false;
  }

  return DoWriteToRAM(ptr, data, addr.GetDataType());
}

bool ExecutionEngine::Addition(ARAddr addr, u32 data)
{
  const u32 ptr = addr.GCAddress();

  LogInfo("Addition %s: %08X", addr.SizeAsString(true), ptr);
  if (!CheckPointer(ptr, addr.GetDataSizeBytes()))
    return false;

  switch (addr.size)
  {
  case DATATYPE_8BIT:
  {
    LogInfo("--------");
    u32 byte = PowerPC::HostRead_U8(ptr);
    u32 new_val = (byte + data) & 0xFF;
    PowerPC::HostWrite_U8(new_val, ptr);
    LogInfo("%u + %u = %u (%d + %d = %d)", byte, data & 0xFF, new_val, static_cast<s8>(byte),
            static_cast<s8>(data & 0xFF), static_cast<s8>(new_val));
    LogInfo("Wrote %02x to address %08x", new_val, ptr);
    LogInfo("--------");
  }
  break;

  case DATATYPE_16BIT:
  {
    LogInfo("--------");
    u32 halfword = PowerPC::HostRead_U16(ptr);
    u32 new_val = (halfword + data) & 0xFFFF;
    PowerPC::HostWrite_U16(new_val, ptr);
    LogInfo("%u + %u = %u (%d + %d = %d)", halfword, data & 0xFFFF, new_val,
            static_cast<s16>(halfword), static_cast<s16>(data & 0xFFFF), static_cast<s16>(new_val));
    LogInfo("Wrote %04x to address %08x", new_val, ptr);
    LogInfo("--------");
  }
  break;

  case DATATYPE_32BIT:
  {
    LogInfo("--------");
    u32 word = PowerPC::HostRead_U32(ptr);
    u32 new_val = word + data;
    PowerPC::HostWrite_U32(new_val, ptr);
    LogInfo("%u + %u = %u (%d + %d = %d)", word, data, new_val, static_cast<s32>(word),
            static_cast<s32>(data), static_cast<s32>(new_val));
    LogInfo("Wrote %08x to address %08x", new_val, ptr);
    LogInfo("--------");
  }
  break;

  case DATATYPE_32BIT_FLOAT:
  {
    LogInfo("--------");
    const u32 read = PowerPC::HostRead_U32(ptr);
    float read_float;
    std::memcpy(&read_float, &read, sizeof(float));
    float data_float;
    std::memcpy(&data_float, &data, sizeof(float));  // data contains a float
    float float_result = read_float + data_float;
    u32 u32_result;
    std::memcpy(&u32_result, &float_result, sizeof(u32));
    PowerPC::HostWrite_U32(u32_result, ptr);
    LogInfo("%g + %g = %g", read_float, data_float, float_result);
    LogInfo("Wrote %08X to %08X", u32_result, ptr);
    LogInfo("--------");
  }
  break;

  default:  // Unreachable
    _assert_(false);
    return false;
  }

  if (m_flush_icache)
  {
    LogInfo("Invalidate Instruction Cache: %08X", ptr);
    PowerPC::ppcState.iCache.Invalidate(ptr);
  }

  return true;
}

bool ExecutionEngine::CompareValues(const u32 val1, const u32 val2, const int type) const
{
  bool result = false;
  switch (type)
  {
  case CONDITIONAL_EQUAL:
    LogInfo("Type 1: If %08X == %08X", val1, val2);
    result = val1 == val2;
    break;

  case CONDITIONAL_NOT_EQUAL:
    LogInfo("Type 2: If %08X != %08X", val1, val2);
    result = val1 != val2;
    break;

  case CONDITIONAL_LESS_THAN_SIGNED:
    LogInfo("Type 3: If [signed] %d < %d", static_cast<s32>(val1), static_cast<s32>(val2));
    result = static_cast<s32>(val1) < static_cast<s32>(val2);
    break;

  case CONDITIONAL_GREATER_THAN_SIGNED:
    LogInfo("Type 4: If [signed] %d > %d", static_cast<s32>(val1), static_cast<s32>(val2));
    result = static_cast<s32>(val1) > static_cast<s32>(val2);
    break;

  case CONDITIONAL_LESS_THAN_UNSIGNED:
    LogInfo("Type 5: If [unsigned] %u < %u", val1, val2);
    result = val1 < val2;
    break;

  case CONDITIONAL_GREATER_THAN_UNSIGNED:
    LogInfo("Type 6: If [unsigned] %u > %u", val1, val2);
    result = val1 > val2;
    break;

  case CONDITIONAL_AND:
    LogInfo("Type 7: If %08X & %08X", val1, val2);
    result = !!(val1 & val2);  // bitwise AND
    break;

  default:  // Unreachable
    _assert_(false);
    return false;
  }
  LogInfo("Result = %s", result ? "true" : "false");
  return result;
}

static u32 SignExtend(u32 val, u32 sign_bit)
{
  const u32 mask = 1 << sign_bit;
  const u32 value_mask = (mask << 1) - 1;
  return ((val & value_mask) ^ mask) - mask;
}

bool ExecutionEngine::CompareAndBranch(ARAddr addr, u32 data)
{
  const u32 ptr = addr.GCAddress();

  LogInfo("Conditional %s: %08X", addr.SizeAsString(false), ptr);
  if (!CheckPointer(ptr, addr.GetDataSizeBytes()))
    return false;

  bool continue_execution = true;
  std::size_t next_ARPC = m_ARNPC;
  switch (addr.subtype)
  {
  case CONDITIONAL_ONE_LINE:
  case CONDITIONAL_TWO_LINES:
  {
    std::size_t lines = m_code.ops.size() - m_ARNPC;
    std::size_t lines_needed = addr.subtype + 1;
    LogInfo("If False: Skip %zu Lines", lines);
    if (lines < lines_needed)
    {
      LogInfo("Cannot skip %zu lines, not enough code lines left.", lines_needed);
      PanicAlertT("Action Replay Error: [%s] Conditional code wants to skip %zu out of %zu lines.",
                  m_code.name.c_str(), lines_needed, lines);
      return false;
    }
    next_ARPC += lines_needed;
  }
  break;

  case CONDITIONAL_DISABLE_EXECUTION:
    LogInfo("If False: Clear Execution Enable Register");
    continue_execution = false;
    break;

  case CONDITIONAL_ABORT_CODE:
    LogInfo("If False: Exit Code");
    next_ARPC = m_code.ops.size();
    break;

  default:  // Unreachable
    _assert_(false);
    return false;
  }

  bool result = true;

  bool is_signed =
      addr.type == CONDITIONAL_GREATER_THAN_SIGNED || addr.type == CONDITIONAL_LESS_THAN_SIGNED;
  switch (addr.size)
  {
  case DATATYPE_8BIT:
  {
    u32 value = PowerPC::HostRead_U8(ptr);
    u32 comparend = data & 0xFF;
    if (is_signed)
    {
      value = SignExtend(value, 7);
      comparend = SignExtend(comparend, 7);
    }
    result = CompareValues(value, comparend, addr.type);
  }
  break;

  case DATATYPE_16BIT:
  {
    u32 value = PowerPC::HostRead_U16(ptr);
    u32 comparend = data & 0xFFFF;
    if (is_signed)
    {
      value = SignExtend(value, 15);
      comparend = SignExtend(comparend, 15);
    }
    result = CompareValues(value, comparend, addr.type);
  }
  break;

  case DATATYPE_32BIT_FLOAT:
  case DATATYPE_32BIT:
    result = CompareValues(PowerPC::HostRead_U32(ptr), data, addr.type);
    break;

  default:  // Unreachable
    _assert_(false);
    return false;
  }

  // If the condition is false then branch to end of protected region
  if (!result)
  {
    m_execution_enabled &= continue_execution;
    m_ARNPC = next_ARPC;
  }

  // Button codes will require cache flushing to work.
  m_flush_icache = true;

  return true;
}

bool ExecutionEngine::MasterCode(ARAddr addr, u32 data)
{
  const u32 hook_addr = addr.GCAddress();
  const u32 hook_id = data & 0x0F;
  const u32 codes_per_call = (data >> 8) & 0xFF;
  const u32 hook_type = (data >> 16) & 3;
  const u32 encryption_seed = data >> 24;

  LogInfo("Master Code: Address = %08X, HookType = %u, EncryptionSeed = %02X", hook_addr, hook_type,
          encryption_seed);
  LogInfo("  HookID = %X%s, CodesPerCall = %u", hook_id,
          hook_id != 0 ? " (Run always)" : " (Run at boot)", codes_per_call);

  PanicAlertT("Action Replay Error: [%s] Master Codes are not supported.\n"
              "Master codes are not needed, do not use master codes.",
              m_code.name.c_str());
  return false;
}

bool ExecutionEngine::AbortExecution(ARAddr, u32)
{
  LogInfo("ZCode: Abort");
  m_ARNPC = m_code.ops.size();
  return true;
}

bool ExecutionEngine::LockExecution(ARAddr, u32)
{
  LogInfo("ZCode: Clear execution enabled register");
  m_execution_enabled = false;
  return true;
}

bool ExecutionEngine::UnlockExecution(ARAddr, u32)
{
  LogInfo("ZCode: Set execution enabled register");
  m_execution_enabled = true;
  return true;
}

bool ExecutionEngine::StridedMemFill(ARAddr, u32 line0_data)
{
  const ARAddr addr{line0_data};
  const u32 value = m_code.ops[m_ARPC + 1].cmd_addr;
  const u32 control = m_code.ops[m_ARPC + 1].value;

  const u32 val_increment = (control >> 24) & 0xFF;
  const u32 iterations = (control >> 16) & 0xFF;
  const u32 addr_increment = (control & 0xFFFF) << addr.size;
  u32 address = addr.GCAddress();

  LogInfo("Stride MemFill %s", addr.SizeAsString(false));
  LogInfo(" Destination = %08X, Address Increment = %u", address, addr_increment);
  LogInfo(" Value = %08X, Value Increment = %u", value, val_increment);
  if (!CheckPointer(address, iterations << addr.size))
    return false;

  switch (addr.size)
  {
  case DATATYPE_8BIT:
  {
    u8 byte = static_cast<u8>(value);
    LogInfo("--------");
    for (u32 i = 0; i < iterations; ++i)
    {
      PowerPC::HostWrite_U8(byte, address);
      LogInfo("Wrote %02X to %08X", byte, address);

      address += addr_increment;
      byte = static_cast<u8>(byte + val_increment);
    }
    LogInfo("--------");
  }
  break;

  case DATATYPE_16BIT:
  {
    u16 halfword = static_cast<u16>(value);
    LogInfo("--------");
    for (u32 i = 0; i < iterations; ++i)
    {
      PowerPC::HostWrite_U16(halfword, address);
      LogInfo("Wrote %04X to %08X", halfword, address);

      address += addr_increment;
      halfword = static_cast<u16>(halfword + val_increment);
    }
    LogInfo("--------");
  }
  break;

  case DATATYPE_32BIT:
  {
    u32 word = value;
    LogInfo("--------");
    for (u32 i = 0; i < iterations; ++i)
    {
      PowerPC::HostWrite_U32(word, address);
      LogInfo("Wrote %08X to %08X", word, address);

      address += addr_increment;
      word += val_increment;
    }
    LogInfo("--------");
  }
  break;

  default:  // Unreachable
    _assert_(false);
    return false;
  }

  if (m_flush_icache)
  {
    address = addr.GCAddress();
    u32 last_line = UINT32_MAX;
    for (u32 i = 0; i < iterations; ++i)
    {
      u32 line = address & ~UINT32_C(31);
      if (line != last_line)
      {
        last_line = line;
        LogInfo("Invalidate Instruction Cache: %08X", line);
        PowerPC::ppcState.iCache.Invalidate(line);
      }
      address += addr_increment;
    }
  }

  return true;
}

bool ExecutionEngine::Memcpy(ARAddr, u32 line0_data)
{
  LogInfo("Memcpy");

  const ARAddr addr{line0_data};
  const u32 dest = addr.GCAddress();
  const u32 source = m_code.ops[m_ARPC + 1].cmd_addr;
  const u32 length = m_code.ops[m_ARPC + 1].value & 0xFFFF;

  return DoMemcpy(dest, source, length);
}

bool ExecutionEngine::MemcpyIndirect(ARAddr, u32 line0_data)
{
  LogInfo("MemcpyIndirect");

  const ARAddr addr{line0_data};
  const auto dest = ReadPointer(addr.GCAddress());
  if (!dest.first)
    return false;
  const auto source = ReadPointer(m_code.ops[m_ARPC + 1].cmd_addr);
  if (!source.first)
    return false;
  const u32 length = m_code.ops[m_ARPC + 1].value & 0xFFFF;

  return DoMemcpy(dest.second, source.second, length);
}

void RunAllActive()
{
  if (!SConfig::GetInstance().bEnableCheats)
    return;

  // Because there are PanicAlerts in the engine, there's a risk of deadlocking the GUI so we
  // have to run the engine outside the lock.
  std::unique_lock<std::mutex> lk(s_lock);

  // Since we modify the code list, we need to make it thread local so that will be safe
  std::vector<ARCode> code_list(std::move(s_active_codes));
  bool first_run = s_last_code_list_version != s_code_list_version;
  s_last_code_list_version = s_code_list_version;

  // Decide what logging scheme to use
  ExecutionEngine::Logging log_mode = ExecutionEngine::Logging::None;
  if (first_run)
    log_mode =
        s_use_internal_log ? ExecutionEngine::Logging::All : ExecutionEngine::Logging::InfoLog;

  lk.unlock();

  std::vector<std::string> log;
  code_list.erase(std::remove_if(code_list.begin(), code_list.end(),
                                 [&](const ARCode& code) {
                                   ExecutionEngine engine{code, log_mode, first_run};
                                   bool success = engine.Execute();
                                   engine.TransferInternalLog(&log);
                                   return !success;
                                 }),
                  code_list.end());

  lk.lock();
  if (s_use_internal_log)
  {
    // Append this log to the global log
    s_internal_log.reserve(s_internal_log.size() + log.size());
    std::move(log.begin(), log.end(), std::back_inserter(s_internal_log));
  }
  if (s_code_list_version == s_last_code_list_version)
  {
    std::swap(s_active_codes, code_list);
    // In case of AddCode() calls
    if (!code_list.empty())
      std::move(code_list.begin(), code_list.end(), std::back_inserter(s_active_codes));
  }
}

}  // namespace ActionReplay
