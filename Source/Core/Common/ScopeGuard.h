// Copyright 2015 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#pragma once

#include <utility>

namespace Common
{
// Generic generalized RAII helper object.
// It runs an arbitrary Functor (lambda usually) when it's destroyed.
// Use "auto var = Common::MakeScopeGuard([&] { ... });" to use it.
// NOTE: This should only be used for one-shot problems. If a destruction
//   pattern occurs multiple times then a specialized handle class is cleaner.
template <class Functor>
class ScopeGuard final
{
public:
  explicit ScopeGuard(Functor&& f) : m_functor(std::forward<Functor>(f)) {}
  ScopeGuard(ScopeGuard&& other) : m_functor(std::move(other.m_functor)), m_run(other.m_run)
  {
    other.m_run = false;
  }
  ~ScopeGuard() { Exit(); }
  ScopeGuard& operator=(ScopeGuard&& other)
  {
    m_functor = std::move(other.m_functor);
    m_run = other.m_run;
    other.m_run = false;
    return *this;
  }

  void Exit()
  {
    if (!m_run)
      return;

    Dismiss();
    m_functor();
  }

  void Dismiss() { m_run = false; }
private:
  Functor m_functor;
  bool m_run = true;
};

// Factory function to produce a ScopeGuard using automatic template
// argument deduction (important because Lambdas have anonymous types).
template <class Functor>
ScopeGuard<Functor> MakeScopeGuard(Functor&& f)
{
  return ScopeGuard<Functor>(std::forward<Functor>(f));
}

}  // Namespace Common
