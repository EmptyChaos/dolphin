// Copyright 2010 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#pragma once

// a simple lockless thread-safe,
// single reader, single writer queue

#include <atomic>
#include <cstddef>
#include <utility>

#include "Common/CommonTypes.h"

namespace Common
{
namespace Details
{
template <bool Enabled>
class FifoQueueSizeStrategy
{
protected:
  void IncrementSize() {}
  void DecrementSize() {}
  void ResetSize() {}
};

template <>
class FifoQueueSizeStrategy<true>
{
public:
  u32 Size() const { return m_size.load(std::memory_order_acquire); }
protected:
  void IncrementSize() { m_size.fetch_add(1, std::memory_order_release); }
  void DecrementSize() { m_size.fetch_sub(1, std::memory_order_release); }
  void ResetSize() { m_size.store(0, std::memory_order_relaxed); }
private:
  std::atomic<u32> m_size{0};
};
}

template <typename T, bool NeedSize = true>
class FifoQueue : public Details::FifoQueueSizeStrategy<NeedSize>
{
public:
  FifoQueue() { m_write_ptr = m_read_ptr = new Element(); }
  FifoQueue(const FifoQueue&) = delete;
  ~FifoQueue()
  {
    Clear();
    delete m_read_ptr;
  }
  FifoQueue& operator=(const FifoQueue&) = delete;

  bool Empty() const { return !m_read_ptr->next.load(std::memory_order_acquire); }
  T& Front() const { return m_read_ptr->current; }
  void Push(const T& t) { Push(T(t)); }
  void Push(T&& t)
  {
    // create the element, add it to the queue
    m_write_ptr->current = std::move(t);
    // set the next pointer to a new element ptr
    // then advance the write pointer
    Element* new_ptr = new Element();
    m_write_ptr->next.store(new_ptr, std::memory_order_release);
    m_write_ptr = new_ptr;
    IncrementSize();
  }

  void Pop()
  {
    DecrementSize();
    Element* tmpptr = m_read_ptr;
    // advance the read pointer
    m_read_ptr = tmpptr->next.load(std::memory_order_acquire);
    delete tmpptr;  // this also deletes the element
  }

  bool Pop(T& t)
  {
    if (Empty())
      return false;

    t = std::move(m_read_ptr->current);
    Pop();
    return true;
  }

  // consumer side only
  void Clear()
  {
    while (true)
    {
      Element* next = m_read_ptr->next.load(std::memory_order_acquire);
      if (!next)
        break;
      delete m_read_ptr;
      m_read_ptr = next;
      DecrementSize();
    }
  }

private:
  // stores a pointer to element
  // and a pointer to the next Element
  struct Element
  {
    std::atomic<Element*> next{nullptr};
    T current;
  };

  Element* m_write_ptr;
  Element* m_read_ptr;
};
}
