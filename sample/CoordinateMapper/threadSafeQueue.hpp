#pragma once

#include <condition_variable>
#include <deque>
#include <mutex>

namespace sim {

/**
 * \brief A fixed size thread safe queue
 */
template <typename T>
class ThreadSafeQ {
 public:
  explicit ThreadSafeQ(size_t maxSize = size_t(-1))
    : mMaxSize{maxSize} {}

  ThreadSafeQ(const ThreadSafeQ<T>&) = delete;

  virtual ~ThreadSafeQ() {
    clear();
  }

  const T& front() {
    std::scoped_lock lock(mMtx);
    return mQueue.front();
  }

  const T& back() {
    std::scoped_lock lock(mMtx);
    return mQueue.back();
  }

  /**
   * Multiple thread safe
   */
  bool tryPopFront(T& v) {
    std::scoped_lock lock(mMtx);
    if (!mQueue.empty()) {
      v = std::move(mQueue.front());
      mQueue.pop_front();
      return true;
    } else {
      return false;
    }
  }

  /**
   * Multiple thread safe
   */
  bool tryPopBack(T& v) {
    std::scoped_lock lock(mMtx);
    if (!mQueue.empty()) {
      v = std::move(mQueue.back());
      mQueue.pop_front();
      return true;
    } else {
      return false;
    }
  }

  T popFront() {
    std::scoped_lock lock(mMtx);

    T v = std::move(mQueue.front());
    mQueue.pop_front();

    return v;
  }

  T popBack() {
    std::scoped_lock lock(mMtx);

    T v = std::move(mQueue.back());
    mQueue.pop_back();

    return v;
  }

  void pushBack(const T& v) {
    std::scoped_lock lock(mMtx);

    if (mQueue.size() > mMaxSize) mQueue.pop_front();

    mQueue.emplace_back(std::move(v));

    std::unique_lock<std::mutex> ul(mMtxBlk);
    mCV.notify_one();
  }

  void pushFront(const T& v) {
    std::scoped_lock lock(mMtx);

    if (mQueue.size() > mMaxSize) mQueue.pop_back();

    mQueue.emplace_front(std::move(v));

    std::unique_lock<std::mutex> ul(mMtxBlk);
    mCV.notify_one();
  }

  bool empty() {
    std::scoped_lock lk(mMtx);
    return mQueue.empty();
  }

  size_t count() {
    std::scoped_lock lk(mMtx);
    return mQueue.size();
  }

  void clear() {
    std::scoped_lock lk(mMtx);
    mQueue.clear();
  }

  void wait() {
    while (empty()) {
      std::unique_lock<std::mutex> ul(mMtxBlk);
      mCV.wait(ul);
    }
  }

 private:
  /* data */
  std::deque<T> mQueue;

  std::mutex mMtx;     // queue
  std::mutex mMtxBlk;  // blocking

  std::condition_variable mCV;

  size_t mMaxSize;
};
}  // namespace sim