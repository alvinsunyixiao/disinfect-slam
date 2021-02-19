#pragma once

#include <chrono>
#include <cinttypes>
#include <deque>
#include <functional>
#include <queue>

static const auto start = std::chrono::steady_clock::now();

// get timestamp from steady clock
// this is preferred
template <typename UNIT>
inline uint64_t GetTimestamp() {
  const auto now = std::chrono::steady_clock::now();
  return std::chrono::duration_cast<UNIT>(now - start).count();
}

// get timestamp from unsteady system clock
// have to have this function because librealsense is using system clock...
template <typename UNIT>
inline uint64_t GetSystemTimestamp() {
  const auto tp = std::chrono::system_clock::now().time_since_epoch();
  return std::chrono::duration_cast<UNIT>(tp).count();
}

template <typename UNIT>
class LocalClock {
 public:
  LocalClock(uint64_t local_tick_now) : offset_(GetTimestamp<UNIT>() - local_tick_now) {}

  uint64_t convert_timestamp(uint64_t local_tick) const { return local_tick + offset_; }

 private:
  const uint64_t offset_;
};

template <typename SyncType, typename AsyncType, typename TimeType = uint64_t>
class TimeSyncer {
 public:
  using CallbackFunc = std::function<void(const SyncType&, const AsyncType&)>;

  TimeSyncer(const CallbackFunc& callback) : callback_(callback) {}

  void AddMeasurement1(const SyncType& meas) {
    sync_q_.push(meas);
    TryInvokeSync();
  }

  void AddMeasurement2(const AsyncType& meas) {
    async_q_.push_back(meas);
    TryInvokeSync();
  }

 private:
  void TryInvokeSync() const {
    if (sync_q_.empty() || async_q_.empty()) { return; }

    if (async_q_.front().timestamp < sync_q_.back().timestamp &&
        async_q_.back().timestamp >= sync_q_.front().timestamp) {
      // remove invalid data samples
      while (sync_q_.front().timestamp < async_q_.front().timestamp) {
        sync_q_.pop();
      }

      const SyncType& sync_data = sync_q_.front();

      auto it_curr = async_q_.begin();
      auto it_next = std::next(it_curr, 1);

      // find zero crossing
      while (it_next != async_q_.end() && it_next->timestamp < sync_data.timestamp) {
        it_curr = (it_next++);
      }

      // synchronize and invoke callback
      if (it_next != async_q_.end()) {
        // TODO(alvin): consider implementing interpolation here
        if (sync_data.timestamp - it_curr->timestamp < it_next->timestamp - sync_data.timestamp) {
          callback_(sync_data, *it_curr);
        } else {
          callback_(sync_data, *it_next);
        }

        // clean up history
        while (!async_q_.empty() && async_q_.front().timestamp <= sync_data.timestamp) {
          async_q_.pop_front();
        }
        while (!sync_q_.empty() && sync_q_.front().timestamp <= sync_data.timestamp) {
          sync_q_.pop();
        }
      }
    }
  }

  CallbackFunc callback_;
  mutable std::queue<SyncType> sync_q_;
  mutable std::deque<AsyncType> async_q_;
};
