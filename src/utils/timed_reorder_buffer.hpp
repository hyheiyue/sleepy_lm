#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <tuple>
#include <utility>

namespace sleepy::utils {

template <typename Sample> class TimedReorderBuffer {
public:
  using sample_type = Sample;
  using key_type = typename sample_type::key_type;

  explicit TimedReorderBuffer(double reorder_window_seconds = 0.0)
      : reorder_window_seconds_(std::max(0.0, reorder_window_seconds)) {}

  void set_window(double reorder_window_seconds) {
    reorder_window_seconds_ = std::max(0.0, reorder_window_seconds);
  }

  [[nodiscard]] double window() const { return reorder_window_seconds_; }

  [[nodiscard]] bool empty() const { return buffer_.empty(); }

  void clear() {
    buffer_.clear();
    max_event_time_seen_ = -std::numeric_limits<double>::infinity();
  }

  template <typename OnReady>
  void push(sample_type sample, OnReady &&on_ready) {
    const double timestamp = sample.timestamp;
    if (!std::isfinite(timestamp)) {
      return;
    }

    max_event_time_seen_ = std::max(max_event_time_seen_, timestamp);
    buffer_.emplace(make_key(sample, timestamp), std::move(sample));
    drain_ready(std::forward<OnReady>(on_ready));
  }

  template <typename OnReady> void flush(OnReady &&on_ready) {
    while (!buffer_.empty()) {
      emit_one(std::forward<OnReady>(on_ready));
    }
  }

private:
  struct EntryKey {
    double timestamp = 0.0;
    key_type order_key{};
  };

  struct EntryKeyLess {
    bool operator()(const EntryKey &lhs, const EntryKey &rhs) const {
      return std::tie(lhs.timestamp, lhs.order_key) <
             std::tie(rhs.timestamp, rhs.order_key);
    }
  };

  EntryKey make_key(const sample_type &sample, double timestamp) const {
    return EntryKey{
        timestamp,
        sample.order_key(),
    };
  }

  template <typename OnReady> void drain_ready(OnReady &&on_ready) {
    const double watermark = max_event_time_seen_ - reorder_window_seconds_;
    while (!buffer_.empty() && buffer_.begin()->first.timestamp <= watermark) {
      emit_one(std::forward<OnReady>(on_ready));
    }
  }

  template <typename OnReady> void emit_one(OnReady &&on_ready) {
    auto it = buffer_.begin();
    sample_type sample = std::move(it->second);
    buffer_.erase(it);
    on_ready(std::move(sample));
  }

  std::multimap<EntryKey, sample_type, EntryKeyLess> buffer_;
  double reorder_window_seconds_ = 0.0;
  double max_event_time_seen_ = -std::numeric_limits<double>::infinity();
};

} // namespace sleepy::utils
