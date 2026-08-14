#pragma once
#include <algorithm>
#include <chrono>
#include <string>
namespace sleepy::utils {
inline std::string to_upper(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return std::toupper(c); });
  return s;
}
template <typename Func>
void dt_once(Func &&func, std::chrono::duration<double> dt) noexcept {
  // 每个模板实例共享一个节流时间戳，适合低频日志和调试发布。
  static auto last_call = std::chrono::steady_clock::now();

  auto now = std::chrono::steady_clock::now();
  if (now - last_call >= dt) {
    last_call = now;
    func();
  }
}
} // namespace sleepy::utils