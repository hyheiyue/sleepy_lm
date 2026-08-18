#pragma once

#include <Eigen/Core>
#include <ankerl/unordered_dense.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <list>
#include <utility>
#include <vector>

namespace sleepy {

inline __attribute__((always_inline)) std::uint64_t
hash_position_index(const Eigen::Matrix<std::uint16_t, 3, 1>& index) {
    return (static_cast<std::uint64_t>(index.x()) << 32)
        | (static_cast<std::uint64_t>(index.y()) << 16) | static_cast<std::uint64_t>(index.z());
}

class PointWithDistance {
public:
    Eigen::Vector3f point;
    float distance = 0.0F;

    __attribute__((always_inline)) PointWithDistance(Eigen::Vector3f point, float distance):
        point(std::move(point)),
        distance(distance) {}

    [[nodiscard]] __attribute__((always_inline)) bool
    operator()(const PointWithDistance& lhs, const PointWithDistance& rhs) const {
        return lhs.distance < rhs.distance;
    }

    [[nodiscard]] __attribute__((always_inline)) bool operator<(const PointWithDistance& rhs
    ) const {
        return distance < rhs.distance;
    }
};

class SmallIVox {
private:
    using GridCache = std::list<Eigen::Vector3f>;
    using GridMap = ankerl::unordered_dense::map<std::uint64_t, GridCache::iterator>;

public:
    __attribute__((always_inline)) explicit SmallIVox(float resolution, std::size_t capacity):
        inv_resolution_(1.0F / resolution),
        capacity_(capacity) {}

    __attribute__((always_inline)) bool add_point(const Eigen::Vector3f& point) {
        const std::uint64_t hash_key = hash_position_index(get_position_index(point));
        const auto iter = grids_map_.find(hash_key);
        if (iter != grids_map_.end()) {
            grids_cache_.splice(grids_cache_.begin(), grids_cache_, iter->second);
            return false;
        }

        grids_cache_.push_front(point);
        grids_map_.emplace(hash_key, grids_cache_.begin());
        if (grids_map_.size() >= capacity_) {
            grids_map_.erase(hash_position_index(get_position_index(grids_cache_.back())));
            grids_cache_.pop_back();
        }
        return true;
    }

    __attribute__((always_inline)) void get_closest_point(
        const Eigen::Vector3f& point,
        std::vector<Eigen::Vector3f>& closest_points,
        std::size_t max_num = 5
    ) {
        closest_points.clear();
        const std::uint64_t hash_key = hash_position_index(get_position_index(point));
        append_if_present(hash_key, closest_points);

        for (unsigned shift = 0; shift < 48; shift += 16) {
            const std::uint64_t mask = std::uint64_t { 0xFFFF } << shift;
            const std::uint64_t stride = std::uint64_t { 1 } << shift;
            append_if_present((hash_key & ~mask) | ((hash_key + stride) & mask), closest_points);
            append_if_present((hash_key & ~mask) | ((hash_key - stride) & mask), closest_points);
        }

        if (closest_points.size() > max_num) [[likely]] {
            candidates_.clear();
            for (const auto& candidate: closest_points) {
                candidates_.emplace_back(candidate, (candidate - point).squaredNorm());
            }
            std::nth_element(
                candidates_.begin(),
                candidates_.begin() + static_cast<std::ptrdiff_t>(max_num) - 1,
                candidates_.end()
            );
            closest_points.clear();
            for (std::size_t i = 0; i < max_num; ++i) {
                closest_points.push_back(candidates_[i].point);
            }
        }
    }

    [[nodiscard]] __attribute__((always_inline)) Eigen::Matrix<std::uint16_t, 3, 1>
    get_position_index(const Eigen::Vector3f& point) const {
        return (point * inv_resolution_).array().floor().cast<std::uint16_t>();
    }

private:
    __attribute__((always_inline)) void
    append_if_present(std::uint64_t hash_key, std::vector<Eigen::Vector3f>& points) {
        const auto iter = grids_map_.find(hash_key);
        if (iter != grids_map_.end()) {
            points.push_back(*iter->second);
        }
    }

    GridMap grids_map_;
    float inv_resolution_;
    std::size_t capacity_;
    GridCache grids_cache_;
    std::vector<PointWithDistance> candidates_;
};

} // namespace sleepy
