#pragma once

#include <atomic>
#include <cstdint>
#include <cmath>
#include <vector>

// Caller holds the per-driver refresh mutex for this guard's lifetime.
class InovanceKinematicsReadScope
{
public:
    explicit InovanceKinematicsReadScope(std::atomic_bool& value) : m_flag(value)
    { m_flag.store(true); }
    ~InovanceKinematicsReadScope() { m_flag.store(false); }
private:
    std::atomic_bool& m_flag;
};

// A saved model is evidence, not permission. Every connection/refresh gets a
// new generation; only a live validated result for that generation is ready.
class InovanceKinematicsSession
{
public:
    std::uint64_t Invalidate() { return m_generation.fetch_add(1) + 1; }
    std::uint64_t Generation() const { return m_generation.load(); }
    bool Publish(std::uint64_t generation)
    {
        if (generation != m_generation.load()) { return false; }
        m_validated.store(generation);
        return Ready();
    }
    bool Ready() const { return m_validated.load() == m_generation.load(); }
    static bool StationarySample(const std::vector<double>& first,
        const std::vector<double>& last, double tolerance = 0.002)
    {
        if (first.size() < 6 || first.size() != last.size()) { return false; }
        for (std::size_t axis = 0; axis < first.size(); ++axis)
        {
            if (!std::isfinite(first[axis]) || !std::isfinite(last[axis])
                || std::abs(first[axis] - last[axis]) > tolerance) { return false; }
        }
        return true;
    }
private:
    std::atomic<std::uint64_t> m_generation{1};
    std::atomic<std::uint64_t> m_validated{0};
};
