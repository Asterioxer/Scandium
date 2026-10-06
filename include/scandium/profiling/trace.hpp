#pragma once

#include <chrono>
#include <cstdint>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

namespace scandium::profiling {

struct TraceEvent {
    std::string name;
    std::uint64_t timestamp_microseconds{};
    std::uint64_t duration_microseconds{};
    std::uint32_t thread_id{};
};

class TraceCollector {
public:
    using clock = std::chrono::steady_clock;

    void record(std::string_view name, clock::time_point start,
                clock::time_point end, std::uint32_t thread_id = 0) {
        events_.push_back({
            std::string(name),
            static_cast<std::uint64_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(
                    start.time_since_epoch()).count()),
            static_cast<std::uint64_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(
                    end - start).count()),
            thread_id
        });
    }

    [[nodiscard]] const std::vector<TraceEvent>& events() const noexcept {
        return events_;
    }

    [[nodiscard]] std::string to_chrome_json() const {
        std::ostringstream out;
        out << "{"traceEvents":[";
        for (std::size_t i = 0; i < events_.size(); ++i) {
            if (i != 0) out << ',';
            const auto& event = events_[i];
            out << "{"name":"";
            for (const char character : event.name) {
                if (character == '"' || character == '\') out << '\';
                out << character;
            }
            out << "","cat":"scandium","ph":"X","ts":"
                << event.timestamp_microseconds
                << ","dur":" << event.duration_microseconds
                << ","pid":1,"tid":" << event.thread_id << '}';
        }
        out << "]}";
        return out.str();
    }

    void clear() { events_.clear(); }

private:
    std::vector<TraceEvent> events_;
};

class TraceScope {
public:
    TraceScope(TraceCollector& collector, std::string_view name,
               std::uint32_t thread_id = 0)
        : collector_(collector), name_(name), thread_id_(thread_id),
          start_(TraceCollector::clock::now()) {}

    ~TraceScope() {
        collector_.record(name_, start_, TraceCollector::clock::now(), thread_id_);
    }

    TraceScope(const TraceScope&) = delete;
    TraceScope& operator=(const TraceScope&) = delete;

private:
    TraceCollector& collector_;
    std::string_view name_;
    std::uint32_t thread_id_{};
    TraceCollector::clock::time_point start_;
};

} // namespace scandium::profiling
