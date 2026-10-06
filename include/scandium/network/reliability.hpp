#pragma once

#include <cstdint>

namespace scandium::network {

struct PacketHeader {
    std::uint16_t protocol{1};
    std::uint16_t flags{};
    std::uint32_t sequence{};
    std::uint32_t latest_ack{};
    std::uint32_t ack_bits{};
};

class AckWindow {
public:
    void observe(std::uint32_t sequence) noexcept {
        if (!initialized_) {
            latest_ = sequence;
            initialized_ = true;
            return;
        }

        const std::int32_t delta = static_cast<std::int32_t>(sequence - latest_);
        if (delta > 0) {
            if (delta >= 32) {
                bits_ = 0;
            } else {
                bits_ <<= delta;
                bits_ |= (1u << (delta - 1));
            }
            latest_ = sequence;
        } else if (delta < 0) {
            const std::int32_t age = -delta;
            if (age <= 32) {
                bits_ |= (1u << (age - 1));
            }
        }
    }

    [[nodiscard]] std::uint32_t latest() const noexcept { return latest_; }
    [[nodiscard]] std::uint32_t ack_bits() const noexcept { return bits_; }

    [[nodiscard]] bool has_seen(std::uint32_t sequence) const noexcept {
        if (!initialized_) return false;
        if (sequence == latest_) return true;

        const std::int32_t delta = static_cast<std::int32_t>(latest_ - sequence);
        if (delta <= 0 || delta > 32) return false;
        return (bits_ & (1u << (delta - 1))) != 0;
    }

private:
    std::uint32_t latest_{};
    std::uint32_t bits_{};
    bool initialized_{};
};

} // namespace scandium::network
