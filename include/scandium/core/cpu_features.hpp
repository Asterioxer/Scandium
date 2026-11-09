#pragma once

#include <cstdint>
#include <string>

#if defined(_MSC_VER)
    #include <intrin.h>
#elif defined(__GNUC__) || defined(__clang__)
    #if defined(__x86_64__) || defined(__i386__)
        #include <cpuid.h>
    #endif
#endif

namespace scandium::core {

struct CPUInfo {
    bool sse{false};
    bool sse2{false};
    bool sse3{false};
    bool ssse3{false};
    bool sse4_1{false};
    bool sse4_2{false};
    bool avx{false};
    bool avx2{false};
    bool fma{false};
    bool avx512f{false};
    bool avx512vl{false};
    bool avx512bw{false};
    bool avx512dq{false};
    bool neon{false};
    std::string vendor;
    std::string brand;
};

class CPUFeatures {
public:
    [[nodiscard]] static const CPUInfo& get() noexcept {
        static const CPUInfo info = detect();
        return info;
    }

    [[nodiscard]] static bool has_sse() noexcept { return get().sse; }
    [[nodiscard]] static bool has_sse2() noexcept { return get().sse2; }
    [[nodiscard]] static bool has_sse3() noexcept { return get().sse3; }
    [[nodiscard]] static bool has_ssse3() noexcept { return get().ssse3; }
    [[nodiscard]] static bool has_sse4_1() noexcept { return get().sse4_1; }
    [[nodiscard]] static bool has_sse4_2() noexcept { return get().sse4_2; }
    [[nodiscard]] static bool has_avx() noexcept { return get().avx; }
    [[nodiscard]] static bool has_avx2() noexcept { return get().avx2; }
    [[nodiscard]] static bool has_fma() noexcept { return get().fma; }
    [[nodiscard]] static bool has_avx512f() noexcept { return get().avx512f; }
    [[nodiscard]] static bool has_neon() noexcept { return get().neon; }

private:
    static CPUInfo detect() noexcept {
        CPUInfo info{};

#if defined(_M_AMD64) || defined(_M_X64) || defined(__x86_64__) || defined(__i386__) || defined(_M_IX86)
        auto run_cpuid = [](int leaf, int subleaf, int regs[4]) noexcept {
#if defined(_MSC_VER)
            __cpuidex(regs, leaf, subleaf);
#elif defined(__GNUC__) || defined(__clang__)
            __cpuid_count(leaf, subleaf, regs[0], regs[1], regs[2], regs[3]);
#else
            regs[0] = regs[1] = regs[2] = regs[3] = 0;
#endif
        };

        int cpu_info[4] = {0, 0, 0, 0};
        run_cpuid(0, 0, cpu_info);
        const int max_leaf = cpu_info[0];

        // Vendor string
        char vendor[13] = {0};
        *reinterpret_cast<int*>(vendor + 0) = cpu_info[1]; // EBX
        *reinterpret_cast<int*>(vendor + 4) = cpu_info[3]; // EDX
        *reinterpret_cast<int*>(vendor + 8) = cpu_info[2]; // ECX
        info.vendor = vendor;

        if (max_leaf >= 1) {
            run_cpuid(1, 0, cpu_info);
            const int ecx = cpu_info[2];
            const int edx = cpu_info[3];

            info.sse = (edx & (1 << 25)) != 0;
            info.sse2 = (edx & (1 << 26)) != 0;
            info.sse3 = (ecx & (1 << 0)) != 0;
            info.ssse3 = (ecx & (1 << 9)) != 0;
            info.sse4_1 = (ecx & (1 << 19)) != 0;
            info.sse4_2 = (ecx & (1 << 20)) != 0;
            info.fma = (ecx & (1 << 12)) != 0;
            info.avx = (ecx & (1 << 28)) != 0;
        }

        if (max_leaf >= 7) {
            run_cpuid(7, 0, cpu_info);
            const int ebx = cpu_info[1];

            info.avx2 = (ebx & (1 << 5)) != 0;
            info.avx512f = (ebx & (1 << 16)) != 0;
            info.avx512dq = (ebx & (1 << 17)) != 0;
            info.avx512bw = (ebx & (1 << 30)) != 0;
            info.avx512vl = (cpu_info[2] & (1 << 31)) != 0;
        }

        // Extended leaf for brand string
        run_cpuid(0x80000000, 0, cpu_info);
        const unsigned int max_ext_leaf = static_cast<unsigned int>(cpu_info[0]);
        if (max_ext_leaf >= 0x80000004) {
            char brand[49] = {0};
            for (unsigned int i = 0; i < 3; ++i) {
                run_cpuid(static_cast<int>(0x80000002 + i), 0, cpu_info);
                *reinterpret_cast<int*>(brand + i * 16 + 0) = cpu_info[0];
                *reinterpret_cast<int*>(brand + i * 16 + 4) = cpu_info[1];
                *reinterpret_cast<int*>(brand + i * 16 + 8) = cpu_info[2];
                *reinterpret_cast<int*>(brand + i * 16 + 12) = cpu_info[3];
            }
            info.brand = brand;
        }
#elif defined(__ARM_NEON) || defined(__arm64__) || defined(_M_ARM64)
        info.neon = true;
        info.vendor = "ARM";
        info.brand = "ARM Processor with NEON";
#endif

        return info;
    }
};

} // namespace scandium::core
