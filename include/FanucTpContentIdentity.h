#pragma once

#include <array>
#include <cstdint>
#include <filesystem>

namespace FanucTpContentIdentity
{
    struct Identity
    {
        std::array<std::uint8_t, 32> sha256{};
        std::uintmax_t size = 0;
    };

    // 固定 TP 是可执行机器人程序，内容身份必须使用加密哈希；文件名和长度都不能作为执行证明。
    bool Read(const std::filesystem::path& path, Identity& identity);
    bool Matches(const Identity& left, const Identity& right);
}
