#include "FanucTpContentIdentity.h"

#include <QByteArrayView>
#include <QCryptographicHash>

#include <algorithm>
#include <fstream>
#include <limits>

bool FanucTpContentIdentity::Read(const std::filesystem::path& path, Identity& identity)
{
    identity = Identity{};
    std::ifstream input(path, std::ios::in | std::ios::binary);
    if (!input.is_open())
    {
        return false;
    }

    QCryptographicHash hash(QCryptographicHash::Sha256);
    char buffer[16 * 1024] = {};
    while (input.good())
    {
        input.read(buffer, sizeof(buffer));
        const std::streamsize count = input.gcount();
        if (count > 0)
        {
            if (count > (std::numeric_limits<int>::max)())
            {
                return false;
            }
            hash.addData(QByteArrayView(buffer, static_cast<qsizetype>(count)));
            identity.size += static_cast<std::uintmax_t>(count);
        }
    }
    if (!input.eof() || identity.size == 0)
    {
        identity = Identity{};
        return false;
    }

    const QByteArray digest = hash.result();
    if (digest.size() != static_cast<int>(identity.sha256.size()))
    {
        identity = Identity{};
        return false;
    }
    std::copy(digest.cbegin(), digest.cend(), identity.sha256.begin());
    return true;
}

bool FanucTpContentIdentity::Matches(const Identity& left, const Identity& right)
{
    return left.size > 0
        && left.size == right.size
        && left.sha256 == right.sha256;
}
