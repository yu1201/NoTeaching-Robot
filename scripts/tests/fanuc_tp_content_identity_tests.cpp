#include "FanucTpContentIdentity.h"

#include <QCoreApplication>
#include <QFile>
#include <QTemporaryDir>

#include <cstdlib>
#include <iostream>

namespace
{
void Check(bool condition, const char* message)
{
    if (!condition)
    {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(1);
    }
}

void WriteExact(const QString& path, const QByteArray& payload)
{
    QFile file(path);
    Check(file.open(QIODevice::WriteOnly | QIODevice::Truncate), "cannot create TP fixture");
    Check(file.write(payload) == payload.size(), "cannot write TP fixture");
    file.close();
}
}

int main(int argc, char** argv)
{
    QCoreApplication app(argc, argv);
    QTemporaryDir temp;
    Check(temp.isValid(), "temporary directory unavailable");

    const QString trustedPath = temp.filePath(QStringLiteral("trusted.tp"));
    const QString remotePath = temp.filePath(QStringLiteral("remote.tp"));
    const QByteArray trusted("FANUC-TP-BINARY-A-0123456789");
    QByteArray sameSizeReplacement("FANUC-TP-BINARY-B-0123456789");
    Check(trusted.size() == sameSizeReplacement.size(), "fixtures are not the same length");
    WriteExact(trustedPath, trusted);
    WriteExact(remotePath, trusted);

    FanucTpContentIdentity::Identity trustedIdentity;
    FanucTpContentIdentity::Identity remoteIdentity;
    Check(FanucTpContentIdentity::Read(trustedPath.toStdWString(), trustedIdentity),
        "trusted identity read failed");
    Check(FanucTpContentIdentity::Read(remotePath.toStdWString(), remoteIdentity),
        "remote identity read failed");
    Check(FanucTpContentIdentity::Matches(trustedIdentity, remoteIdentity),
        "equal TP bytes did not match");

    WriteExact(remotePath, sameSizeReplacement);
    Check(FanucTpContentIdentity::Read(remotePath.toStdWString(), remoteIdentity),
        "replacement identity read failed");
    Check(remoteIdentity.size == trustedIdentity.size, "replacement size changed unexpectedly");
    Check(!FanucTpContentIdentity::Matches(trustedIdentity, remoteIdentity),
        "same-size different TP bytes bypassed SHA-256 identity");

    std::cout << "PASS: fixed FANUC TP identity rejects same-size byte replacement\n";
    return 0;
}
