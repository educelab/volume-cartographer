#include <cstdint>
#include <cstring>
#include <iostream>

#include <QDataStream>
#include <QTcpServer>

#include "vc/app_support/GetMemorySize.hpp"
#include "vc/apps/server/VolumeClient.hpp"
#include "vc/apps/server/VolumeProtocol.hpp"
#include "vc/core/neighborhood/CuboidGenerator.hpp"
#include "vc/core/types/Volume.hpp"
#include "vc/core/types/VolumePkg.hpp"
#include "vc/core/util/Logging.hpp"

namespace vc = volcart;

vc::VolumeClient::VolumeClient(const QString& ip, quint16 port, QObject* parent)
    : QObject{parent}
{
    client_ = new QTcpSocket(this);
    connect(
        client_, &QAbstractSocket::disconnected, client_,
        &QObject::deleteLater);
    connect(
        client_, &QAbstractSocket::connected, this,
        &VolumeClient::newConnection);
    connect(
        client_, &QAbstractSocket::errorOccurred, this,
        &VolumeClient::connectionError);
    client_->connectToHost(ip, port);
}

void vc::VolumeClient::newConnection()
{
    // CarbonSquares
    // 20180509123106
    // 20180509123119
    vc::Logger()->info("Connection established.");
    protocol::RequestHdr requestHdr;
    requestHdr.numRequests = 2;
    client_->write(
        reinterpret_cast<char*>(&requestHdr), sizeof(protocol::RequestHdr));
    client_->flush();
    for (std::uint32_t i = 0; i < requestHdr.numRequests; i++) {
        // Neighborhood should be 27 with these settings
        protocol::RequestArgs requestArgs;
        std::memset(&requestArgs, 0, sizeof(requestArgs));
        std::strncpy(requestArgs.volpkg, "CarbonSquares", protocol::VOLPKG_SZ);
        std::strncpy(requestArgs.volume, "20180509123106", protocol::VOLUME_SZ);
        requestArgs.centerX = 100.0f;
        requestArgs.centerY = 50.0f;
        requestArgs.centerZ = 100.0f;
        requestArgs.basis0X = 1.0f;
        requestArgs.basis1Y = 1.0f;
        requestArgs.basis2Z = 1.0f;
        requestArgs.samplingRX = 40.0f;
        requestArgs.samplingRY = 20.0f;
        requestArgs.samplingRZ = 40.0f;
        requestArgs.samplingInterval = 1.0f / (i + 1);
        client_->write(
            reinterpret_cast<char*>(&requestArgs),
            sizeof(protocol::RequestArgs));
        client_->flush();
    }
    // Read response from server
    auto* responseArgs = new protocol::ResponseArgs[requestHdr.numRequests];
    QDataStream* dataStream = new QDataStream(client_);
    while (client_->waitForReadyRead()) {
        dataStream->startTransaction();
        bool abort = false;
        for (std::uint32_t i = 0; i < requestHdr.numRequests; i++) {
            int bytesArgs = dataStream->readRawData(
                reinterpret_cast<char*>(&responseArgs[i]),
                sizeof(protocol::ResponseArgs));
            if (bytesArgs != sizeof(protocol::ResponseArgs)) {
                dataStream->rollbackTransaction();
                abort = true;
                break;
            }
            vc::Logger()->info("Skipping {} bytes.", responseArgs[i].size);
            int bytesSkipped = dataStream->skipRawData(responseArgs[i].size);
            if (bytesSkipped != responseArgs[i].size) {
                dataStream->rollbackTransaction();
                abort = true;
                break;
            }
        }
        if (!abort && dataStream->commitTransaction()) {
            break;
        }
    }
    for (std::uint32_t i = 0; i < requestHdr.numRequests; i++) {
        vc::Logger()->info("=== Response: #{} ===", i);
        vc::Logger()->info("Volume Package: {}", responseArgs[i].volpkg);
        vc::Logger()->info("Volume: {}", responseArgs[i].volume);
    }
    delete dataStream;
    delete[] responseArgs;
    client_->disconnectFromHost();
    emit finished();
}

void vc::VolumeClient::connectionError(QAbstractSocket::SocketError socketError)
{
    vc::Logger()->error("{}", client_->errorString().toStdString());
    emit finished();
}
