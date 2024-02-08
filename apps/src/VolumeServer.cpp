#include <cstddef>
#include <cstdint>
#include <iostream>

#include <QCoreApplication>
#include <QSignalMapper>

#include "vc/app_support/GetMemorySize.hpp"
#include "vc/apps/server/VolumeServer.hpp"
#include "vc/core/neighborhood/CuboidGenerator.hpp"
#include "vc/core/util/Logging.hpp"

namespace vc = volcart;

std::string vc::VolumeServer::socketStr_(QTcpSocket* socket)
{
    return "[" + socket->peerAddress().toString().toStdString() + ":" +
           std::to_string(socket->peerPort()) + "]: ";
}

vc::VolumeServer::VolumeServer(
    VolumePkgMap volpkgs, quint16 port, std::size_t memory, QObject* parent)
    : QObject{parent}, volpkgs_{volpkgs}, memory_{memory}
{
    server_ = new QTcpServer(this);
    connect(
        server_, &QTcpServer::newConnection, this,
        &VolumeServer::acceptConnection);
    if (!server_->listen(QHostAddress::Any, port)) {
        vc::Logger()->info("Failed to start server.");
    } else {
        vc::Logger()->info("Listening on port: {}", port);
    }
}

void vc::VolumeServer::socketReadyRead(QDataStream* dataStream)
{
    QTcpSocket* socket = reinterpret_cast<QTcpSocket*>(dataStream->device());
    // Read all data from the socket
    dataStream->startTransaction();
    protocol::RequestHdr requestHdr;
    int bytesHdr = dataStream->readRawData(
        reinterpret_cast<char*>(&requestHdr), sizeof(requestHdr));
    if (bytesHdr != sizeof(protocol::RequestHdr)) {
        dataStream->rollbackTransaction();
        return;
    }
    if (requestHdr.magic != protocol::MAGIC) {
        vc::Logger()->error(
            "{}: magic value is incorrect: {}", socketStr_(socket),
            requestHdr.magic);
        // TODO: actually exit
    }
    if (requestHdr.version != protocol::V1) {
        vc::Logger()->error(
            "{}: version is unsupported: {}", socketStr_(socket),
            static_cast<std::uint32_t>(requestHdr.version));
        // TODO: actually exit
    }
    vc::Logger()->info(
        "{}: Need to resolve {} requests.", socketStr_(socket),
        requestHdr.numRequests);
    protocol::RequestArgs* requestArgs =
        new protocol::RequestArgs[requestHdr.numRequests];

    dataStream->startTransaction();
    int bytesArgs = dataStream->readRawData(
        reinterpret_cast<char*>(requestArgs),
        sizeof(protocol::RequestArgs) * requestHdr.numRequests);
    if (bytesArgs !=
        static_cast<int>(
            sizeof(protocol::RequestArgs) * requestHdr.numRequests)) {
        dataStream->rollbackTransaction();
    } else {
        dataStream->commitTransaction();
    }
    if (!dataStream->commitTransaction()) {
        delete[] requestArgs;
        return;
    }
    // Resolve requests
    for (std::uint32_t i = 0; i < requestHdr.numRequests; i++) {
        resolveRequest_(socket, &requestArgs[i]);
    }
    // Clean up
    vc::Logger()->info("{}: Closing connection...", socketStr_(socket));
    delete[] requestArgs;
    delete dataStream;
    socket->disconnectFromHost();
}

void vc::VolumeServer::acceptConnection()
{
    QTcpSocket* socket = server_->nextPendingConnection();
    connect(
        socket, &QAbstractSocket::disconnected, socket, &QObject::deleteLater);
    vc::Logger()->info("{}: Accepted connection...", socketStr_(socket));
    QDataStream* dataStream = new QDataStream(socket);
    connect(socket, &QTcpSocket::readyRead, [this, dataStream] {
        socketReadyRead(dataStream);
    });
}

void vc::VolumeServer::resolveRequest_(
    QTcpSocket* socket, protocol::RequestArgs* args)
{
    Volume::Pointer volume = nullptr;
    protocol::ResponseArgs responseArgs;
    std::memset(&responseArgs, 0, sizeof(protocol::ResponseArgs));
    std::strncpy(responseArgs.volpkg, args->volpkg, protocol::VOLPKG_SZ);
    std::strncpy(responseArgs.volume, args->volume, protocol::VOLUME_SZ);
    if (!volumes_.count(args->volume)) {
        vc::Logger()->info(
            "{}: Request for volume ({}, {}): need to load for the first "
            "time",
            socketStr_(socket), args->volpkg, args->volume);
        try {
            volume = volpkgs_.at(args->volpkg).volume(args->volume);
            volumes_.insert({args->volume, volume});
            // Update memory allocation distribution for all loaded volumes
            std::size_t memPerVolume = static_cast<std::size_t>(
                static_cast<double>(memory_) /
                static_cast<double>(volumes_.size()));
            vc::Logger()->info(
                "Reallocating memory per loaded volume to {} bytes.",
                memPerVolume);
            for (auto& pair : volumes_) {
                try {
                    pair.second->setCacheMemoryInBytes(memPerVolume);
                    if (pair.second->getCacheCapacity() < 1) {
                        throw std::runtime_error("Cache capacity is 0");
                    }
                } catch (const std::exception& e) {
                    vc::Logger()->error("{}", e.what());
                    // TODO: exit or something
                }
            }
        } catch (std::exception& e) {
            // TODO: solve this
            vc::Logger()->error("Unable to load volume: {}", e.what());
            socket->write(
                reinterpret_cast<char*>(&responseArgs),
                sizeof(protocol::ResponseArgs));
            return;
        }
    } else {
        volume = volumes_.at(args->volume);
        vc::Logger()->info(
            "{}: Request for volume ({}, {}): found in cache",
            socketStr_(socket), args->volpkg, args->volume);
    }
    // Generate subvolume for this request
    vc::CuboidGenerator subvolume;
    // This must be in x/y/z order.
    cv::Vec3d center{args->centerX, args->centerY, args->centerZ};
    cv::Vec3d xvec{args->basis0X, args->basis0Y, args->basis0Z};
    cv::Vec3d yvec{args->basis1X, args->basis1Y, args->basis0Z};
    cv::Vec3d zvec{args->basis2X, args->basis2Y, args->basis2Z};
    // This must be in z/y/x order.
    subvolume.setSamplingRadius(
        args->samplingRZ, args->samplingRY, args->samplingRX);
    subvolume.setSamplingInterval(args->samplingInterval);
    // This must be in z/y/x order.
    vc::Neighborhood neighborhood =
        subvolume.compute(volume, center, {zvec, yvec, xvec});
    vc::Logger()->info("{}: Subvolume generated...", socketStr_(socket));
    responseArgs.size =
        static_cast<std::uint32_t>(neighborhood.size() * sizeof(uint16_t));
    auto extents = neighborhood.extents();
    responseArgs.extentX = static_cast<std::uint32_t>(extents[2]);
    responseArgs.extentY = static_cast<std::uint32_t>(extents[1]);
    responseArgs.extentZ = static_cast<std::uint32_t>(extents[0]);
    socket->write(
        reinterpret_cast<char*>(&responseArgs), sizeof(protocol::ResponseArgs));
    socket->write(
        reinterpret_cast<char*>(neighborhood.data()),
        sizeof(std::uint16_t) * neighborhood.size());
    socket->flush();
}
