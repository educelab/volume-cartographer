#pragma once

#include <QDataStream>
#include <QObject>
#include <QTcpServer>
#include <QTcpSocket>
#include <cstddef>
#include <map>

#include "vc/apps/server/VolumeProtocol.hpp"
#include "vc/core/types/Volume.hpp"
#include "vc/core/types/VolumePkg.hpp"

namespace volcart
{

/** Class for implementing the VolumeServer. */
class VolumeServer : public QObject
{
    Q_OBJECT

public:
    /** Convenience type for a map of strings to VolumePkgs. */
    using VolumePkgMap = std::unordered_map<std::string, VolumePkg>;

    /** Convenience type for a map of strings to Volume pointers. */
    using VolumeMap = std::unordered_map<std::string, Volume::Pointer>;

    /** Construct a new VolumeServer object. */
    explicit VolumeServer(
        VolumePkgMap volpkgs,
        quint16 port,
        std::size_t memory,
        QObject* parent = nullptr);

private slots:
    /** Called when a new client connection has been established. */
    void acceptConnection();

    /** Called when a socket is ready to be read from. */
    void socketReadyRead(QDataStream* dataStream);

signals:
    /** Called when it's time to exit the application. */
    void finished();

private:
    /** A pointer to the TCP server object. */
    QTcpServer* server_;

    /** A map of loaded volpkgs identified by string key. */
    VolumePkgMap volpkgs_;

    /** A map of cached/loaded volumes identified by string key. */
    VolumeMap volumes_;

    /** How much memory the server should use for caching volumes. */
    std::size_t memory_;

    /** Generate a string for representing a socket. */
    std::string socketStr_(QTcpSocket* socket);

    /** Resolve a single sub-volume request. */
    void resolveRequest_(QTcpSocket* socket, protocol::RequestArgs* args);
};

}  // namespace volcart
