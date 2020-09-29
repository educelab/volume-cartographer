#pragma once

#include <QObject>
#include <QTcpSocket>

namespace volcart
{

/** Class implementing a sample client that uses the VolumeProtocol. */
class VolumeClient : public QObject
{
    Q_OBJECT

public:
    /** Construct a new VolumeClient object. */
    explicit VolumeClient(
        const QString& ip, quint16 port, QObject* parent = nullptr);

private slots:
    /** Called when a new connection has been established. */
    void newConnection();
    /** Called when an existing connection has an error. */
    void connectionError(QAbstractSocket::SocketError socketError);

signals:
    /** Called when it is time to exit the application. */
    void finished();

private:
    /** Store a pointer to the client connection socket. */
    QTcpSocket* client_;
};

}  // namespace volcart
