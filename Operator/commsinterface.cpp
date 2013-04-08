#include "commsinterface.h"

#include <QtNetwork/QUdpSocket>

#define CM_INCOMING_PORT 7755

CommsInterface::CommsInterface()
    : QObject()
{
}

void CommsInterface::Initialize()
{
    _udpSocket = new QUdpSocket(this);
    _udpSocket->bind(QHostAddress::LocalHost, CM_INCOMING_PORT);

    connect(_udpSocket, SIGNAL(readyRead()),
                 this, SLOT(readPendingDatagrams()));
}

void CommsInterface::readPendingDatagrams()
{
    while (_udpSocket->waitForReadyRead(-1)) {
            while(_udpSocket->hasPendingDatagrams()) {
                QByteArray datagram;
                datagram.resize(_udpSocket->pendingDatagramSize());
                QHostAddress sender;
                quint16 senderPort;

                _udpSocket->readDatagram(datagram.data(), datagram.size(),
                                        &sender, &senderPort);
                //cout << "datagram received from " << sender.toString() << endl;
            }
        }
}
