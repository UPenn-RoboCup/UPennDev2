/*!
    This class encapsulates the comms interface.

    initial class
    @author jdavis45
    @date 02/21/2013

 */
#ifndef COMMSINTERFACE_H
#define COMMSINTERFACE_H

#include <QObject>
#include <QtNetwork/QUdpSocket>

#include "manager.h"

class CommsInterface : public QObject
{
public:
    CommsInterface();

    void Initialize();

private slots:
    void readPendingDatagrams();

private:
    QUdpSocket* _udpSocket;

};

#endif // COMMSINTERFACE_H
