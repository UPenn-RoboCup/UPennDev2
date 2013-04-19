/*!
 This class represents the data manager class.
 It encapsulates data loaders and manages video.

 initial class
 @author jdavis
 @date 02/21/2013
 */

#ifndef DATAMANAGER_H
#define DATAMANAGER_H

#include <QThread>

class DataManager : public QThread
{

public:
    static DataManager* _instance;
    static DataManager* Instance();

    DataManager();

protected:
    void run();

};

#endif // DATAMANAGER_H
