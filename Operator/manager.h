#ifndef MANAGER_H
#define MANAGER_H

#include <QThread>

class DataManager;
class VideoManager;
class CommsInterface;

class Manager : public QThread
{

public:
    Manager(const std::string& _identifier);

    static void InitManagers();

    static VideoManager* _videoManager;
    static DataManager* _dataManager;
    static CommsInterface* _commsInterface;

    std::string _identifier;

};


#endif // MANAGER_H
