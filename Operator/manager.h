#ifndef MANAGER_H
#define MANAGER_H

#include <QObject>

class DataManager;
class VideoManager;

class Manager : public QObject
{

public:
    Manager();

    void InitManagers();

};


#endif // MANAGER_H
