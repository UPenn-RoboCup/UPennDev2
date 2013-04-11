#include "datamanager.h"

DataManager* DataManager::_instance = 0;

DataManager* DataManager::Instance()
{
    if (_instance == 0)
    {
        _instance = new DataManager();
    }
    return _instance;
}


DataManager::DataManager()
{
}

void DataManager::run()
{

}
