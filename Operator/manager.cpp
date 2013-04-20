#include "manager.h"
#include "videomanager.h"
#include "datamanager.h"
#include "commsinterface.h"

VideoManager* Manager::_videoManager = new VideoManager();
DataManager* Manager::_dataManager = new DataManager();
CommsInterface* Manager::_commsInterface = new CommsInterface();


Manager::Manager(const std::string& identifier)
    : QThread(), _identifier(identifier)
{

}

void Manager::InitManagers()
{

}

