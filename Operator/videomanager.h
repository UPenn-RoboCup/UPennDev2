/*!
 This class acts as a video feed services.
 It provides video feeds based on sensor identifiers.
 Feed access is provided via signal/slots.

 @author jdavis
 @date 02/21/2013

 TODO finish
 */

#ifndef VIDEOMANAGER_H
#define VIDEOMANAGER_H

#include "manager.h"
#include <map>

class QImage;
class VideoFeed;

class VideoManager : public Manager
{
public:
    static VideoManager* _instance;
    static VideoManager* Instance();

    VideoManager();

    void run();

public slots:
    void updateFeed(const std::string& identifier, const char* data, uint offset, uint len);

private:
    typedef std::map<std::string,VideoFeed*> VideoMap;
    VideoMap _videoFeeds;

};

#endif // VIDEOMANAGER_H
