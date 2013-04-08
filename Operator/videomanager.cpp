#include "videomanager.h"

#include <QImage>

#include "videofeed.h"

VideoManager::VideoManager()
    : Manager("VideoManager")
{
}

void VideoManager::run()
{

}

void VideoManager::updateFeed(const std::string &identifier, const char* data, uint offset, uint len)
{
    if (_videoFeeds.find(identifier) != _videoFeeds.end())
    {
        _videoFeeds[identifier]->updateImage(data, offset, len);
    }
    else
    {
        //_videoFeeds[identifier] = new VideoFeed(identifier);
    }
}
