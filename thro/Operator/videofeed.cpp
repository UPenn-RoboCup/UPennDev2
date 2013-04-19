#include "videofeed.h"
#include <QImage>

VideoFeed::VideoFeed(const std::string& identifier, QImage* image)
    : _identifier(identifier), _image(image)
{

}

void VideoFeed::updateImage(const char* data, int offset, int len)
{

}
