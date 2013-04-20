#ifndef VIDEOFEED_H
#define VIDEOFEED_H

#include <string>

class QImage;

class VideoFeed
{
public:
    VideoFeed(const std::string& identifier, QImage* image);

    void updateImage(const char* data, int offset, int len);

private:
    std::string _identifier;
    QImage* _image;

};

#endif // VIDEOFEED_H
