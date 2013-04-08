#ifndef LUAENGINE_H
#define LUAENGINE_H

#include <QObject>

class LuaEngine : public QObject
{
    Q_OBJECT
public:
    explicit LuaEngine(QObject *parent = 0);
    
    int runLua(const std::string& filename);
signals:
    
public slots:
    
};

#endif // LUAENGINE_H
