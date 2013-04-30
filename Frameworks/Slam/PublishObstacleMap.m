function PublishObstacleMap
global OMAP

content = VisMap2DSerializer('serialize',OMAP);
ipcAPIPublishVC(OMAP.msgName,content);
