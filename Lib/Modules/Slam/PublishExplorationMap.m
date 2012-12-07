function PublishExplorationMap
global EMAP

%publish the exploration map to Vis
content = VisMap2DSerializer('serialize',EMAP);
ipcAPIPublishVC(EMAP.msgName,content);
