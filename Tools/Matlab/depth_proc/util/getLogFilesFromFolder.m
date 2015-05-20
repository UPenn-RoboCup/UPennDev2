function [ fileSequence] = getLogFilesFromFolder( folderName, type )

fileSequence = [];
olderFolder = cd(folderName);
logFiles = dir('*.log');
if isempty(logFiles)
    cd(olderFolder);
    return;
end

numfiles = numel(logFiles);

if numfiles < 1 
    cd(olderFolder);
    return;
end

if strcmp(type,'kinect') == 1
    n_k = 0;
    for n=1:numfiles
        if ~isempty(strfind(logFiles(n).name,'depth_m'))
            n_k = n_k + 1;
            fileSequence{n_k} = strcat(folderName,'/', logFiles(n).name);
        end
    end
else strcmp(type,'lidar') == 1
    disp('lidar to be added');
    
end

cd(olderFolder);
