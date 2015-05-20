function [ fileSequence] = getMatFilesFromFolder( folderName )

fileSequence = [];
olderFolder = cd(folderName);
matFiles = dir('*.mat');
if isempty(matFiles)
    cd(olderFolder);
    return;
end

numfiles = length(matFiles);
fileSequence = cell(numfiles,1);

if numfiles < 1 
    cd(olderFolder);
    return;
end

for n=1:numfiles
    fileSequence{n} = strcat(folderName,'/', matFiles(n).name);
end

cd(olderFolder);
