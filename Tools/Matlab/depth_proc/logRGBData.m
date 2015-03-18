% logData
function logRGBData(datargb, metargb)

persistent datestamp
persistent pathToSave
persistent count

if isempty(datestamp) 
    count = 0;
    datestamp = int2str(yyyymmdd(datetime));
    
    olderFolder = cd('/home/leebhoram/Data/LOGS_SC');
    if ~exist(datestamp,'dir'),mkdir(datestamp);end
    cd(olderFolder);
    
    pathToSave = sprintf('/home/leebhoram/Data/LOGS_SC/%s/',datestamp);
end

filename = sprintf('rgb_%06d.mat',count); 
save(strcat(pathToSave,filename),'datargb', 'metargb');
count = count + 1;
end