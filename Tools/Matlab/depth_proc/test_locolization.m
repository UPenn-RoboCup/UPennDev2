clear all;
close all;
foldername = '/Volumes/Macintosh/Users/jianqiaoli/Desktop/testdata/corner_walk2/Unpacked/';
datestamp = '03.25.2015.16.40.49';
% foldername = '/Volumes/Macintosh/Users/jianqiaoli/Desktop/testdata/';
% datestamp = '03.25.2015.12.48.39';

[ fileSequence] = getMatFilesFromFolder( strcat(foldername,datestamp));
for ilog=1:length(fileSequence)
    load(fileSequence{ilog}); 
    uisetting;
    [x,y,head_theta,body_theta,flag]=locolization(depthRaw,metad,rgb_img);
    pause(0.0001)
end