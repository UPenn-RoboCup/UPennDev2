vidObj = VideoWriter('slam.avi');
open(vidObj);

shm_slam4
currFrame = getframe;
writeVideo(vidObj,currFrame)