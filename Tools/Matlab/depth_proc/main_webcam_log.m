clear all;
close all;

cam = webcam
if ~isempty(cam) && ~isempty(cam.Name)
    pause;
    for n=1:10
        img{n} = snapshot(cam);  
        figure(1), imshow(img{n});
        pause;
    end
end