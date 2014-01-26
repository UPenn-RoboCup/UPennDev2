function [map,i_path,j_path] = view_map(map_name)

    if nargin==1,
        fid = fopen(map_name);
        dims = fread(fid,2,'*double');
        disp(dims)
        data = fread(fid,inf,'*double');
        fclose(fid);
        map = reshape(data,dims(2),dims(1));

        figure(1);
        clf;
        imagesc(map');
        title(map_name);
        axis square;
    end
    
    fid = fopen('path.raw');
    if fid~=-1
        data = fread(fid,inf,'*int');
        fclose(fid);
        n_path = numel(data)/2;
        i_path = data(1:n_path);
        j_path = data(n_path+1:end);
        start = [i_path(1), j_path(1)];
        finish = [i_path(end), j_path(end)];
        figure(2);
        im = imread('map.png');
        imagesc(im);
        hold on;
        plot(j_path,i_path,'b*-');
        plot(start(2),start(1),'g+');
        plot(finish(2),finish(1),'r+');
        hold off;
        
    end
    
end