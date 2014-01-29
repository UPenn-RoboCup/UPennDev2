function [map,i_path,j_path,xy,omap] = view_map(map_name)

    map = [];
    if nargin==1,
        fid = fopen(map_name);
        dims = fread(fid,2,'*double');
        data = fread(fid,inf,'*double');
        fclose(fid);
        map = reshape(data,dims(2),dims(1));
        figure(1);
        clf;
        imagesc(map);
        title(map_name);
        axis equal;
        xlabel('x');
        ylabel('y');
    end
    
    i_path = [];
    j_path = [];
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
        plot(i_path,j_path,'b*-');
        plot(start(1),start(2),'g+');
        plot(finish(1),finish(2),'r+');
        hold off;
        axis equal;
        xlabel('x');
        ylabel('y');
    end
    
    fid = fopen('xy.raw');
    xy = [];
    if fid~=-1
        data = fread(fid,inf,'*double');
        fclose(fid);
        xy = reshape(data,2,numel(data)/2);
        figure(3);
        clf;
        x = xy(1,:);
        y = xy(2,:);
        plot(x,y,'.');
        xlabel('x');
        ylabel('y');
        axis equal;
    end
    
    fid = fopen('omap.raw');
    if fid~=-1
        dims = fread(fid,2,'*double');
        data = fread(fid,inf,'*uint8');
        fclose(fid);
        omap = reshape( data, dims(2),dims(1) );
        figure(4);
        imagesc(omap);
        axis equal;
    end
    
end