%% Cleanup
%close all;
clear all;

f_yuyv = figure(10);
im_yuyv = image(zeros(240, 320));
hold on;
p_l1 = plot([0],[0],'m*-');
p_l2 = plot([0],[0],'k*-');
p_bb = plot([0],[0],'r+-');
hold off;
drawnow;

f_radon = figure(11);
nth = 45;
nr = 101;
im_radon = imagesc(zeros(nr, nth));

f_grey = figure(12);
subplot(1,2,1);
im_grey = imagesc(zeros(101, 31));
hold on;
p_l1s = plot([0],[0],'m*-');
p_l2s = plot([0],[0],'k*-');
hold off;
axis image;
colorbar;
subplot(1,2,2);
im_edge = imagesc(zeros(97, 27));
axis image;
colorbar;

%% Channel setup
skt = zmq( 'subscribe', 'ipc', 'edge' );
while 1
    idx = zmq('poll',1000);
    for s=1:numel(idx)
        s_idx = idx(s);
        [data, has_more] = zmq( 'receive', s_idx );
        % Get the metadata
        [metadata, offset] = msgpack('unpack',data);
        % Show the bounding box
        bbox = double(metadata.bbox);
        x0 = bbox(1);
        x1 = bbox(2);
        y0 = bbox(3);
        y1 = bbox(4);
        bbox_w = x1 - x0 + 1;
        bbox_h = y1 - y0 + 1;
        % Times 2 since we were in the downsampled space
        xbox = 2*([x0, x1, x1, x0, x0]);
        ybox = 2*([y0, y0, y1, y1, y0]);
        set(p_bb, 'XData', xbox);
        set(p_bb, 'YData', ybox);
        % Set the lines
        % (5x5 Kernel means offset of 2. 1 indexing means off of 1. So 3.)
        if isfield(metadata,'l1')
            offset_x = double(metadata.offset(1)+3);
            offset_y = double(metadata.offset(2)+3);
            l1_x = double([metadata.l1.iMin, metadata.l1.iMean metadata.l1.iMax]);
            l1_y = double([metadata.l1.jMin, metadata.l1.jMean metadata.l1.jMax]);
            l2_x = double([metadata.l2.iMin, metadata.l2.iMean metadata.l2.iMax]);
            l2_y = double([metadata.l2.jMin, metadata.l2.jMean metadata.l2.jMax]);
            % Calculations done in subsampled space of half size. So scale by 2
            set(p_l1, 'Xdata', 2*(l1_x+offset_x) );
            set(p_l1, 'Ydata', 2*(l1_y+offset_y) );
            set(p_l2, 'Xdata', 2*(l2_x+offset_x) );
            set(p_l2, 'Ydata', 2*(l2_y+offset_y) );
        end
        
        % Process the JPEG data
        if has_more~=1
            disp('NO JPEG RECEIVED');
            return
        end
        [data, has_more] = zmq( 'receive', s_idx );
        % Set the jpeg image
        set(im_yuyv, 'Cdata', djpeg(data));
        
        % Receive the radon space
        if has_more~=1
            disp('NO RADON RECEIVED');
            return
        end
        [data, has_more] = zmq( 'receive', s_idx );
        radon_space = reshape(typecast(data,'uint32'), double(metadata.MAXR), double(metadata.NTH));
        set(im_radon, 'Cdata', radon_space);
        
        % Receive the transformed gray space
        if has_more~=1
            disp('NO TRANSFORMED GRAY RECEIVED');
            return
        end
        [data, has_more] = zmq( 'receive', s_idx );
        grey_space = reshape(typecast(data,'int32'), bbox_w, bbox_h)';
        set(im_grey, 'Cdata', grey_space);
        if isfield(metadata,'l1')
            % Calculations done in subsampled space of half size. So scale by 2
            set(p_l1s, 'Xdata', l1_x);
            set(p_l1s, 'Ydata', l1_y);
            set(p_l2s, 'Xdata', l2_x);
            set(p_l2s, 'Ydata', l2_y);
        end
        
        % Receive the edge image
        if has_more~=1
            disp('NO EDGE IMAGE RECEIVED');
            return
        end
        [data, has_more] = zmq( 'receive', s_idx );
        edge_img = reshape(typecast(data,'int32'), bbox_w-4, bbox_h-4)';
        set(im_edge, 'Cdata', edge_img);
        
        drawnow;
    end
end

%% Raw image and sample gray
figure(1);
subplot(1,2,1);
imshow('Data/edge_img.jpeg');
axis image;
subplot(1,2,2);
imshow('Data/edge_gray.jpeg');
axis image;

%% Edges from LoG convolution
%h = 120;
%w = 160;
%h = 116;
%w = 156;
h = 97;
w = 27;
% raw edges
fid = fopen('Data/edge.raw');A = fread(fid,Inf,'*int32');fclose(fid);
edge_raw = double(reshape(A,[w, h])');
% char edges
fid = fopen('Data/edge_char.raw');A = fread(fid,Inf,'*int8');fclose(fid);
edge_char = logical(reshape(A,[w, h])');
%
figure(2);
subplot(1,2,1);
imagesc(edge_raw);
colormap;
axis image;
subplot(1,2,2);
imagesc(edge_char);
axis image;

%% Line counts and sums
nr = 101;
%nth = 36;
%nth = 45;
nth = 180;
fid = fopen('Data/line_cnts.raw');A = fread(fid,Inf,'*uint32');fclose(fid);
line_counts = double(reshape(A,[nr, nth]));
fid = fopen('Data/line_sums.raw');A = fread(fid,Inf,'*int32');fclose(fid);
line_sums = double(reshape(A,[nr, nth]));
%
figure(3);
subplot(2,1,1);
imagesc(line_counts);
%mesh(line_counts);
title('Line Counts');
subplot(2,1,2);
imagesc(line_sums);
%mesh(line_sums);
title('Line Sums');
%
figure(4);
% Origin is the center pixel. For us, it it 0,0 top left
% Use explicit default theta
TH = 0:179;
R = radon(edge_char,TH);
IR = iradon(R,TH);
subplot(2,1,1);
imagesc(R);
title('MATLAB gut check');
subplot(2,1,2);
imagesc(IR);

figure(5);
subplot(2,1,1);
plot(max(line_counts));
xlim([0,nth]);
subplot(2,1,2);
plot(std(line_counts));
xlim([0,nth]);

%% Line extrema
%{
nr = 106;
nth = 36;
%nth = 180;
fid = fopen('Data/line_min.raw');A = fread(fid,Inf,'*int32');fclose(fid);
line_min = double(reshape(A,[nr, nth]));
fid = fopen('Data/line_max.raw');A = fread(fid,Inf,'*int32');fclose(fid);
line_max = double(reshape(A,[nr, nth]));
line_min(line_min>nr) = 0;
line_max(line_max<-nr) = 0;
%
figure(6);
subplot(2,1,1);
imagesc(line_min);
subplot(2,1,2);
imagesc(line_max);
%}
%% bbox of the image
w = 31; h = 101;
%w = 31; h = 41;
fid = fopen('Data/ys_bbox.raw');A = fread(fid,Inf,'*uint8');fclose(fid);
ys_bbox = double(reshape(A,[w, h])');
fid = fopen('Data/us_bbox.raw');A = fread(fid,Inf,'*uint8');fclose(fid);
us_bbox = double(reshape(A,[w, h])');
fid = fopen('Data/vs_bbox.raw');A = fread(fid,Inf,'*uint8');fclose(fid);
vs_bbox = double(reshape(A,[w, h])');
fid = fopen('Data/trans_bbox.raw');A = fread(fid,Inf,'*double');fclose(fid);
trans_bbox = double(reshape(A,[w, h])');
%
figure(7);
subplot(2,2,1);
imagesc(ys_bbox);
title('Y');
axis image;
colorbar;
subplot(2,2,2);
imagesc(us_bbox);
title('U');
axis image;
colorbar;
subplot(2,2,3);
imagesc(vs_bbox);
title('V');
axis image;
colorbar;
subplot(2,2,4);
imagesc(255*trans_bbox);
title('Transformed');
axis image;
colorbar;