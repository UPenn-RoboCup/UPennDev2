%% Cleanup
%close all;
clear all;

f_yuyv = figure(10);
im_yuyv = image();
a_yuyv = gca;
set(a_yuyv, 'xlimmode','manual',...
'ylimmode','manual',...
'ydir','reverse');
hold on;
p_l1 = plot([0],[0],'m*-');
p_l2 = plot([0],[0],'k*-');
p_bb = plot([0],[0],'r+-');
hold off;
drawnow;

f_radon = figure(11);
im_radon = imagesc();
a_radon = gca;
set(a_radon, 'xlimmode','manual',...
'ylimmode','manual',...
'ydir','reverse');

f_grey = figure(12);
subplot(1,2,1);
im_grey = imagesc();
a_grey = gca;
set(a_grey, 'xlimmode','manual',...
'ylimmode','manual',...
'ydir','reverse');
axis image;
hold on;
p_l1s = plot([0],[0],'m*-');
p_l2s = plot([0],[0],'k*-');
hold off;
colorbar;
subplot(1,2,2);
im_edge = imagesc();
a_edge = gca;
set(a_edge, 'xlimmode','manual',...
'ylimmode','manual',...
'ydir','reverse');
axis image;
colorbar;

f_radon2 = figure(13);
subplot(1,2,1);
im_radon2 = imagesc();
a_radon2 = gca;
set(a_radon2, 'xlimmode','manual',...
'ylimmode','manual',...
'ydir','reverse');
subplot(1,2,2);
im_iradon2 = imagesc();
a_iradon2 = gca;
set(a_iradon2, 'xlimmode','manual',...
'ylimmode','manual',...
'ydir','reverse');

%f_line_sum = figure(14);
%nth = 180;
%nr = 101;
%im_line_sum = imagesc();

%% Channel setup
skt = zmq( 'subscribe', 'ipc', 'edge' );
while 1
    idx = zmq('poll',1000);
    for s=1:numel(idx)
        s_idx = idx(s);
        [data, has_more] = zmq( 'receive', s_idx );
        % Get the metadata
        [metadata, offset] = msgpack('unpack',data);
        % Get the kernel that was used
        k_h = metadata.kh;
        k_w = metadata.kw;
        kernel = reshape( typecast(metadata.kernel, 'double'), k_h, k_w );
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
            offset_x = double(x0+2);
            offset_y = double(y0+2);
            l1_x = double([metadata.l1.iMin, metadata.l1.iMean,...
                metadata.l1.iMax]);
            l1_y = double([metadata.l1.jMin, metadata.l1.jMean,...
                metadata.l1.jMax]);
            l2_x = double([metadata.l2.iMin, metadata.l2.iMean,...
                metadata.l2.iMax]);
            l2_y = double([metadata.l2.jMin, metadata.l2.jMean,...
                metadata.l2.jMax]);
            % Calculations done in subsampled space of half size.
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
        jpg = djpeg(data);
        set(im_yuyv, 'Cdata', jpg);
        set(a_yuyv, 'XLim', [1, size(jpg,2)], 'YLim', [1, size(jpg,1)]);
        
        % Receive the radon space
        if has_more~=1
            disp('NO RADON RECEIVED');
            return
        end
        [data, has_more] = zmq( 'receive', s_idx );
        radon_space = reshape(typecast(data,'uint32'), ...
            double(metadata.MAXR), double(metadata.NTH));
        set(im_radon, 'Cdata', radon_space);
        set(a_radon, 'XLim', [1, size(radon_space,2)], 'YLim', [1, size(radon_space,1)]);
        
        
        % Receive the transformed gray space
        if has_more~=1
            disp('NO TRANSFORMED GRAY RECEIVED');
            return
        end
        [data, has_more] = zmq( 'receive', s_idx );
        grey_space = reshape(typecast(data,'double'), bbox_w, bbox_h)';
        set(im_grey, 'Cdata', grey_space);
        set(a_grey, 'XLim', [1, size(grey_space,2)], 'YLim', [1, size(grey_space,1)]);
        if isfield(metadata,'l1')
            % Calculations done in subsampled space of half size. So scale by 2
            set(p_l1s, 'Xdata', l1_x+3);
            set(p_l1s, 'Ydata', l1_y+3);
            set(p_l2s, 'Xdata', l2_x+3);
            set(p_l2s, 'Ydata', l2_y+3);
        end
        
        % Receive the edge image
        if has_more~=1
            disp('NO EDGE IMAGE RECEIVED');
            return
        end
        [data, has_more] = zmq( 'receive', s_idx );
        edge_img = reshape(typecast(data,'double'), bbox_w-(k_w-1), bbox_h-(k_h-1))';
        edge_img = double(edge_img);
        sigma = std(edge_img(:));
        %edge_img(abs(edge_img)<sigma) = 0;
        %edge_img(edge_img>=sigma) = 1;
        %edge_img(edge_img<=-sigma) = -1;
        set(im_edge, 'Cdata', edge_img);
        set(a_edge, 'XLim', [1, size(edge_img,2)], 'YLim', [1, size(edge_img,1)]);
        
        % Receive the line sums
        %{
        if has_more~=1
            disp('NO LINE SUMS RECEIVED');
            return
        end
        [data, has_more] = zmq( 'receive', s_idx );
        line_sum = reshape(typecast(data,'int32'), double(metadata.MAXR), double(metadata.NTH));
        set(im_line_sum, 'Cdata', line_sum );
        %}
        
        % Form the MATLAB radon
        R = radon(edge_img,0:179);
        IR = iradon(R,0:179);
        set(im_radon2, 'Cdata', R);
        set(a_radon2, 'XLim', [1, size(R,2)], 'YLim', [1, size(R,1)]);
        set(im_iradon2, 'Cdata', IR);
        set(a_iradon2, 'XLim', [1, size(IR,2)], 'YLim', [1, size(IR,1)]);
        
    end
    drawnow;
end