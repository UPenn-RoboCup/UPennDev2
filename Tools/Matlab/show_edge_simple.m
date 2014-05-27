%% Cleanup
%close all;
clear all;

f_radon = figure(1);
im_radon = imagesc();
a_radon = gca;
set(a_radon, 'xlimmode','manual',...
'ylimmode','manual',...
'ydir','reverse');
drawnow;

%% Channel setup
skt = zmq( 'subscribe', 'ipc', 'radon' );
while 1
    idx = zmq('poll',1000);
    for s=1:numel(idx)
        s_idx = idx(s);
        [data, has_more] = zmq( 'receive', s_idx );
        % Get the metadata
        [metadata, offset] = msgpack('unpack',data);
        
        % Get the radon counts
        [data, has_more] = zmq( 'receive', s_idx );
        radon_space = reshape(typecast(data,'int32'), ...
            double(metadata.MAXR), double(metadata.NTH));
        set(im_radon, 'Cdata', radon_space);
        set(a_radon, 'XLim', [1, size(radon_space,2)], 'YLim', [1, size(radon_space,1)]);
        
    end
    drawnow;
end