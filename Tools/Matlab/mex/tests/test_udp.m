clear all;
recv_fd = udp_recv();
udp_recv('getQueueSize');
s_udp = zmq( 'fd', recv_fd );

while 1
    [data,idx] = zmq('poll',1000);
    if numel(idx)~=0
        udp_data = udp_recv('receive');
        if numel(udp_data)>0
            fprintf('data amount: %d bytes.\n', numel(udp_data) )
        end
    else
        disp('no data');
    end
end