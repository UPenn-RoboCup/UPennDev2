%clear all;
%test_lidartrans
%s_vertex = zmq( 'publish', 'ipc', 'vertex' );
matlab_time = now;
unix_time_raw = round(8.64e7 * (matlab_time - datenum('1970', 'yyyy')));
unix_time = unix_time_raw / 1000;
aligned_bytes = verts';
nbytes1 = zmq( 'send', s_vertex, single( aligned_bytes(:) ) );
unix_time
verts(1,:)