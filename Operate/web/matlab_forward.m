tmp = single( verts(:) )
s_vertex = zmq( 'publish', 'ipc', 'vertex' );
nbytes1 = zmq( 'send', s_vertex, tmp )
tmp(201:203)
