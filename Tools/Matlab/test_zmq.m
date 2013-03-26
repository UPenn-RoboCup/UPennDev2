%http://www.mathworks.com/help/matlab/matlab_external/bringing-java-classes-and-methods-into-matlab-workspace.html
%javaaddpath('/usr/local/share/java/zmq.jar');
%java.lang.System.load('/usr/local/lib/libzmq.dylib');
%java.lang.System.load('/usr/local/lib/libjzmq.dylib');
%zmsq_obj = javaObjectEDT('org.zeromq.ZMsg');
%ctx = javaObjectEDT('org.zeromq.ZContext');
%methodsview org.zeromq.ZMQ$Socket

% SUB: 2 (NOTE: This may change!)
%javaMethodEDT('createSocket',ctx, 2);
%skt = ctx.createSocket( 2 );
%skt.connect('ipc:///tmp/img');
% https://www.mathworks.com/support/solutions/en/data/1-OVUMA/index.html?solution=1-OVUMA
clear all;
javaaddpath('/usr/local/share/java/zmq.jar');
import org.zeromq.*
a = ZContext();
%{
% This works
s = a.createSocket(1); %1-pub, 2-sub
channel = 'ipc:///tmp/img';
s.bind( channel );
msg = java.lang.String('hello');
s.send( msg.getBytes() );
%}

s = a.createSocket(2); %1-pub, 2-sub
channel = 'ipc:///tmp/img2';
s.connect( channel );
mask=java.lang.String('a');
s.subscribe( mask.getBytes() );
s.setReceiveTimeOut(1);
%%
while 1
    q = s.recv();
    if(~isempty(q))
        disp('got img');
%        qq = reshape( q(2:end),[640 480]);
%        imagesc(qq);
        img = djpeg(q(2:end));
        %imagesc( img );
    end
    %pause(.1);
end
