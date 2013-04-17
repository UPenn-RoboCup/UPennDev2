clear all;
skeleton_s = zmq( 'subscribe',  'skeleton' );
NITE_JOINT_HEAD = 1;
NITE_JOINT_NECK = 2;
NITE_JOINT_LEFT_SHOULDER = 3;
NITE_JOINT_RIGHT_SHOULDER = 4;
NITE_JOINT_LEFT_ELBOW = 5;
NITE_JOINT_RIGHT_ELBOW = 6;
NITE_JOINT_LEFT_HAND = 7;
NITE_JOINT_RIGHT_HAND = 8;
NITE_JOINT_TORSO = 9;
NITE_JOINT_LEFT_HIP = 10;
NITE_JOINT_RIGHT_HIP = 11;
NITE_JOINT_LEFT_KNEE = 12;
NITE_JOINT_RIGHT_KNEE = 13;
NITE_JOINT_LEFT_FOOT = 14;
NITE_JOINT_RIGHT_FOOT = 15;


%% Store the values in meters
pos = zeros(15,3);
offsets = [];

%% Setup the plot
p = plot(pos(:,1),pos(:,2),'r.');
xlabel('x');
ylabel('y');
axis([-1,1, -1, 2]);

while 1
    [data,idx] = zmq('poll',100);
    if numel(data)==1
        positions = msgpack('unpacker', data{1});
        % Save the values, and convert to meters
        for i=1:15
            pos(i,1) = positions{i}.x/1000;
            pos(i,2) = positions{i}.y/1000;
            pos(i,3) = positions{i}.z/1000;
        end
        
        % Send UDP data of the end effector offsets
        offset_r = ...
            pos(NITE_JOINT_RIGHT_HAND,:)-pos(NITE_JOINT_RIGHT_SHOULDER,:);
        offset_l = ...
            pos(NITE_JOINT_LEFT_HAND,:)-pos(NITE_JOINT_LEFT_SHOULDER,:);
        ret = udp_send('send',...
            msgpack('pack', {offset_l,offset_r})...
            );
        if ret < 0
            disp('Bad send');
        end
        
        % Update the plot
        set(p, 'XData', pos(:,1));
        set(p, 'YData', pos(:,2));
        drawnow;
    end
end