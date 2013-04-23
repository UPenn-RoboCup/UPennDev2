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

arm_idx_l = 3:2:7;
arm_idx_r = 4:2:8;
leg_idx_l = 10:2:14;
leg_idx_r = 11:2:15;
spine_idx = [1,2,9];

%% Publish joint values
%actuator_p = zmq( 'publish',  'actuator' );

%% Store the values in meters
pos = zeros(15,3);
offsets = [];

%% Setup the plot
%p = plot3(pos(:,1),pos(:,2),pos(:,3),'r.');
figure(1);
clf;
hold on;
p_arm_l = plot3(pos(arm_idx_l,1),pos(arm_idx_l,2),pos(arm_idx_l,3),'r.-');
p_arm_r = plot3(pos(arm_idx_r,1),pos(arm_idx_r,2),pos(arm_idx_r,3),'b.-');
p_leg_l = plot3(pos(leg_idx_l,1),pos(leg_idx_l,2),pos(leg_idx_l,3),'r*-');
p_leg_r = plot3(pos(leg_idx_r,1),pos(leg_idx_r,2),pos(leg_idx_r,3),'b*-');
p_spine = plot3(pos(spine_idx,1),pos(spine_idx,2),pos(spine_idx,3),'ko-');
hold off;

xlabel('x');
ylabel('y');
zlabel('z');
axis([ 1.8, 2.5, -1,1, -1.2, 1.5]);

while 1
    [data,idx] = zmq('poll',100);
    if numel(data)==1
        positions = msgpack('unpacker', data{1});
        % Save the values, and convert to meters
        for i=1:15
            pos(i,1) = positions{i}.z/1000;
            pos(i,2) = positions{i}.x/1000;
            pos(i,3) = positions{i}.y/1000;
        end
        
        % Send UDP data of the end effector offsets
        %offset_r = ...
        %    pos(NITE_JOINT_RIGHT_HAND,:)-pos(NITE_JOINT_RIGHT_SHOULDER,:);
        %offset_l = ...
        %    pos(NITE_JOINT_LEFT_HAND,:)-pos(NITE_JOINT_LEFT_SHOULDER,:);
        %zmq( 'send', actuator_p, offset_r )
        
        %% Update the plot
        %{
        set(p, 'XData', pos(:,1));
        set(p, 'YData', pos(:,2));
        set(p, 'ZData', pos(:,3));
        %}
        set(p_arm_l, 'XData', pos(arm_idx_l,1));
        set(p_arm_l, 'YData', pos(arm_idx_l,2));
        set(p_arm_l, 'ZData', pos(arm_idx_l,3));
        %
        set(p_arm_r, 'XData', pos(arm_idx_r,1));
        set(p_arm_r, 'YData', pos(arm_idx_r,2));
        set(p_arm_r, 'ZData', pos(arm_idx_r,3));
        %
        set(p_leg_l, 'XData', pos(leg_idx_l,1));
        set(p_leg_l, 'YData', pos(leg_idx_l,2));
        set(p_leg_l, 'ZData', pos(leg_idx_l,3));
        %
        set(p_leg_r, 'XData', pos(leg_idx_r,1));
        set(p_leg_r, 'YData', pos(leg_idx_r,2));
        set(p_leg_r, 'ZData', pos(leg_idx_r,3));
        %
        set(p_spine, 'XData', pos(spine_idx,1));
        set(p_spine, 'YData', pos(spine_idx,2));
        set(p_spine, 'ZData', pos(spine_idx,3));
        %% Draw
        drawnow;
    end
end