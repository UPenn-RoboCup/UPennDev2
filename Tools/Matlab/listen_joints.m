clear all;
skeleton_s = zmq( 'subscribe',  'actuator_cmd' );

%% Set up easy variables
Head=1;Neck=2;
L_Shoulder_Pitch=3; L_Shoulder_Roll=4; L_Shoulder_Yaw=5;L_Elbow=6;
L_Hip_Yaw=7; L_Hip_Roll=8; L_Hip_Pitch=9; L_Knee_Pitch=10; L_Ankle_Pitch=11; L_Ankle_Roll=12;
R_Hip_Yaw=13; R_Hip_Roll=14; R_Hip_Pitch=15; R_Knee_Pitch=16; R_Ankle_Pitch=17; R_Ankle_Roll=18;
R_Shoulder_Pitch=19; R_Shoulder_Roll=20; R_Shoulder_Yaw=21;R_Elbow=22;
Waist_Roll=23;

%% Set up the indices
arm_idx_l = L_Shoulder_Pitch:L_Elbow;
arm_idx_r = R_Shoulder_Pitch:R_Elbow;
leg_idx_l = L_Hip_Yaw:L_Ankle_Roll;
leg_idx_r = R_Hip_Yaw:R_Ankle_Roll;
spine_idx = [1,2,23];

%% Store the values in meters
js = zeros(23);
nlogs = 300;
js_log = zeros(nlogs,23);
log_num = 1;
offsets = [];

while 1
    [data,idx] = zmq('poll',100);
    if numel(data)==1
        %% Record logs
        js_log(log_num,:) = typecast(data{1}, 'double');
        log_num = log_num+1;
    end
end

%% Plot data
figure(2);
plot( js_log(:,L_Shoulder_Roll), js_log(:,L_Elbow), 'r*');