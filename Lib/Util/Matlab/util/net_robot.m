function h = net_robot(teamNumber, playerID)
% function create the same struct as the team message from
% shared memory. for local debugging use

h.teamNumber = teamNumber;
h.playerID = playerID;
h.user = getenv('USER');
h.robot_msg = {};
h.team_msg = {};
h.yuyv = [];
h.yuvSub = [];
h.labelA = [];
h.labelAsub = [];
h.labelB = [];
h.scale = 1;

% Setup arrays to track the images
h.yuyv_arr = construct_array('yuyv');
h.yuvSub_arr = construct_array('yuvSub');
h.labelA_arr = construct_array('labelA');
h.labelAsub_arr = construct_array('labelAsub');
h.labelB_arr = construct_array('labelB');

% set function pointers
h.update = @update;
h.get_team_struct = @get_team_struct;
h.get_monitor_struct = @get_monitor_struct;
h.get_yuyv = @get_yuyv;
h.get_yuvSub = @get_yuvSub;
h.get_rgb = @get_rgb;
h.get_rgb_sub = @get_rgb_sub;
h.get_labelA = @get_labelA;
h.get_labelAsub = @get_labelAsub;
h.get_labelB = @get_labelB;

    function scale = update( msg )
        %fprintf('msg.team# / h.team#:\t %d / %d\n',msg.team.number,h.teamNumber);
        %fprintf('msg.playerid# / h.playerid#:\t %d / %d\n', msg.team.id, h.playerID);
        % Check if the id field is correct before updating this robot
        if( msg.team.player_id == h.playerID && msg.team.number == h.teamNumber )
            if (isfield(msg, 'arr'))
                h.yuyv  = h.yuyv_arr.update_always(msg.arr);
                h.labelA = h.labelA_arr.update_always(msg.arr);
                h.labelB = h.labelB_arr.update(msg.arr);
                h.labelAsub = h.labelAsub_arr.update(msg.arr);
                h.yuvSub = h.yuvSub_arr.update(msg.arr);
                if(~isempty(h.labelB)) % labelB is gotten in one packet
                    h.scale = 4;
                else
                    h.scale = 2;
                end
            else
                % Update the robot
                h.robot_msg = msg;
                h.team_msg = msg.team;
                h.team_msg.id = h.playerID;
                h.team_msg.teamNumber = h.teamNumber;
                h.team_msg.ball = msg.ball;
                h.team_msg.pose = msg.robot.pose;
                h.team_msg.teamColor = msg.team.color;
            end
        end
        scale = h.scale;
    end

    function r = get_team_struct()
        % returns the robot struct (in the same form as the team messages)
        r = h.team_msg;
        %{
        r = [];
        try
            r.teamNumber = h.gcmTeam.get_number();
            r.teamColor = h.gcmTeam.get_color();
            r.id = h.gcmTeam.get_player_id();
            r.role = h.gcmTeam.get_role();
            
            pose = h.wcmRobot.get_pose();
            r.pose = struct('x', pose(1), 'y', pose(2), 'a', pose(3));
            
            ballxy = h.wcmBall.get_xy();
            ballt = h.wcmBall.get_t();
            r.ball = struct('x', ballxy(1), 'y', ballxy(2), 't', ballt );
            
        catch
        end
        %}
    end

    function r = get_monitor_struct()
        % returns the monitor struct (in the same form as the monitor messages)
        r = h.robot_msg;
        %{
        r = [];
        try
            r.teamNumber = h.gcmTeam.get_number();
            r.teamColor = h.gcmTeam.get_color();
            r.id = h.gcmTeam.get_player_id();
            r.role = h.gcmTeam.get_role();
            
            pose = h.wcmRobot.get_pose();
            r.pose = struct('x', pose(1), 'y', pose(2), 'a', pose(3));
            
            ballxy = h.wcmBall.get_xy();
            ballt = h.wcmBall.get_t();
            ball = {};
            ball.detect = h.vcmBall.get_detect();
            ball.centroid = {};
            centroid = h.vcmBall.get_centroid();
            ball.centroid.x = centroid(1);
            ball.centroid.y = centroid(2);
            ball.axisMajor = h.vcmBall.get_axisMajor();
            r.ball = struct('x', ballxy(1), 'y', ballxy(2), 't', ballt, ...
                'centroid', ball.centroid, 'axisMajor', ball.axisMajor, ...
                'detect', ball.detect);
        catch
        end
        %}
    end

    function yuyv = get_yuyv()
        % returns the raw YUYV image
        yuyv = h.yuyv;
    end

    function [yuv yuv_raw] = get_yuvSub()
        % returns the raw YUV image
        yuv = h.yuvSub;
        width = size(yuv,2)/3;
        height = size(yuv,1);
        yuv = yuv';
        yuv_raw = yuv(:);
        yuv = reshape(yuv(:), [3 height*width]);
        y = reshape(yuv(1,:), [width height]);
        u = reshape(yuv(2,:), [width height]);
        v = reshape(yuv(3,:), [width height]);
        yuv = zeros(width,height,3);
        yuv(:,:,1) = y;
        yuv(:,:,2) = u;
        yuv(:,:,3) = v;
        yuv = permute(yuv,[2 1 3]);
        yuv = uint8(yuv);
    end

    function rgb = get_rgb()
        % returns the raw RGB image (not full size)
        yuyv = h.get_yuyv();
        rgb = yuyv2rgb(yuyv);
    end

    function rgbsub = get_rgb_sub()
        % returns the raw RGB image (not full size)
        [yuv yuv_raw] = h.get_yuvSub();
        rgbsub = ycbcr2rgb(yuv);
    end

    function labelA = get_labelA()
        % returns the labeled image
        labelA = h.labelA;
    end

    function labelAsub = get_labelAsub()
        % returns the labeled image
        labelAsub = h.labelAsub;
    end

    function labelB = get_labelB()
        % returns the bit-ored labeled image
        labelB = h.labelB;
    end
end

