function h = net_robot(teamNumber, playerID)
% function create the same struct as the team message from
% shared memory. for local debugging use

h.teamNumber = teamNumber;
h.playerID = playerID;
h.user = getenv('USER');
h.robot_msg = {};
h.team_msg = {};
h.y = uint8(0);
h.v = uint8(0);
h.u = uint8(0);
h.yuv = [];
h.yuv2 = [];
h.yuyv = [];
h.labelA = [];
h.labelB = [];
h.scale = 1;

% Setup arrays to track the images 
	% Number defined in sending function
h.labelA_arr = image_array(16);
h.labelB_arr = image_array(17);
h.y_arr = image_array(18);
h.u_arr = image_array(19);
h.v_arr = image_array(20);


% set function pointers
h.update = @update;
h.get_team_struct = @get_team_struct;
h.get_monitor_struct = @get_monitor_struct;
h.get_yuyv = @get_yuyv;
h.get_yuyv2 = @get_yuyv2;
h.get_yuvSub = @get_yuvSub;
h.get_rgb = @get_rgb;
h.get_rgb_sub = @get_rgb_sub;
h.get_labelA = @get_labelA;
h.get_labelAsub = @get_labelAsub;
h.get_labelB = @get_labelB;

    function ret = update( msg )
        %fprintf('msg.team# / h.team#:\t %d / %d\n',msg.team.number,h.teamNumber);
        %fprintf('msg.playerid# / h.playerid#:\t %d / %d\n', msg.team.id, h.playerID);
        % Check if the id field is correct before updating this robot
        if( msg.team.player_id == h.playerID && msg.team.number == h.teamNumber )
            if (isfield(msg,'image'))
%							disp(msg.image)
								h.y  = h.y_arr.update(msg.image);
								h.u  = h.u_arr.update(msg.image);
								h.v  = h.v_arr.update(msg.image);
								h.labelA = h.labelA_arr.update(msg.image);
								h.labelB = h.labelB_arr.update(msg.image);
%								disp([h.labelB_arr.get_width(),h.labelB_arr.get_height()]);
								% yuyv
								[Yupdate,Ycount] = h.y_arr.get_update();
								[Uupdate,Ucount] = h.u_arr.get_update();
								[Vupdate,Vcount] = h.v_arr.get_update();
%								disp([Ycount,Ucount,Vcount]);
				        if (Yupdate && Uupdate && Vupdate)
									h.y_arr.reset();
									h.u_arr.reset();
									h.v_arr.reset();
									y_width = h.y_arr.get_width();
									y_height = h.y_arr.get_height();
									h.yuv = uint32(h.y)*16777216*4 + uint32(h.u)*256*4 + uint32(h.v)*4; 
									h.yuyv = reshape(h.yuv,y_width,y_height);
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
        ret = 1;
    end

    function r = get_team_struct()
        % returns the robot struct (in the same form as the team messages)
        r = h.team_msg;
%        r = [];
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
    end

    function r = get_monitor_struct()
        % returns the monitor struct (in the same form as the monitor messages)
        r = h.robot_msg;
%        r = [];
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

						r.goal = {};
						r.goal.detect = 0;

						r.free = {};
						r.free.detect = 0;

						r.bd = {};
						r.bd.detect = 0;

						r.occ = {};
						r.occ.detect = 0;
        catch
        end
    end

    function yuyv = get_yuyv()

        % returns the raw YUYV image
        yuyv = h.yuyv;
    end

    function rgb = get_rgb()
        % returns the raw RGB image (not full size)
        yuyv = h.get_yuyv();
        rgb = yuyv2rgb(yuyv);
    end

    function labelA = get_labelA()
        % returns the labeled image
        labelA = h.labelA;
    end

    function labelB = get_labelB()
        % returns the bit-ored labeled image
				if (size(h.labelB,2)>1) 
					widthB = h.labelB_arr.get_width();
					heightB = h.labelB_arr.get_height();
	        labelB = reshape(h.labelB,[widthB,heightB])';
				else
					labelB = [];
				end
    end
end

