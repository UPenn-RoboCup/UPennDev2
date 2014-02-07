function ret = controlbody()
global CONTROL SLAM BODY LIDAR POSE MODELS WAYPOINTS

CONTROL=[];

CONTROL.body=[];
CONTROL.head=[];
CONTROL.arm=[];

CONTROL.body_control = @body_control;
CONTROL.head_control = @head_control;
CONTROL.arm_control = @arm_control;
CONTROL.send_control_packet=@send_control_packet;

CONTROL.udp_send_id = -1;
ret = CONTROL;

    function head_control(h,~,evt)
        deg2rad = pi/180;
        if strcmp(evt,'stepscan')
            CONTROL.send_control_packet([],[],...
            'vcm','head_lidar','scanlines',[0*deg2rad, 60*deg2rad, 5/deg2rad]);
            send_control_packet('HeadFSM', 'tiltscan');
        end
        send_control_packet('HeadFSM',evt);
    end

    function body_control(h,~,evt)
        if strcmp(evt,'approach')
            % Set waypoints to shared memory
            set_waypoints(4);
            %send_control_packet( 'HeadFSM', 'center');
            send_control_packet( 'BodyFSM', 'follow' );
        elseif strcmp(evt, 'navigate')
            set_waypoints(2);
            %send_control_packet( 'HeadFSM', 'center');
            send_control_packet( 'BodyFSM', 'follow' );
        elseif strcmp(evt, 'sidedone')
            send_control_packet([], [],...
                'hcm', 'motion', 'sideways_status', 1);
        elseif strcmp(evt, 'stepover')            
            %should define motion fsm as well
            send_control_packet( 'BodyFSM', 'stepover' );        
        else

            send_control_packet( 'BodyFSM', evt );
        end
    end

    function lidar_control(h,~,evt)
        send_control_packet('LidarFSM',evt);
    end

    function arm_control(h,~,evt)
        if strcmp(evt,'grab')
            send_control_packet( 'ArmFSM', strcat(MODELS.ooi,'grab') );
        else
            send_control_packet( 'ArmFSM', evt );
        end
    end

    function motion_control(h,~,evt)
        send_control_packet('MotionFSM',evt);
    end

    function set_waypoints(flags)
        if flags==1
            send_control_packet('BodyFSM','teleop');
        elseif flags==2
            %waypoint=SLAM.get_waypoints();
            waypoint = WAYPOINTS.get_waypoints();            
            disp(waypoint)
            if numel(waypoint)>0
                waypoint_num = size(waypoint,1);
                send_control_packet([], [],...
                    'hcm', 'motion', 'waypoints',...
                    reshape(waypoint,[1 3*waypoint_num]));
                send_control_packet([], [],...
                    'hcm', 'motion', 'nwaypoints', waypoint_num);
                send_control_packet([], [],...
                    'hcm', 'motion', 'waypoint_frame', 1); %global
            end
        elseif flags==3
            disp('Single point approach')
            %targetpos = LIDAR.get_single_approach();
            targetpos = WAYPOINTS.get_single_approach();

            if numel(targetpos)>0
                send_control_packet([], [],...
                    'hcm', 'motion', 'waypoints',...
                    reshape(targetpos,[1 3]));
                send_control_packet([], [],...
                    'hcm', 'motion', 'nwaypoints', 1);
                send_control_packet([], [],...
                    'hcm', 'motion', 'waypoint_frame', 1); %Global
            end
        elseif flags==4
            disp('Two point approach')
            %wp = LIDAR.get_double_approach();
            wp = WAYPOINTS.get_double_approach();
            if numel(wp) == 0
              wp = WAYPOINTS.get_double_approach();
            end
            if numel(wp)>0
              send_control_packet([], [],...
                    'hcm', 'motion', 'waypoints',...
                    reshape(wp,[1 3]));
                send_control_packet([], [],...
                    'hcm', 'motion', 'nwaypoints', 1);
                send_control_packet([], [],...
                    'hcm', 'motion', 'waypoint_frame', 1); %Global
            end
        end
    end


    function send_control_packet( fsmtype, event, shared, segment, key, data )
        disp('Sending a control packet')
        
        senddata=[];
        if numel(fsmtype)>0
            fprintf('Control fsm:\t%s / %s\n',fsmtype,event);
            senddata.fsm = fsmtype;
            senddata.evt = event;
        end
        
        if nargin==6
            fprintf('Control data:\t%s / %s\n',shared,segment,key);
            senddata.shm = shared;
            senddata.segment = segment;
            senddata.key = key;
            senddata.val = data;
            disp(data);
        end
        
        if numel(senddata)==0
            return;
        end
        
        % Form the string payload
        data_msg = msgpack( 'pack', senddata );
        % Send UDP packet
        % TODO: This should use reliable rpc...
        nBytes   = udp_send( 'send', CONTROL.udp_send_id, data_msg );
        fprintf( 'Control packet sent %d/%d bytes\n', nBytes, numel(data_msg) );
        disp(senddata);
    end
end
