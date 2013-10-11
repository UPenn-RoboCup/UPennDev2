function ret = controlbody()
global CONTROL SLAM BODY LIDAR POSE MODELS

CONTROL=[];

CONTROL.body=[];
CONTROL.head=[];
CONTROL.arm=[];
CONTROL.setup_slambody_controls = @setup_slambody_controls;
CONTROL.setup_robotbody_controls = @setup_robotbody_controls;
CONTROL.setup_lidarbody_controls = @setup_lidarbody_controls;
CONTROL.setup_model_controls = @setup_model_controls;

CONTROL.setup_body_controls = @setup_body_controls;
CONTROL.setup_head_controls = @setup_head_controls;
CONTROL.setup_arm_controls = @setup_arm_controls;
CONTROL.setup_wp_controls = @setup_wp_controls;

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
            send_control_packet( 'HeadFSM', 'center');
            send_control_packet( 'BodyFSM', 'follow' );
        elseif strcmp(evt, 'navigate')
            set_waypoints(2);
            send_control_packet( 'HeadFSM', 'center');
            send_control_packet( 'BodyFSM', 'follow' );
        elseif strcmp(evt, 'sidedone')
            send_control_packet([], [],...
                'hcm', 'motion', 'sideways_status', 1);
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

    function setup_slambody_controls(b1,b2,b3)
        set(b1,'CallBack',{SLAM.set_zoomlevel,1});
        set(b2,'CallBack',{SLAM.set_zoomlevel,2});
        set(b3,'CallBack', SLAM.clear_waypoint);
    end

    function setup_robotbody_controls(b1,b2,b3,b4)
        set(b1,'CallBack',{BODY.set_viewpoint,1});
        set(b2,'CallBack',{BODY.set_viewpoint,2});
        set(b3,'CallBack',{BODY.set_viewpoint,3});
        set(b4,'CallBack',{BODY.set_viewpoint,4});
    end

    function setup_lidarbody_controls(b1,b2, lmb1,lmb2,lmb3,lmb4,lmb5)
        set(b1,'CallBack',{LIDAR.set_meshtype,0});
        set(b2,'CallBack',{LIDAR.set_meshtype,1});
        
        set(lmb1,'CallBack',{LIDAR.set_zoomlevel,1});
        set(lmb2,'CallBack',{LIDAR.set_zoomlevel,2});
        set(lmb3,'CallBack',{LIDAR.clear_points});
        set(lmb4,'CallBack',{LIDAR.set_img_display});
        set(lmb5,'CallBack',{LIDAR.get_depth_img});
    end

    function setup_model_controls(b1,b2,b3, b4)
        set(b1,'CallBack',MODELS.wheel_calc);
        set(b2,'CallBack',MODELS.door_calc);
        set(b3,'CallBack',MODELS.tool_calc);
        set(b4,'CallBack',MODELS.step_calc);
    end

    function setup_body_controls(b1,b2,b3,b4,b5,b6)
        set(b1,'CallBack',{@body_control,'init'});
        set(b2,'CallBack',{@body_control,'approach'});
        set(b3,'CallBack',{@body_control,'navigate'});
        set(b4,'CallBack',{@body_control,'teleop'});
        set(b5,'CallBack',{@body_control,'sideways'});
        set(b6,'CallBack',{@body_control,'sidedone'});
    end

    function setup_head_controls(b1,b2,b3,b4)
        CONTROL.head.fixed = b1;
        CONTROL.head.freelook = b2;
        CONTROL.head.qscan = b3;
        CONTROL.head.scan = b4;
        set(b1,'CallBack',{@head_control,'reset'});
        set(b2,'CallBack',{@head_control,'center'});
        set(b3,'CallBack',{@head_control,'tiltscan'});
        set(b4,'CallBack',{@head_control,'stepscan'});
    end

    function setup_arm_controls(b1,b2,b3,b4,b5)
        % standard
        set(b1,'CallBack',{@arm_control,'init'});
        set(b2,'CallBack',{@arm_control,'ready'});
        set(b3,'CallBack',{@arm_control,'reset'});
        % grab
        set(b4,'CallBack',{@arm_control,'grab'});
        set(b5,'CallBack',{@arm_control,'teleop'});
    end

%
    function set_waypoints(flags)
        if flags==1
            send_control_packet('BodyFSM','teleop');
        elseif flags==2
            waypoint=SLAM.get_waypoints();
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
            targetpos = LIDAR.get_single_approach();
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
            wp = LIDAR.get_double_approach();
            if numel(wp) == 0
                wp = SLAM.get_double_approach();
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
        SLAM.clear_waypoint();
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
