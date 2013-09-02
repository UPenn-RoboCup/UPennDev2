function ret = controlbody()
global CONTROL SLAM BODY LIDAR POSE

CONTROL=[];

CONTROL.body=[];
CONTROL.head=[];
CONTROL.arm=[];
CONTROL.setup_slambody_controls = @setup_slambody_controls;
CONTROL.setup_robotbody_controls = @setup_robotbody_controls;
CONTROL.setup_lidarbody_controls = @setup_lidarbody_controls;

CONTROL.setup_body_controls = @setup_body_controls;
CONTROL.setup_head_controls = @setup_head_controls;
CONTROL.setup_arm_controls = @setup_arm_controls;
CONTROL.setup_fd = @setup_fd;

CONTROL.send_control_packet=@send_control_packet;


CONTROL.udp_send_id = -1;
ret = CONTROL;

    function setup_slambody_controls(b1,b2)
        set(b1,'CallBack',{SLAM.set_zoomlevel,1});
        set(b2,'CallBack',{SLAM.set_zoomlevel,2});
    end

    function setup_robotbody_controls(b1,b2,b3,b4)
        set(b1,'CallBack',{BODY.set_viewpoint,1});
        set(b2,'CallBack',{BODY.set_viewpoint,2});
        set(b3,'CallBack',{BODY.set_viewpoint,3});
        set(b4,'CallBack',{BODY.set_viewpoint,4});
    end

    function setup_lidarbody_controls(b1,b2, lmb1,lmb2,lmb3,lmb4,lmb5,lmb6,lmb7,lmb8)
        set(b1,'CallBack',{LIDAR.set_meshtype,1});
        set(b2,'CallBack',{LIDAR.set_meshtype,2});
        
        set(lmb1,'CallBack',{LIDAR.set_zoomlevel,1});
        set(lmb2,'CallBack',{LIDAR.set_zoomlevel,2});
        set(lmb3,'CallBack',{LIDAR.clear_points});
        set(lmb4,'CallBack',{LIDAR.set_img_display});
        set(lmb5,'CallBack',{LIDAR.get_depth_img});
        set(lmb6,'CallBack',LIDAR.wheel_calc);
        %set(lmb7,'CallBack',LIDAR.door_calc);
        %set(lmb8,'CallBack',LIDAR.tool_calc);
        
    end

    function setup_body_controls(b1,b2,b3,b4)
        CONTROL.body.stop = b1;
        CONTROL.body.teleop = b2;
        CONTROL.body.navigate = b3;
        CONTROL.body.wapproach = b4;
        set(b1,'CallBack',{@body_control,1});
        set(b2,'CallBack',{@body_control,2});
        set(b3,'CallBack',{@body_control,3});
        set(b4,'CallBack',{@body_control,4});
    end

    function setup_head_controls(b1,b2,b3,b4)
        CONTROL.head.fixed = b1;
        CONTROL.head.freelook = b2;
        CONTROL.head.qscan = b3;
        CONTROL.head.scan = b4;
        set(b1,'CallBack',{@head_control,1});
        set(b2,'CallBack',{@head_control,2});
        set(b3,'CallBack',{@head_control,3});
        set(b4,'CallBack',{@head_control,4});
    end

    function head_control(h,~,flags)
        if flags==1
            send_control_packet('head','fixed');
        elseif flags==2
            send_control_packet('head','teleop');
        elseif flags==3
            send_control_packet('head','scan');
        elseif flags==4
            send_control_packet('head','slowscan');
        end
    end

    function targetpos = transform_global(relpos)
        pose = POSE.pose;
        targetpos=[];
        targetpos(1) =pose(1) + cos(pose(3)) * relpos(1) - sin(pose(3))*relpos(2);
        targetpos(2) =pose(2) + sin(pose(3)) * relpos(1) + cos(pose(3))*relpos(2);
    end

    function body_control(h,~,flags)
        if flags==1
            %	    send_control_packet('body','stop');
            send_control_packet('body','teleop');
        elseif flags==2
            waypoint=SLAM.get_waypoints();
            if numel(waypoint)>0
                waypoint_num = size(waypoint,1);
                send_control_packet('body','navigate',reshape(waypoint,[1 2*waypoint_num]));
            end
        elseif flags==3
            disp('Single point approach')
            targetpos = LIDAR.get_single_approach();
            if numel(targetpos)>0
                send_control_packet('body','navigate',targetpos);
            end
        elseif flags==4  % Two-position approach
            disp('Two point approach')
            wp=LIDAR.get_double_approach();
            if numel(wp)>0
                send_control_packet('body','navigate',wp);
            end
        end
        SLAM.clear_waypoint();
    end

    function setup_arm_controls(b1,b2,b3,b4)
        CONTROL.arm.init  = b1;
        CONTROL.arm.grab  = b2;
        CONTROL.arm.reset = b3;
        CONTROL.arm.stop  = b4;
        
        % init
        set(b1,'CallBack',{@send_arm_event,'init'});
        set(b2,'CallBack',{@send_arm_event,'ready'});
        set(b3,'CallBack',{@send_arm_event,'wheelgrab'});
        set(b4,'CallBack',{@send_arm_event,'reset'});
    end

    function send_arm_event(~,~,evt)
        send_control_packet( 'ArmFSM', evt )
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