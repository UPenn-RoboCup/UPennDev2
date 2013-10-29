function ret = robotbody()
  global BODY POSE LIDAR SLAM
  % simple robot visualization code 
  % SJ 2013

  %Joint positions
  OFFSET_SHOULDER = [0,0.219,0.144];
  UARM_LENGTH = 0.246;
  ELBOW_OFFSET_X = 0.030;
  LARM_LENGTH = 0.242;
  OFFSET_HAND = [0.113 0.053 0];

  BODY=[];
  
  
  BODY.ubodysize = [0.20 0.40 0.180];
  BODY.lbodysize = [0.10 0.10 0.270];
  BODY.shouldersize = [0.08 0.14 0.14];
  BODY.uarmsize = [0.25 0.08 0.08];
  BODY.larmsize = [0.25 0.08 0.08];
  BODY.wristsize = [0.10 0.08 0.08];
  BODY.fingersize = [0.08 0.03 0.03];
  BODY.headsize = [0.15 0.10 0.15];

  BODY.ulegsize = [0.10 0.10 0.30];
  BODY.llegsize = [0.10 0.10 0.30];
  BODY.footsize = [0.26 0.145 0.03];


  BODY.waistoffset = [0 0 0.35];
  BODY.neckoffset = [0 0 0.40];
  BODY.lshoulderoffset = [0 0.216 0.162];
  BODY.rshoulderoffset = [0 -0.216 0.162];
  BODY.uarmoffset = [0.246 0 0.03];
  BODY.larmoffset = [0.242 0 -0.03];
  BODY.lfingeroffset = [0.08 -0.03 0];
  BODY.rfingeroffset = [0.08 0.03 0];
  BODY.finger2offset = [0.08 0.0 0];

  BODY.lhipoffset = [0 0.072 -0.270];
  BODY.rhipoffset = [0 -0.072 -0.270];
  BODY.kneeoffset = [-0.03 0 -0.30];
  BODY.ankleoffset = [0.03 0 -0.30];
  
  
  BODY.ubodycom = [0 0 0.09];
  BODY.lbodycom = [0 0 -0.135];
  BODY.headcom = [0 0 0];

  BODY.uarmcom = [0.12 0 0];
  BODY.larmcom = [0.12 0 -0.03];
  BODY.wristcom = [0.05 0 0];

  BODY.fingercom = [0.05 0 0];

  BODY.ulegcom = [0 0 -0.15];
  BODY.llegcom = [0.03 0 -0.15];
  BODY.footcom = [0.018 0 -0.015];

  BODY.neckangle=[0,0];
  BODY.waistangle=[0,0];
  BODY.larmangle=[0,0,0,0,0,0];
  BODY.rarmangle=[0,0,0,0,0,0];

  BODY.llegangle=[0,0,0,0,0,0];
  BODY.rlegangle=[0,0,0,0,0,0];

  BODY.lfingerangle = [0 0 0];
  BODY.rfingerangle = [0 0 0];

  BODY.verts=zeros(200,3);
  BODY.faces=zeros(200,3);
  BODY.vert_num = 0;
  BODY.face_num = 0;

  BODY.viewpoint = 1;

  BODY.update_angles = @update_angles;
  BODY.update = @update;
  BODY.init = @init;
  BODY.set_viewpoint = @set_viewpoint;

  BODY.rotate_view = @rotate_view;
  BODY.toggle_robot = @toggle_robot;
  BODY.mouselook = [0 0];
  BODY.draw_robot = 1;

  POSE = [];
  POSE.pose = [ 0 0 0];
  POSE.pose_odom = [ 0 0 0];
  POSE.pose_slam = [ 0 0 0];
  POSE.battery = 0;

  POSE.body_height = 0;
  POSE.rpy = [0 0 0];

  function set_viewpoint(h,~,flags)
    BODY.viewpoint = flags;
    calculate_transforms();
    plot_parts();
    BODY.mouselook= [0 0];
  end
  
  function rotate_view(movement)
    BODY.mouselook = BODY.mouselook + [movement(1) -movement(2)]/180*pi;
  end

  function toggle_robot(~,~)
    BODY.draw_robot = 1-BODY.draw_robot;
  end

  function update_viewpoint()
    flags = BODY.viewpoint;    
    pose = POSE.pose;

    look_angle = [0 pose(3)]+ BODY.mouselook;
    look_angle(2) = min(pi/2,max(-pi/2,look_angle(2)));
      
    robot_pos = [pose(1) pose(2) POSE.body_height];
    pose_dir = [cos(look_angle(2))*cos(look_angle(1)),...
                cos(look_angle(2))*sin(look_angle(1)),...
                sin(look_angle(2))];

    cam_dist1 = 20;

    if flags==1 

      set(BODY.p,'CameraTarget',robot_pos+ 2.5*pose_dir);
      set(BODY.p,'CameraPosition',...
				robot_pos +2.5*pose_dir+ -20*pose_dir);
      set(BODY.p,'XLim',[-3 3]+pose(1));
      set(BODY.p,'YLim',[-3 3]+pose(2));
      set(BODY.p,'ZLim',[-1 2]);
      camva(BODY.p,6.6)


    elseif flags==2 
      set(BODY.p,'CameraTarget',robot_pos + 2*pose_dir );
      set(BODY.p,'CameraPosition',robot_pos + 2*pose_dir + 18*[-1 -1 1]);
      set(BODY.p,'XLim',[-3 3]+pose(1)+2.5*pose_dir(1));
      set(BODY.p,'YLim',[-3 3]+pose(2)+2.5*pose_dir(2));
      set(BODY.p,'ZLim',[-1 2]);
      camva(BODY.p,6.6)


    elseif flags==3 %top view
      set(BODY.p,'CameraTarget',robot_pos+ 1.2*pose_dir);
      set(BODY.p,'CameraPosition',...
				robot_pos -2*pose_dir+[0 0 20]);
      set(BODY.p,'XLim',[-2.2 2.2]+pose(1));
      set(BODY.p,'YLim',[-2.2 2.2]+pose(2));
      set(BODY.p,'ZLim',[-1 2]);
      camva(BODY.p,6.6)


    elseif flags==4 %fist person view from head
      camva(BODY.p,70)
      set(BODY.p,'CameraTarget',robot_pos + 2*pose_dir + [0 0 0] );
      set(BODY.p,'CameraPosition',robot_pos + 0.2*pose_dir+ [0 0 0]);
      set(BODY.p,'XLim',[-3 3]+pose(1)+2.5*pose_dir(1));
      set(BODY.p,'YLim',[-3 3]+pose(2)+2.5*pose_dir(2));
      set(BODY.p,'ZLim',[-1 2]);

    end
  end

  function plot_mesh(pos1,pos2,pos3,pos4)
    BODY.verts(BODY.vert_num+1,:)=pos1;
    BODY.verts(BODY.vert_num+2,:)=pos2;
    BODY.verts(BODY.vert_num+3,:)=pos3;
    BODY.verts(BODY.vert_num+4,:)=pos4;
    BODY.faces(BODY.face_num+1,:)=[BODY.vert_num+1 BODY.vert_num+2 BODY.vert_num+3];
    BODY.faces(BODY.face_num+2,:)=[BODY.vert_num+2 BODY.vert_num+4 BODY.vert_num+3];
    BODY.cdatas(BODY.face_num+1,:)=[0.2 0.2 0.2];
    BODY.cdatas(BODY.face_num+2,:)=[0.2 0.2 0.2];
    BODY.vert_num = BODY.vert_num + 4;
    BODY.face_num = BODY.face_num + 2;
  end

  function plot_part(siz,offset,tr)
    tr1 = getpos(tr*trans(offset+[siz(1) siz(2) siz(3)]/2));
    tr2 = getpos(tr*trans(offset+[siz(1) siz(2) -siz(3)]/2));
    tr3 = getpos(tr*trans(offset+[siz(1) -siz(2) siz(3)]/2));
    tr4 = getpos(tr*trans(offset+[siz(1) -siz(2) -siz(3)]/2));
    tr5 = getpos(tr*trans(offset+[-siz(1) siz(2) siz(3)]/2));
    tr6 = getpos(tr*trans(offset+[-siz(1) siz(2) -siz(3)]/2));
    tr7 = getpos(tr*trans(offset+[-siz(1) -siz(2) siz(3)]/2));
    tr8 = getpos(tr*trans(offset+[-siz(1) -siz(2) -siz(3)]/2));

    plot_mesh(tr1,tr2,tr3,tr4);
    plot_mesh(tr5,tr6,tr7,tr8);
    plot_mesh(tr1,tr2,tr5,tr6);
    plot_mesh(tr3,tr4,tr7,tr8);
    plot_mesh(tr1,tr3,tr5,tr7);
    plot_mesh(tr2,tr4,tr6,tr8);
  end

  function plot_parts()
    BODY.vert_num = 0;
    BODY.face_num = 0;
    
    if BODY.draw_robot>0 
      plot_part(BODY.lbodysize, BODY.lbodycom, BODY.TrLBody);
      plot_part(BODY.ubodysize, BODY.ubodycom, BODY.TrUBody);
      plot_part(BODY.headsize, BODY.headcom, BODY.TrHead);

      plot_part(BODY.shouldersize, [0 0 0], BODY.TrLShoulder);
      plot_part(BODY.uarmsize, BODY.uarmcom, BODY.TrLUArm);
      plot_part(BODY.larmsize, BODY.larmcom, BODY.TrLLArm);
      plot_part(BODY.shouldersize, [0 0 0],  BODY.TrRShoulder);
      plot_part(BODY.uarmsize, BODY.uarmcom, BODY.TrRUArm);
      plot_part(BODY.larmsize, BODY.larmcom, BODY.TrRLArm);

      plot_part(BODY.wristsize, BODY.wristcom, BODY.TrLWrist);
      plot_part(BODY.wristsize, BODY.wristcom, BODY.TrRWrist);

      plot_part(BODY.fingersize, BODY.fingercom, BODY.TrLFinger11);
      plot_part(BODY.fingersize, BODY.fingercom, BODY.TrLFinger21);
      plot_part(BODY.fingersize, BODY.fingercom, BODY.TrLFinger12);
      plot_part(BODY.fingersize, BODY.fingercom, BODY.TrLFinger22);

      plot_part(BODY.fingersize, BODY.fingercom, BODY.TrRFinger11);
      plot_part(BODY.fingersize, BODY.fingercom, BODY.TrRFinger21);
      plot_part(BODY.fingersize, BODY.fingercom, BODY.TrRFinger12);
      plot_part(BODY.fingersize, BODY.fingercom, BODY.TrRFinger22);

      plot_part(BODY.ulegsize, BODY.ulegcom, BODY.TrRULeg);
      plot_part(BODY.ulegsize, BODY.ulegcom, BODY.TrLULeg);

      plot_part(BODY.llegsize, BODY.llegcom, BODY.TrRLLeg);
      plot_part(BODY.llegsize, BODY.llegcom, BODY.TrLLLeg);

      plot_part(BODY.footsize, BODY.footcom, BODY.TrRFoot);
      plot_part(BODY.footsize, BODY.footcom, BODY.TrLFoot);

      set(BODY.h,'Vertices',BODY.verts(1:BODY.vert_num,:));
      set(BODY.h,'Faces',BODY.faces(1:BODY.face_num,:));
      set(BODY.h,'FaceVertexCData',BODY.cdatas);
    else
      set(BODY.h,'Vertices',[]);
      set(BODY.h,'Faces',[]);
      set(BODY.h,'FaceVertexCData',[]);
    end

    update_viewpoint();

  end


  function calculate_transforms()
		pose = POSE.pose;

    lfingerangle1 = BODY.lfingerangle(1);
    lfingerangle2 = BODY.lfingerangle(2);

    rfingerangle1 = BODY.rfingerangle(1);
    rfingerangle2 = BODY.rfingerangle(2);

    BODY.TrLBody = eye(4)*trans([pose(1) pose(2) POSE.body_height])*rotZ(pose(3))*rotY(POSE.rpy(2));

    BODY.TrLULeg = BODY.TrLBody*trans(BODY.lhipoffset)*...
      rotZ(BODY.llegangle(1))*rotX(BODY.llegangle(2))*rotY(BODY.llegangle(3));
    BODY.TrLLLeg = BODY.TrLULeg*trans(BODY.kneeoffset)*rotY(BODY.llegangle(4));
    BODY.TrLFoot = BODY.TrLLLeg*trans(BODY.ankleoffset)*...
      rotY(BODY.llegangle(5))*rotX(BODY.llegangle(6));

    BODY.TrRULeg = BODY.TrLBody*trans(BODY.rhipoffset)*...
      rotZ(BODY.rlegangle(1))*rotX(BODY.rlegangle(2))*rotY(BODY.rlegangle(3));
    BODY.TrRLLeg = BODY.TrRULeg*trans(BODY.kneeoffset)*rotY(BODY.rlegangle(4));
    BODY.TrRFoot = BODY.TrRLLeg*trans(BODY.ankleoffset)*...
      rotY(BODY.rlegangle(5))*rotX(BODY.rlegangle(6));


    BODY.TrUBody = BODY.TrLBody*rotZ(BODY.waistangle(1))*rotY(BODY.waistangle(2));
    BODY.TrHead = BODY.TrUBody*trans(BODY.neckoffset)*...
    rotZ(BODY.neckangle(1))*rotY(BODY.neckangle(2));


    BODY.TrLShoulder = BODY.TrUBody*trans(BODY.lshoulderoffset)*...
		rotY(BODY.larmangle(1))*rotZ(BODY.larmangle(2)) ;
    BODY.TrLUArm = BODY.TrLShoulder*rotX(BODY.larmangle(3));
    BODY.TrLLArm = BODY.TrLUArm*trans(BODY.uarmoffset)*...
		rotY(BODY.larmangle(4));
    BODY.TrLWrist = BODY.TrLLArm*trans(BODY.larmoffset)*...
			rotX(BODY.larmangle(5))*rotZ(BODY.larmangle(6));
    BODY.TrLFinger11 = BODY.TrLWrist*trans(BODY.lfingeroffset)*...
			rotZ(lfingerangle1-pi/4);
    BODY.TrLFinger12 = BODY.TrLFinger11*trans(BODY.finger2offset)*...
			rotZ(lfingerangle2);
    BODY.TrLFinger21 = BODY.TrLWrist*trans(BODY.lfingeroffset)*...
			rotZ(-lfingerangle1-pi/4);
    BODY.TrLFinger22 = BODY.TrLFinger21*trans(BODY.finger2offset)*...
			rotZ(-lfingerangle2);

    BODY.TrRShoulder = BODY.TrUBody*trans(BODY.rshoulderoffset)*...
		rotY(BODY.rarmangle(1))*rotZ(BODY.rarmangle(2)) ;
    BODY.TrRUArm = BODY.TrRShoulder*rotX(BODY.rarmangle(3));
    BODY.TrRLArm = BODY.TrRUArm*trans(BODY.uarmoffset)*...
		rotY(BODY.rarmangle(4));
    BODY.TrRWrist = BODY.TrRLArm*trans(BODY.larmoffset)*...
			rotX(BODY.rarmangle(5))*rotZ(BODY.rarmangle(6));
    BODY.TrRFinger11 = BODY.TrRWrist*trans(BODY.rfingeroffset)*...
			rotZ(rfingerangle1+ pi/4);
    BODY.TrRFinger21 = BODY.TrRWrist*trans(BODY.rfingeroffset)*...
			rotZ(-rfingerangle1+pi/4);
    BODY.TrRFinger12 = BODY.TrRFinger11*trans(BODY.finger2offset)*...
			rotZ(rfingerangle2);
    BODY.TrRFinger22 = BODY.TrRFinger21*trans(BODY.finger2offset)*...
			rotZ(-rfingerangle2);

  end

  function init(a)
    axes(a);
    BODY.p = a;
    BODY.h = patch('FaceColor','flat','EdgeColor','none',...
     'AmbientStrength',0.4,'SpecularStrength',0.9 );
    light('Position',[0 3 3]);
	  set(a,'CameraViewAngleMode','manual');
    set(a,'DataAspectRatioMode','manual')

    set(BODY.p,'XLim',[-2 2]);
    set(BODY.p,'YLim',[-2 2]);
    set(BODY.p,'ZLim',[-1 2]);
    set(a,'XGrid','on')
    set(a,'YGrid','on')

    calculate_transforms();
    plot_parts();    
  end

  function setup_controls(b1,b2,b3,b4)

  end

  function update_angles(data)
    BODY.waistangle=double(data.waistangle);
    BODY.neckangle=double(data.neckangle);
    BODY.larmangle=double(data.larmangle);
    BODY.rarmangle=double(data.rarmangle);
    BODY.llegangle=double(data.llegangle);
    BODY.rlegangle=double(data.rlegangle);
    BODY.lfingerangle = double(data.lgrip);
    BODY.rfingerangle = double(data.rgrip);

    BODY.pose_odom = data.pose_odom;
    BODY.pose_slam = data.pose_slam;POSE.rpy = double(data.rpy);
    POSE.body_height = double(data.body_height);
    calculate_transforms();
    plot_parts();
  end

  function [ nBytes ]= update(fd)
    nBytes = 0;
    while udp_recv('getQueueSize',fd) > 0
      udp_data = udp_recv('receive',fd);
      nBytes = nBytes + numel(udp_data);
    end
    data = msgpack('unpack',udp_data);
    update_angles(data);    
    POSE.data = data;
		POSE.pose_odom = double(data.pose_odom);
		POSE.pose_slam = double(data.pose_slam);
		POSE.pose = double(data.pose);
    POSE.battery = double(data.battery);
    POSE.rpy = double(data.rpy);
    POSE.body_height = double(data.body_height);
    if isfield(SLAM,'update_pose')
      SLAM.update_pose(POSE.pose,POSE.pose_slam);
    end
  end

  ret= BODY;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Transform functions


  function ret = rotX(angle)
    ca=cos(angle); sa=sin(angle);
    ret = [1 0 0 0;0 ca -sa 0;0 sa ca 0; 0 0 0 1];
  end

  function ret = rotY(angle)
    ca=cos(angle); sa=sin(angle);
    ret = [ca 0 sa 0; 0 1 0 0; -sa 0 ca 0; 0 0 0 1];
  end

  function ret = rotZ(angle)
    ca=cos(angle); sa=sin(angle);
    ret = [ca -sa 0 0;sa ca 0 0; 0 0 1 0; 0 0 0 1];
  end

  function ret = trans(pos)
    ret = [1 0 0 pos(1);0 1 0 pos(2); 0 0 1 pos(3); 0 0 0 1];
  end

  function ret = getpos(tr)
    ret = [tr(1,4) tr(2,4) tr(3,4)];
  end

end


