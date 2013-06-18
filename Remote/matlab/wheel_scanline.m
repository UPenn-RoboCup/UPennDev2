function wheel_scanline(ret)
  global HEAD_LIDAR

  tic;
  xposa= ret.xpos;
  lpos=[];rpos=[];
  tpos=[];bpos=[];

size(xposa)


  %Horizontal scan
  for i=1:size(xposa,1)
    xpos = xposa(i,:);
    lr = wheel_scanline_match(xpos);
    if numel(lr)>0
      lpos = [lpos; [ret.xpos(i,lr(1)) ret.ypos(i,lr(1)) ret.zpos(i,lr(1))]];
      rpos = [rpos; [ret.xpos(i,lr(2)) ret.ypos(i,lr(2)) ret.zpos(i,lr(2))]];
    end
  end

  %Vertical scan
  for i=1:size(xposa,2)
    xpos = xposa(:,i)';
    lr = wheel_scanline_match(xpos);
    if numel(lr)>0
      tpos = [tpos; [ret.xpos(lr(1),i) ret.ypos(lr(1),i) ret.zpos(lr(1),i)]];
      bpos = [bpos; [ret.xpos(lr(2),i) ret.ypos(lr(2),i) ret.zpos(lr(2),i)]];
    end
  end

  %Kill outliers
  dist_th = 0.02;

  dist_th = 0.01;


  lpos2 = [];rpos2=[];tpos2=[];bpos2=[];

  for i=2:size(lpos,1)-1
    xyz0 = lpos(i-1,:);
    xyz1 = lpos(i,:);
    xyz2 = lpos(i+1,:);
    dist0 = norm(xyz0-xyz1);
    dist1 = norm(xyz2-xyz1);
    dist2 = norm(xyz2-xyz0);
    if dist0<dist_th && dist1<dist_th && dist2<2*dist_th
      lpos2 =[lpos2;lpos(i,:)];
    end
  end
  for i=2:size(rpos,1)-1
    xyz0 = rpos(i-1,:);
    xyz1 = rpos(i,:);
    xyz2 = rpos(i+1,:);
    dist0 = norm(xyz0-xyz1);
    dist1 = norm(xyz2-xyz1);
    dist2 = norm(xyz2-xyz0);
    if dist0<dist_th && dist1<dist_th && dist2<2*dist_th
      rpos2 =[rpos2;rpos(i,:)];
    end
  end

  for i=2:size(tpos,1)-1
    xyz0 = tpos(i-1,:);
    xyz1 = tpos(i,:);
    xyz2 = tpos(i+1,:);
    dist0 = norm(xyz0-xyz1);
    dist1 = norm(xyz2-xyz1);
    dist2 = norm(xyz2-xyz0);
    if dist0<dist_th && dist1<dist_th && dist2<2*dist_th
      tpos2 =[tpos2;tpos(i,:)];
    end
  end
  for i=2:size(bpos,1)-1
    xyz0 = bpos(i-1,:);
    xyz1 = bpos(i,:);
    xyz2 = bpos(i+1,:);
    dist0 = norm(xyz0-xyz1);
    dist1 = norm(xyz2-xyz1);
    dist2 = norm(xyz2-xyz0);
    if dist0<dist_th && dist1<dist_th && dist2<2*dist_th
      bpos2 =[bpos2;bpos(i,:)];
    end
  end


%{
  figure(2);
  clf;
  hold on;
  plot3(lpos2(:,1),lpos2(:,2),lpos2(:,3));
  axis equal
pause;
  plot3(rpos2(:,1),rpos2(:,2),rpos2(:,3));
  axis equal
pause;

  clf;
  hold on;
  plot3(tpos2(:,1),tpos2(:,2),tpos2(:,3));
  axis equal
pause;

  plot3(bpos2(:,1),bpos2(:,2),bpos2(:,3));
  axis equal

pause;
%}








  posxyz = [lpos2;rpos2]; %LR
  posxyztb = [tpos2;bpos2]; %TB


  %Generate initial estimate for parameters
  [max_x i_max_x] = max(posxyz(:,1));
  [min_x i_min_x] = min(posxyz(:,1));
  [max_y i_max_y] = max(posxyz(:,2));
  [min_y i_min_y] = min(posxyz(:,2));
  [max_z i_max_z] = max(posxyz(:,3));
  [min_z i_min_z] = min(posxyz(:,3));

  xyz_top = posxyz(i_max_z,:);
  xyz_bottom = posxyz(i_min_z,:);
  xyz_left = posxyz(i_max_y,:);
  xyz_right = posxyz(i_min_y,:);

  [max_x_tb i_max_x_tb] = max(posxyztb(:,1));
  [min_x_tb i_min_x_tb] = min(posxyztb(:,1));
  [max_y_tb i_max_y_tb] = max(posxyztb(:,2));
  [min_y_tb i_min_y_tb] = min(posxyztb(:,2));
  [max_z_tb i_max_z_tb] = max(posxyztb(:,3));
  [min_z_tb i_min_z_tb] = min(posxyztb(:,3));

  xyz_top_tb = posxyztb(i_max_z_tb,:);
  xyz_bottom_tb = posxyztb(i_min_z_tb,:);
  xyz_left_tb = posxyztb(i_max_y_tb,:);
  xyz_right_tb = posxyztb(i_min_y_tb,:);


%{
  wheel_xyz = 0.5*[(min_x+max_x) (min_y+max_y) (min_z+max_z)];
  wheel_radius = .25*( norm(xyz_top-xyz_bottom)+ norm(xyz_left-xyz_right)  );
  wheel_yaw = atan2(xyz_left(2)-xyz_right(2),xyz_left(1)-xyz_right(1))-pi/2;
  wheel_pitch = -(atan2(xyz_top(3)-xyz_bottom(3),xyz_top(1)-xyz_bottom(1))-pi/2);
%}




  wheel_xyz = 0.25*(xyz_left+xyz_right+xyz_top_tb+xyz_bottom_tb);
  wheel_radius = .25*( norm(xyz_top_tb-xyz_bottom_tb)+ norm(xyz_left-xyz_right)  );
  wheel_yaw = atan2(xyz_left(2)-xyz_right(2),xyz_left(1)-xyz_right(1))-pi/2;
  wheel_pitch = -(atan2(xyz_top_tb(3)-xyz_bottom_tb(3),xyz_top_tb(1)-xyz_bottom_tb(1))-pi/2);


%{
  wheel_xyz
  wheel_radius
  wheel_yaw_deg=wheel_yaw*180/pi
  wheel_pitch_deg=wheel_pitch*180/pi

  toc;

  tic;
  param0=[wheel_xyz(1),wheel_xyz(2),wheel_xyz(3),...
				wheel_yaw,wheel_pitch,wheel_radius];

  err0 = evaluate(param0,posxyz)
  param1 = gdescent(param0,[0.002 0.002 0.002 pi/180 pi/180 0.005],posxyz);

  err1 = evaluate(param1,posxyz)
  toc;

  wheel_xyz=param1(1:3);
  wheel_yaw=param1(4);
  wheel_pitch=param1(5);
  wheel_radius=param1(6);

%  wheel_xyz=[0.39 0.05 0.14];
%  wheel_yaw=-0.2*pi/180;
%  wheel_pitch = 12.9*pi/180;

%}

  wheel_xyz
  wheel_radius
  wheel_yaw_deg=wheel_yaw*180/pi
  wheel_pitch_deg=wheel_pitch*180/pi







  t=[0:pi/180;2*pi];
  xyz_guess=[];
  for t=0:pi/180:2*pi
    end_p = trans(wheel_xyz)*rotZ(wheel_yaw)*rotY(wheel_pitch)...
				*trans([0 cos(t) sin(t)]*wheel_radius);
    end_pos = getpos(end_p);
    xyz_guess = [xyz_guess;end_pos];
  end



  figure(2);
  clf;
  hold on;
  plot3(lpos2(:,1),lpos2(:,2),lpos2(:,3));
  plot3(rpos2(:,1),rpos2(:,2),rpos2(:,3));
  plot3(xyz_guess(:,1),xyz_guess(:,2),xyz_guess(:,3),'r');
  hold off;
  axis equal;


  t0 = 0;
  t1 = pi/2;
  t2 = pi;
  t3= pi*3/2; 
  end_p0 = getpos(trans(wheel_xyz)*rotZ(wheel_yaw)*rotY(wheel_pitch)...
 			    	*trans([0 cos(t0) sin(t0)]*wheel_radius));
  end_p1 = getpos(trans(wheel_xyz)*rotZ(wheel_yaw)*rotY(wheel_pitch)...
 			    	*trans([0 cos(t1) sin(t1)]*wheel_radius));
  end_p2 = getpos(trans(wheel_xyz)*rotZ(wheel_yaw)*rotY(wheel_pitch)...
 			    	*trans([0 cos(t2) sin(t2)]*wheel_radius));
  end_p3 = getpos(trans(wheel_xyz)*rotZ(wheel_yaw)*rotY(wheel_pitch)...
 			    	*trans([0 cos(t3) sin(t3)]*wheel_radius));


  





%  function ret = evaluate(xyz,yaw,pitch,radius)
  function ret = evaluate(param,pos)
    xyz = param(1:3);
    yaw = param(4);
    pitch = param(5);
    radius = param(6);
    inv_tr = rotY(-pitch)*rotZ(-yaw)*trans(-xyz);
    sum_error = 0;
    for i=1:size(pos,1)
      inv_p = getpos(inv_tr*trans(pos(i,:)));
      d_error = min(0.01,abs(inv_p(1))    );
      r_error = min(0.01,abs(radius-sqrt(inv_p(2)^2+inv_p(3)^2))   );
      if d_error==0.01 d_error = 0.5;end
      if r_error==0.01 r_error = 0.2;end
      sum_error = d_error + r_error;
    end
    ret=sum_error;
  end

  function ret = gdescent(p0,p_var,pos)
    p_old =  p0;
    err_old = evaluate(p0,pos);
    for i=1:200
      p_new = p_old + p_var .*(rand(size(p_var))-.5);
%hack
	    p_new([2,3,6]) = p_old([2,3,6]);

      err_new = evaluate(p_new,pos);
      if err_new<err_old 
        p_old = p_new;
        err_old = err_new;
      end
    end
    ret = p_old;
  end


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
