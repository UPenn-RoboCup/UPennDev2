% Starting timestamp
datestamp0 = '12.29.2014.17.50.39';
% datestamp0 = '12.05.2014.14.06.51';

prefix = 'mesh_m_';
nprefix = numel(prefix);
log_files0 = dir(strcat('Data/',prefix,'*.log'));
n0 = numel(log_files0);
for idx0=1:n0
    if strfind(log_files0(idx0).name, datestamp0)
        break;
    end
end
fprintf('Timestamp0: %s (Index0 %d of %d)\n', datestamp0, idx0, n0);
log_files = log_files0(idx0:end);

REAL_ROBOT = 0;
figure(1);

n = numel(log_files);
for idx=1:n
    datestamp = log_files(idx).name(nprefix+1:end-4);
    fprintf('Timestamp: %s (Index %d of %d)\n', datestamp, idx, n);
    % Depth
    fid = fopen(sprintf('Data/mesh_m_%s.log',datestamp));
    meshMeta = fread(fid, Inf, '*uint8');
    fclose(fid);
    meshMeta = msgpack('unpacker', meshMeta, 'uint8');
    
    % Grab the Depth logged information
    f_mesh = fopen(sprintf('Data/mesh_r_%s.log', datestamp));
    % Individual reading
    nlog = numel(meshMeta);
    for ilog=1:nlog
        meta = meshMeta{ilog};
        fprintf('Log %d of %d\n', ilog, nlog);
        n_scanlines = meta.dims(1);
        n_returns = meta.dims(2);
        s_angles = meta.a;
        s_pitch = meta.pitch;
        s_roll = meta.roll;
        s_pose = meta.pose;
        
        meshRaw = fread(f_mesh, [n_returns n_scanlines], '*single')';      
        
        % clamp on ranges
        meshRaw(meshRaw>5) = 0;
        meshRaw(meshRaw<0.1) = 0;
        
        rfov = floor(meta.rfov / pi * 180);
        v_angles = rfov(1):0.25:rfov(2);        
        v_angles = v_angles / 180 * pi;
        
        if length(v_angles)>n_returns
            v_angles = v_angles(1:n_returns);
        end
               
        % Convert to x, y, z
        xs0 = bsxfun(@times, cos(s_angles)', bsxfun(@times, meshRaw, cos(v_angles)));
        ys0 = bsxfun(@times, sin(s_angles)', bsxfun(@times, meshRaw, cos(v_angles)));
        zs0 = -1*bsxfun(@times, meshRaw, sin(v_angles)) + 0.1; %lidarX offset
                    
        xs = xs0; ys = ys0; zs = zs0;
        
        if REAL_ROBOT==1
            body_pitch_offset = -2.5/180*pi;
        else
            body_pitch_offset = 0;
        end
        
        pre_pose = s_pose(1);
        for i = 1:n_scanlines 
            % TODO: better factorization
            body_pitch = s_pitch(i) + body_pitch_offset;
            body_roll = s_roll(i);
            
            % A hack...
            if i<=length(s_pose) 
                cur_pose = s_pose(i) ;
                pre_pose = cur_pose;
            else 
                cur_pose = pre_pose;
            end
            
            
%             cur_pose = s_pose(i);
            cur_pose = cur_pose{1}; 
            
            rot_yaw = [cos(cur_pose(3)) -sin(cur_pose(3)); 
                      sin(cur_pose(3))  cos(cur_pose(3))];
            
            rot_pitch = [cos(body_pitch) sin(body_pitch); 
                      -sin(body_pitch)  cos(body_pitch)];
                  
            rot_roll = [cos(body_roll) -sin(body_roll); 
                      sin(body_roll)  cos(body_roll)];
                  
            
            new_xz = rot_pitch*[xs0(i,:); zs0(i,:)];
            new_yz = rot_roll*[ys0(i,:); new_xz(2,:)];
            new_xy = rot_yaw*[new_xz(1,:); new_yz(1,:)];
            
            
            % Now xs, ys, and zs are in GLOBAL coordinates
            xs(i,:) = new_xy(1,:) + cur_pose(1);
            ys(i,:) = new_xy(2,:) + cur_pose(2);
            zs(i,:) = new_yz(2,:) + 0.93;  %TODO: bodyHeight
              
            plot3(xs(i,:), ys(i,:), zs(i,:), '.');
            hold on;
        end
%         view([0 0]);
        hold off;
        
        pause(0.5);
    end
    fclose(f_mesh);
end

clear fid;
