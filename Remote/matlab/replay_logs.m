% Add mex files for the msgpack functionality
clear all;
addpath(genpath('.'));

%% Reading logs
% Open head logs
fid = fopen('head_05.20.2013.06.55.log');
msg = fread(fid,'*uint8');
fclose(fid);
head = msgpack('unpacker', msg);
clear fid msg;

% Open chest logs
fid = fopen('chest_05.20.2013.06.48.log');
msg = fread(fid,'*uint8');
fclose(fid);
chest = msgpack('unpacker', msg);
clear fid msg;

%% Plotting
% % Plot the chest lidar data
% hf_chest = figure(1);
% hp_chest = plot(zeros(1081,1));
% ha_chest = gca;
% ht_chest = title(ha_chest,'Chest');
% for i=1:numel(chest)
%     ranges = typecast(chest{i}.ranges,'single');
%     chest_angle = chest{i}.joints{36};
%     set(hp_chest,'YData',ranges);
%     set(ht_chest,'String',sprintf('Chest: %.3f Degrees',chest_angle) );
%     drawnow;
% end

% Plot the chest lidar data
hf_head = figure(2);
hp_head = plot(zeros(1081,1));
ha_head = gca;
ht_head = title(ha_head,'Head');
for i=1:numel(head)
    ranges = typecast(head{i}.ranges,'single');
    head_angle = head{i}.joints{28}; %Pitch
    set(hp_head,'YData',ranges);
    set(ht_head,'String',sprintf('Head: %.3f Degrees',head_angle) );
    drawnow;
end

%% Cleanup
clear i ranges;