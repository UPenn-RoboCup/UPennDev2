function [ pts ] = findVertSidePoints( L , center_s, id, bbox )

h_ = size(L,1);
centerind = center_s(1)*h_ + center_s(2);
% vertical-side points 
vrange = [min(bbox(:,2)) max(bbox(:,2))];
hrange = [min(bbox(:,1)) min(bbox(:,1))];
pts = [];

%%%%% 
% Let's use getmapcell from ray

t_ind1 = find(L(:,center_s(1))==id);

if ~isempty(t_ind1)
    % direction 3-7
    t_ind3 = find(L(center_s(2),:)==id);     
    if  ~isempty(t_ind3)
        idx_2 = centerind:(h_-1):numel(L);
        idx_6 = centerind:(1-h_):h_*max((center_s(1)-(h_-center_s(2))),1);
        idx_4 = centerind:(h_+1):numel(L);
        idx_8 = centerind:(-1-h_):h_*max((center_s(1)-(h_-center_s(2))),1);        
        t_ind2 = find(L(idx_2)==id,1,'last');                              
        t_ind6 = find(L(idx_6)==id,1,'last');  
        t_ind4 = find(L(idx_4)==id,1,'last');  
        t_ind8 = find(L(idx_8)==id,1,'last');

        if  ~isempty(t_ind2) && ~isempty(t_ind6) && ~isempty(t_ind4) && ~isempty(t_ind8)
            pts(1,:) = [min(t_ind1) center_s(1)];
            pts(5,:) = [max(t_ind1) center_s(1)];
            pts(3,:) = [center_s(2) min(t_ind3)];
            pts(7,:) = [center_s(2) max(t_ind3)];
            % direction 2-6 figure(2),

            [t_1, t_2] = ind2sub(size(L),idx_2(t_ind2)); 
            pts(2,:) = [t_1, t_2] ;
            [t_1, t_2] = ind2sub(size(L),idx_6(t_ind6)); 
            pts(6,:) =  [t_1, t_2];                          
            % direction 4-8
            [t_1, t_2] = ind2sub(size(L),idx_4(t_ind4)); 
            pts(4,:) = [t_1, t_2] ;                         
            [t_1, t_2] = ind2sub(size(L),idx_8(t_ind8)); 
            pts(8,:) =  [t_1, t_2];   
        end
    end
end

end

