function [ connImg, ids, count_, indices ] = test4Connectivity( bw )

szImg = size(bw);
connImg = zeros(size(bw));
curID = 0;
count = [];
eqlabel = cell(1,1);
valid = [];
indices = cell(1,1);
flags__ = zeros(200,1);
for k=1:szImg(1)
    for j=1:szImg(2) 
        
        if bw(k,j) > 0
            idx = sub2ind(szImg,k,j);
            valid = [valid; idx];
            id = 0;
            % on the boundary along the x-axis
            if k==1
                if j > 1 && connImg(k,j-1) > 0
                    id =  connImg(k,j-1); 
                    connImg(k,j) = id;
                    count(id) = count(id)+1;
                else % j==1 || ( connImg(k,j-1)  == 0 for j > 1)
                    curID = curID + 1;
                    connImg(k,j) = curID;     
                    count(curID) = 1;
                    eqlabel{curID,1} = curID;                
                end                
            elseif j==1 % on the boundary along the y-axis
                if  connImg(k-1,j) > 0
                    id = connImg(k-1,j);
                    connImg(k,j) = id;
                    count(id) = count(id)+1;
                else
                    curID = curID + 1;
                    connImg(k,j) = curID;     
                    count(curID) = 1;
                    eqlabel{curID,1} = curID;                
                end
            else % here !!
                neighbors = [ connImg(k-1,j) connImg(k,j-1)];
                flags = [ (neighbors(1)>0)  (neighbors(2)>0)];
                if ~flags(1) && ~flags(2)
                    curID = curID + 1;
                    id = curID;
                    connImg(k,j) = curID;     
                    count(curID) = 1;
                    eqlabel{curID,1} = curID;         
                else
                    if  flags(1) && flags(2)
                        [id, who] = min(neighbors);   
                        connImg(k,j) = id;   
                        count(id) = count(id)+1;                        
                        if neighbors(1) ~= neighbors(2)
                            for t = 1:length(eqlabel{neighbors(1),1})
                                t_label = eqlabel{neighbors(1),1}(t);    
                                
                                if isempty(find(eqlabel{t_label,1}==neighbors(2)))
                                    eqlabel{t_label,1} = [eqlabel{t_label,1} neighbors(2)];
                                end
                                if isempty(find(eqlabel{t_label,1}==neighbors(1)))
                                    eqlabel{t_label,1} = [eqlabel{t_label,1} neighbors(1)];
                                end
                            end
                            for t = 1:length(eqlabel{neighbors(2),1})
                                t_label = eqlabel{neighbors(2),1}(t);         
                                if isempty(find(eqlabel{t_label,1}==neighbors(1)))
                                    eqlabel{t_label,1} = [eqlabel{t_label,1} neighbors(1)];
                                end
                                
                                if isempty(find(eqlabel{t_label,1}==neighbors(2)))
                                    eqlabel{t_label,1} = [eqlabel{t_label,1} neighbors(2)];
                                end
                            end
                            
                           eqlabel{neighbors(1),1} = unique([eqlabel{neighbors(1),1} eqlabel{neighbors(2),1}]);
                           eqlabel{neighbors(2),1} = eqlabel{neighbors(1),1};
                        else                       
                            if isempty(find(eqlabel{neighbors(1),1}==id))
                                eqlabel{neighbors(1),1} = [eqlabel{neighbors(1),1} id];
                            end
                        end
                    else
                        if flags(1)
                            id = neighbors(1);
                        else % if flags(2)
                            id = neighbors(2);
                        end
                        connImg(k,j) = id;   
                        count(id) = count(id)+1;
                    end  
                end
            end   
            
            if id > 0
                if length(flags__) < id
                    flags__(id) = 0;
                end
                if flags__(id) == 0                 
                    indices{id,1} = idx;
                    flags__(id) = 1;
                else
                    indices{id,1} = [indices{id,1} idx]; 
                end
            end
        end
    end
end

minlabel = zeros(size(eqlabel));
ids = -1;
cc = 0;
count_ = [];
for k=1:length(eqlabel)
    eqlabel{k,1} = sort(eqlabel{k,1});
    minlabel(k) = eqlabel{k,1}(1);
    if ids(end) < minlabel(k)
        cc = cc + 1;
        ids(cc) = minlabel(k);
        count_(cc) = sum(count(eqlabel{k,1}));       
        for t=2:length(eqlabel{k,1})
           
            if  length(indices) >= eqlabel{k,1}(t)
                indices{eqlabel{k,1}(1),1} = [indices{eqlabel{k,1}(1),1} indices{eqlabel{k,1}(t),1}]; 
            end
        end
    end
end

for k=1:length(valid)
    connImg(valid(k)) = minlabel(connImg(valid(k)));
end
    
end

