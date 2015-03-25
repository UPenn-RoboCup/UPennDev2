function [mergedPlanes, mergedNumPlanes, mergedPoi, mergedP3d] = mergePlanes(Planes,numPlanes,Poi,P3d)

% limit the number of planes for possible slow-down

if numPlanes < 8
    
    mInd = [1:numPlanes];
    
    for j1_ = 1:numPlanes
        for j2_ = (j1_+1):numPlanes
            j1 = mInd(j1_);
            j2 = mInd(j2_);
            if ~isequal(j1,j2) && ~isempty(Planes{j1}) && ~isempty(Planes{j2})
                % test normal 
                if (abs(Planes{j1}.Normal'*Planes{j2}.Normal) > 0.95) &&  ( abs(Planes{j1}.Normal'*(Planes{j1}.Center - Planes{j2}.Center)) < 0.05 )
                        % merge 

                        Planes{j1}.Normal = (Planes{j1}.Size*Planes{j1}.Normal+Planes{j2}.Size*Planes{j2}.Normal)/(Planes{j1}.Size+Planes{j2}.Size);
                        Planes{j1}.Normal = Planes{j1}.Normal/norm(Planes{j1}.Normal);
                        Planes{j1}.Center = (Planes{j1}.Size*Planes{j1}.Center+Planes{j2}.Size*Planes{j2}.Center)/(Planes{j1}.Size+Planes{j2}.Size);

                        Planes{j1}.Size = Planes{j1}.Size+Planes{j2}.Size;
                        Planes{j1}.Points = [Planes{j1}.Points Planes{j2}.Points];
                        P3d{j1} = [P3d{j1} P3d{j2}];
                        mInd(j2) = j1;                    
                end
            end
        end
    end
    
    mInd = unique(mInd);
    mergedNumPlanes = numel(mInd);
    if mergedNumPlanes > 0
        mergedPoi = intersect(Poi, mInd);
        for j = 1:mergedNumPlanes  
            mergedPlanes{j} = Planes{mInd(j)};
        end
        
        if nargin==4
            for j = 1:mergedNumPlanes  
                mergedP3d{j} = P3d{mInd(j)};
            end
        end
    end
else

    mergedPlanes = Planes;
    mergedNumPlanes = numPlanes;
    mergedPot = Poi;

end

end