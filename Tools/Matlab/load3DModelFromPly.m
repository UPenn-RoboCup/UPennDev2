function [ Pts ] = load3DModelFromPly( datasetname )

Pts = [];

if strcmp(datasetname{1},'S_Hinterstoisser')
    folder = strcat('~/Data/',datasetname{1}, '/',datasetname{2},'/');
    filename = strcat(folder,'mesh.ply');
    
    Pts = ply_points(filename);

end

end

