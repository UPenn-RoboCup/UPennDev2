function h = shm_primesense()
% function create access to the skeleton information from the Kinect

h.teamNumber = 18;
h.playerID = 1;
h.user = getenv('USER');

% create shm wrappers
h.confidence  = shm(sprintf('primecmConfidence%d%d%s',  h.teamNumber, h.playerID, h.user));
h.orientation = shm(sprintf('primecmOrientation%d%d%s', h.teamNumber, h.playerID, h.user));
h.position  = shm(sprintf('primecmPosition%d%d%s',  h.teamNumber, h.playerID, h.user));
h.skeleton  = shm(sprintf('primecmSkeleton%d%d%s',  h.teamNumber, h.playerID, h.user));

% set function pointers
h.get_joint = @get_joint;

    function r = get_joint( jointName )
        % returns the skeleton head
        r = [];
        try
            r.t = h.skeleton.get_timestamp();
            % Super bad for performance, I think, 
            % but don't know a better way...
            r.confidence = eval( strcat('feval( h.confidence.get_',jointName,' )') );
            orientation = eval( strcat('feval( h.orientation.get_',jointName,' )') );
            r.rot = reshape( orientation, [3 3] );
            r.position = eval( strcat('feval( h.position.get_',jointName,' )') );
        catch
            fprintf('Illegal get! %s',jointName);
            r = [];
        end
    end

end

