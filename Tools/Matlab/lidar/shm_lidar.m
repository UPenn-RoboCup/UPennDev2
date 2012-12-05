function h = shm_lidar(teamNumber, playerID)
% function create access to the skeleton information from the Kinect

h.teamNumber = teamNumber;
h.playerID = playerID;
h.user = getenv('USER');

% create shm wrappers
h.lidar  = shm(sprintf('rcmLidar%d%d%s',  h.teamNumber, h.playerID, h.user));


% set function pointers
h.get_ranges = @get_ranges;
h.set_ranges = @set_ranges;

    function r = get_ranges(  )
        % returns the skeleton head
        r = [];
        try
            r.t = h.lidar.get_timestamp();
            r.odom = h.lidar.get_odom();
            tmp = h.lidar.get_ranges();
            r.ranges = typecast(tmp,'single'); % 16bit precision
        catch
            fprintf('Illegal LIDAR get!');
            r = [];
        end
    end

    function r = set_ranges( myranges, mytimestamp )
        % returns the skeleton head
        r = [];
        try 
            h.lidar.set_timestamp(mytimestamp);
            h.lidar.set_ranges(myranges);
        catch
            fprintf('Illegal LIDAR set!');
            r = [];
        end
    end

end

