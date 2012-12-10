function h = shm_lidar(teamNumber, playerID)
% function create access to the skeleton information from the Kinect

h.teamNumber = teamNumber;
h.playerID = playerID;
h.user = getenv('USER');

% create shm wrappers
h.lidar  = shm(sprintf('rcmLidar%d%d%s',  h.teamNumber, h.playerID, h.user));
h.robot  = shm(sprintf('rcmRobot%d%d%s',  h.teamNumber, h.playerID, h.user));


% set function pointers
h.get_ranges = @get_ranges;
h.set_ranges = @set_ranges;

    function r = get_ranges(  )
        % returns the skeleton head
        r = [];
        try
            r.t = h.lidar.get_timestamp();
            r.odom = h.robot.get_odom();
            r.imu = h.robot.get_imu();
            r.gyro = h.robot.get_gyro();
            tmp = h.lidar.get_ranges();
            tmp = typecast(tmp,'single'); % 16bit precision
            r.ranges = tmp(1:1081);
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

