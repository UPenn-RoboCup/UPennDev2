clear all;
s_laser = zmq('subscribe',5555);
s_imu = zmq('subscribe',5556);
t_last_laser = 0;
t_last_imu = 0;
while 1
    [data,idx] = zmq('poll',1000);
    if numel(idx)==0
        break;
    end
    for i=1:numel(idx)
        if idx(i)==s_laser
            laser = msgpack('unpack', data{i});
            fprintf('Laser FPS: %f\n',1/(laser.t-t_last_laser) );
            t_last_laser = laser.t;
        else
            imu = msgpack('unpack', data{i});
            %fprintf('IMU FPS: %f\n',1/(imu.t-t_last_imu) );
            t_last_imu = imu.t;
        end
    end
end