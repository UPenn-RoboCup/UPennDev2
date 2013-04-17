clear all;
skeleton_s = zmq( 'subscribe',  'skeleton' );

%% Store the values in meters
x = zeros(15,1);
y = zeros(15,1);
z = zeros(15,1);

%% Setup the plot
p = plot(x,y,'r.');
xlabel('x');
ylabel('y');
axis([-1,1, -1, 2]);

while 1
    [data,idx] = zmq('poll',100);
    if numel(data)==1
        positions = msgpack('unpacker', data{1});
        % Save the values, and convert to meters
        for i=1:15
            x(i) = positions{i}.x/1000;
            y(i) = positions{i}.y/1000;
        end
        set(p, 'XData', x);
        set(p, 'YData', y);
        drawnow;
    end
end