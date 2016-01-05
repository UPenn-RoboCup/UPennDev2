%% Read the RGB
logname = 'trial2';
logdir = strcat('Data/darwin/', logname, '/');
prefix = 'yuyv';
suffix = '.log';
d = dir(strcat(logdir, prefix, '_m_*', suffix));

close all;

h_f = figure(1);
clf;
h_a = gca;
h_img = image;
xlim([1, 160]);
ylim([1, 120]);
set(h_a, 'ydir', 'reverse');
hold on;
h_ball = plot(0, 0, 'go');
hold off;

t0 = -1;

%% Extra: plot the ball position
ball = [];
ball_t = [];
ball_head = [];
ball_rgb = {};

%% Loop
for i=1:numel(d)
    fname = d(i).name;
    fid = fopen(strcat(logdir, fname));
    msg = fread(fid, inf, '*uchar');
    fclose(fid);
    %%
    if numel(msg)>0
        objs = msgpack('unpacker', msg);
        clear msg fid;
        timestamp = fname(numel(prefix)+4:end-numel(suffix));
        fname = strcat(logdir, prefix, '_r_', timestamp, suffix);
        fid = fopen(fname);
        for o=1:numel(objs)
            metadata = objs{o};
            if t0 < 0
                t0 = metadata.t;
            end
            
            yuyv = fread(fid, metadata.rsz, '*uint8');
            if numel(yuyv) == metadata.rsz
                % YUYV to RGB
                yuyv_u8 = reshape(yuyv, [4, metadata.w/2, metadata.h]);
                ycbcr = yuyv_u8([1 2 4], :, 1:2:end);
                ycbcr = permute(ycbcr, [3 2 1]);
                rgb = ycbcr2rgb(ycbcr);
                % Show image
                set(h_img,'cdata', rgb);
            end
            %if metadata.t - t0 > 33.5 && metadata.t - t0 < 36.5
            if exist('metadata.ball.detect', 'var') && metadata.ball.detect>0
                %ball_t = [ball_t; metadata.t-t0, metadata.ball.t - t0];
                ball_t = [ball_t; metadata.t-t0];
                ball = [ball; metadata.ball.v];
                ball_head = [ball_head; metadata.head];
                ball_rgb{numel(ball_rgb)+1} = rgb;
                % Show ball
                set(h_ball, 'XData', metadata.ball.propsA.centroid(1) + 1);
                set(h_ball, 'YData', metadata.ball.propsA.centroid(2) + 1);
                set(h_ball, 'MarkerSize', 2*metadata.ball.propsA.axisMajor);
                title(h_a,sprintf('Log %d:%d | Head (%3.1f, %3.1f) | Ball (%.2f %.2f)', ...
                    i, o, ...
                    rad2deg(metadata.head), ...
                    metadata.ball.v(1), metadata.ball.v(2) )...
                    );
                %pause(1/30);
                %pause(1/2);
            else
                title(h_a, sprintf('Log %d:%d', i, o));
                set(h_ball, 'XData', []);
                set(h_ball, 'YData', []);
                %pause(1/60);
            end
            %if i==4 && o==9, return; end
            pause(1/30);
        end
        fclose(fid);
    end
end

%% Save
save(strcat('Data/darwin/',logname,'.mat'), ...
    'ball', 'ball_t', 'ball_head', 'ball_rgb');
