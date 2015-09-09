% Number of null space dimensions
nNull = 2;

%% Filter the nullspace in time
gamma = 0.5;
Ns = cell(size(nulls));
% Torch comes in transposed!
Ns{1} = nulls{1}';
for i=2:numel(nulls)
    Ns{i} = (1-gamma)*Ns{i-1} + gamma*nulls{i}';
end

%% Save the SVD and form the null space coordinates
Ss = zeros(np, nq);
lambda = zeros(np, nNull);
Vs = cell(np, 1);
Us = cell(np, 1);

for i=1:numel(Ns)
    [U, S, V] = svd(Ns{i});
    Vs{i} = V(:, 1:nNull);
    Us{i} = U(:, 1:nNull);
    Ss(i, :) = diag(S);
    lambda(i, :) = S(1:nNull, 1:nNull) * V(:, 1:nNull)' * (qwPath{i} - qwPath{end});
end

%% Determine the signs from the SVD for consistent basis directions
% NOTE: This should be slow, but not *too* bad ;)
swapidx = [1];
for iN=1:nNull
    for i=2:numel(nulls)
        dirlambda = dot(Vs{i}(:, iN), Vs{i-1}(:, iN));
        if abs(dirlambda) > 0.9 % Pretty much the same dir...
            if dirlambda < 0
                Vs{i}(:, iN) = -Vs{i}(:, iN);
                Us{i}(:, iN) = -Us{i}(:, iN);
                lambda(i, iN) = -lambda(i, iN);
            end
        else
            swapidx = [swapidx, i];
        end
    end
end
clear dirlambda;
swapidx = [swapidx, numel(nulls)];
swapidx = sort(unique(swapidx));

%% Now on the swaps, check the directions
for i=2:numel(swapidx)-1
    for iN=1:nNull-1
        A1 = Vs{swapidx(i)-1}(:, iN);
        A2 = Vs{swapidx(i)}(:, iN);
        B1 = Vs{swapidx(i)-1}(:, iN+1);
        B2 = Vs{swapidx(i)}(:, iN+1);
        dirswap1 = dot(A1, B2);
        dirswap2 = dot(B1, A2);
        if abs(dirswap1)>0.9 && abs(dirswap1)>0.9
            % Swap the two!
            disp([iN, i, swapidx(i), dirswap1, dirswap2]);
            if dirswap1<-0.9 || dirswap2<-0.9
                % Also change the direction...
                for ir=swapidx(i):swapidx(i+1)
                    Vs{ir}(:, iN:iN+1) = -Vs{ir}(:, iN:iN+1);
                    Us{ir}(:, iN:iN+1) = -Us{ir}(:, iN:iN+1);
                    lambda(ir, iN:iN+1) = -lambda(ir, iN:iN+1);
                end
            end
        end
    end
end
    figure(12);
    clf;
    hold on;
    cmap = hsv(numel(swapidx)-1);
    for i=1:numel(swapidx)-1
        range = swapidx(i):swapidx(i+1);
        for iN=1:nNull
            plot(t(range), lambda(range, iN), '-s', 'Color', cmap(i,:));
        end
    end
    hold off;
    xlim(tlim);
    xlabel('Time (s)');
    title('Original lambda [MATLAB]');

%% Run the optimization for each dimension...?
ddlambda = zeros(size(lambda));
for i=1:numel(swapidx)-1
    range = swapidx(i):swapidx(i+1);
    ddlambda(range, :) = subopt_lambda(lambda(range, :));
end

%% Run as normal
dlambda = subopt_lambda(lambda);

%% Change back into q
qLambda = zeros(np, nq);
dqLambda = zeros(np, nq);
for i=1:size(lambda, 1)
    % TODO: Us should be a cell array and then choose nNull columns
    dqLambda(i, :) = Us{i} * (ddlambda(i, :) - lambda(i, :))';
    qLambda(i, :) = qwPath{i} + dqLambda(i, :)';
end