function [ qLambda, dt_opt, lambda_opt ] = ...
    optimize_armplan_lambda(qwPath0, vwPath0, nullPath0, jacobianPath0, qwStar)

nExtraNull = 0;

%% Reshape the path for optimization
% Number of trajectory points
np = size(qwPath0, 1);
% Number of joints
nq = size(qwPath0, 2);
% Number of null space projections to optimize
nNull = nq - 6 + nExtraNull;

%% Filter the nullspace in time
gamma = 1;
Ns = cell(np);
% Torch comes in transposed!
Ns{1} = nullPath0{1}';
for i=2:np
    Ns{i} = (1-gamma) * Ns{i-1} + gamma * nullPath0{i}';
end

Ns = cell(np);
for i=1:np
    % MATLAB/torch transpose...
    JTJ = pinv(jacobianPath0{i}') * jacobianPath0{i}';
    Ns{i} = eye(size(JTJ, 1)) - JTJ;
end
clear JTJ;

%% Save the SVD and form the null space coordinates
Ss = zeros(np, nq);
lambda = zeros(np, nNull);
Vs = cell(np, 1);
Us = cell(np, 1);
for i=1:np
    [U, S, V] = svd(Ns{i});
    Vs{i} = V(:, 1:nNull);
    Us{i} = U(:, 1:nNull);
    Ss(i, :) = diag(S);
    lambda(i, :) = S(1:nNull, 1:nNull) * V(:, 1:nNull)' * ...
        (qwPath0(i, :) - qwStar)';
end

%% Filter on the separation of singular values
ds = ones(np, 1);
dSs = diff(Ss, 1, 2);
dsTrack = var(dSs) > 0.01;
dSsImportant = -dSs(:,dsTrack);
if numel(dSsImportant)>0
    ds0 = min(dSsImportant, [], 2);
    ds = conv(ds0.^2, [1,2,1], 'same');
    ds = ds / max(ds);
end

%% Determine the signs from the SVD for consistent basis directions
% NOTE: This should be slow, but not *too* bad ;)
swapidx = [1];
for iN=1:nNull
    for i=2:np
        dirlambda = dot(Vs{i-1}(:, iN), Vs{i}(:, iN));
        %fprintf(1, '%d: %.2f: %.4f\n', iN, 0, dirlambda);
        if abs(dirlambda) > 0.91 % Pretty much the same dir...
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
swapidx = [swapidx, np];
swapidx = sort(unique(swapidx));

%% Now on the swaps, check the directions
segment = true(size(swapidx));
for i=2:numel(swapidx)-1
    for iN=1:nNull-1
        is = swapidx(i);
        A1 = Vs{is-1}(:, iN);
        A2 = Vs{is}(:, iN);
        B1 = Vs{is-1}(:, iN+1);
        B2 = Vs{is}(:, iN+1);
        dirswap1 = dot(A1, B2);
        dirswap2 = dot(B1, A2);
        dirswapA = dot(A1, A2);
        dirswapB = dot(B1, B2);
        %fprintf(1, '[%d]: %.2f {%.2f, %.2f}, {%.2f, %.2f}\n', ...
        %    iN, 0, dirswap1, dirswap2, dirswapA, dirswapB);
        %if abs(dirswap1)>0.9 && abs(dirswap2)>0.9
        if abs(dirswap1)>abs(dirswapA) || abs(dirswap2)>abs(dirswapB) ...
                || abs(dirswap1)>abs(dirswapB) || abs(dirswap2)>abs(dirswapA)
            % Swap the two!
            fprintf(1, '!! Swapidx %d: %d\n', i, is);
            segment(i) = 0;
            FLIP_DIR1 = 1;
            FLIP_DIR2 = 1;
            if dirswap1<0, FLIP_DIR1 = -1; end
            if dirswap2<0, FLIP_DIR2 = -1; end
            
            % Do the flipping
            for ir=swapidx(i-1):is-1
                Vnow = FLIP_DIR1*Vs{ir}(:, iN);
                Vs{ir}(:, iN) = FLIP_DIR2*Vs{ir}(:, iN+1);
                Vs{ir}(:, iN+1) = Vnow;
                Unow = FLIP_DIR1*Us{ir}(:, iN);
                Us{ir}(:, iN) = FLIP_DIR2*Us{ir}(:, iN+1);
                Us{ir}(:, iN+1) = Unow;
                Lnow = FLIP_DIR1*lambda(ir, iN);
                lambda(ir, iN) = FLIP_DIR2*lambda(ir, iN+1);
                lambda(ir, iN+1) = Lnow;
            end
        elseif dirswapA < 0
            fprintf(1, '!! SwapA %d: %d (iN: %d)\n', i, is, iN);
            for ir=swapidx(i-1):is-1
                Vs{ir}(:, iN) = -Vs{ir}(:, iN);
                Us{ir}(:, iN) = -Us{ir}(:, iN);
                lambda(ir, iN) = -lambda(ir, iN);
            end
            segment(i) = 0;
        elseif dirswapB < 0
            fprintf(1, '!! SwapB %d: %d (iN: %d)\n', i, is, iN);
            for ir=swapidx(i-1):is-1
                Vs{ir}(:, iN+1) = -Vs{ir}(:, iN+1);
                Us{ir}(:, iN+1) = -Us{ir}(:, iN+1);
                lambda(ir, iN+1) = -lambda(ir, iN+1);
            end
            segment(i) = 0;
        end
    end
end

%% Run the optimization for each dimension...?
clear ddlambda dlambda dqLambda qLambda;
swapidx0 = swapidx;
swapidx = swapidx(segment);
%%{
disp('Optimizing ddlambda!');
ddlambda = lambda;
dt_opt = [];
lambda_opt = [];
for i=1:numel(swapidx)-1
    range = swapidx(i):swapidx(i+1)-1;
    % Only if enough points
    if numel(range)>5
        fprintf(1, 'Optimizing %d\n', numel(range));
        [ddlambda(range, :), dt_opt_lambda, opt_val] = subopt_lambda(lambda(range, :), ds);
        dt_opt = [dt_opt; dt_opt_lambda];
        lambda_opt = [lambda_opt, opt_val];
    end
end

%% Run as normal
%disp('Optimizing dlambda!');
%dlambda = subopt_lambda(lambda, ds);

%% Change back into q
qLambda = zeros(np, nq);
dqLambda = zeros(np, nq);
for i=1:size(lambda, 1)
    % TODO: Us should be a cell array and then choose nNull columns
    dqLambda(i, :) = Us{i} * (ddlambda(i, :) - lambda(i, :))';
    qLambda(i, :) = qwPath0(i, :) + dqLambda(i, :);
end
%}

end