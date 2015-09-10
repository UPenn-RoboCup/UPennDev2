%% Number of null space dimensions
nNull = 2;
nt = dt * np;
t = 0:dt:nt-dt;

%% Filter the nullspace in time
gamma = 1;
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
lambda0 = lambda;

ds = min(abs(Ss(:,1) - Ss(:,2)), [], 2);

%% Determine the signs from the SVD for consistent basis directions
% NOTE: This should be slow, but not *too* bad ;)
swapidx = [1];
for iN=1:nNull
    for i=2:numel(nulls)
        dirlambda = dot(Vs{i-1}(:, iN), Vs{i}(:, iN));
        %fprintf(1, '%d: %.2f: %.2f\n', iN, t(i), dirlambda);
        if abs(dirlambda) > 0.94 % Pretty much the same dir...
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
        fprintf(1, '[%d]: %.2f {%.2f, %.2f}, {%.2f, %.2f}\n', ...
            iN, t(is), dirswap1, dirswap2, dirswapA, dirswapB);
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
        end
    end
end 

%% Run the optimization for each dimension...?
clear ddlambda dlambda dqLambda qLambda;
swapidx0 = swapidx;
%swapidx = swapidx(segment);
%%{
ddlambda = lambda;
for i=1:numel(swapidx)-1
    range = swapidx(i):swapidx(i+1)-1;
    % Only if enough points
    if numel(range)>5
        ddlambda(range, :) = subopt_lambda(lambda(range, :), ds);
    end
end

%% Run as normal
dlambda = subopt_lambda(lambda, ds);

%% Change back into q
qLambda = zeros(np, nq);
dqLambda = zeros(np, nq);
for i=1:size(lambda, 1)
    % TODO: Us should be a cell array and then choose nNull columns
    dqLambda(i, :) = Us{i} * (ddlambda(i, :) - lambda(i, :))';
    qLambda(i, :) = qwPath{i} + dqLambda(i, :)';
end
%}