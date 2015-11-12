t0 = tic;
for i=1:numel(nulls)
    %[U{i}, S{i}, V{i}] = svd(nulls{i});
    [U, S, V] = svd(nulls{i}');
    %[U, S, V] = svds(nulls{i}', 1);
    Ss(i,:) = diag(S);
    lambda(i) = S(1:1,1:1) * V(:, 1)' * (qwPath{i} - qwPath{end});
    %{
    if i>=47 && i<=50
        i
        U
        diag(S)'
        V
        eig(nulls{i}')
    end
    if i>=49
        lambda(i) = -1*lambda(i);
    end
    %}
end
t1 = toc(t0)
