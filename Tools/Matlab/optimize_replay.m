%% Main loop
for i=1:numel(raw)-1
        %% Reformat the Data
        % Original Path
        qw0 = [qwPath{:}]';
        % Task Space target
        vw0 = [vwPath{:}]';
        
        %% Run the optimizer
        % Regular Optimization
        [qw, dt_opt] = ...
            optimize_armplan(raw(i).qw0, raw(i).vw0, nulls, Js);
        % Subspace Optimization
        [qLambda, dt_opt_lambda] = ...
            optimize_armplan_lambda(raw(i).qw0, raw(i).vw0, nulls, Js);
end

%% Finish with a plot
%show_armplan;