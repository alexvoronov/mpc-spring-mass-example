%% MPC example using YALMIP.
%
% This file goes step-by-step from simple experiments with ODE solver and
% simple MPC solutions to the MPC iterations and plotting of the results.

%% Parameters of the system.
p = struct;
p.M = 7;       % Number of spring-mass elements.
p.m = 1;       % Mass of each element.
p.c = 0.1;     % Friction coefficient.
p.k = 5;       % Linear spring coefficient, h(r) = k * r - k_nl * r .^ 3.
p.k_nl = 0.01; % Non-linear spring coefficient, h(r) = k * r - k_nl * r .^ 3.
p.u_max = 5;   % Max force that controller can apply.

M = p.M;  % M is used so often we lift it up.

%% Initial values for simulations.
x0 = [-1 3 1.5 -4 0.3 -0.5 -0.3];
x0 = x0(1:M);  % Remove elements from x0 if there are more than M.
v0 = zeros(1, M);

%% Plot predicted x positions after a direct solve.
addpath(genpath('YALMIP'))

[x, v, u, time_mpc] = mpc_linear_solve(x0, v0, p);
x_mpc = value(x);
figure
hs = stackedplot(time_mpc, x_mpc, '.-', 'DisplayLabels', "Elem " + string([1:M]'));
for i = 1:M; hs.AxesProperties(i).YLimits = [-0.25 0.25]; end
grid on

%% Plot predicted x positions for one MPC iteration.
tstart = tic;
[MpcSolver, time_mpc] = mpc_linear_opt(p);
fprintf('Solver prepared in %g sec\n', toc(tstart));
tstart = tic;
[sol, errorcode] = MpcSolver({x0, v0});
x_mpc = value(sol{1});
u = sol{2};
fprintf('MPC iteration done in %g sec\n', toc(tstart));
figure;
hs = stackedplot(time_mpc, x_mpc, '.-', 'DisplayLabels', "Elem " + string([1:M]'));
for i = 1:M; hs.AxesProperties(i).YLimits = [-0.5 0.5]; end
grid on

%% Solve uncontrolled linear system using MATLAB's ODE.
u0 = @(t) 0;  % Zero control signal.
isLinear = true;
[t, y] = ode45(@(t, y) springsmasssystem(t, y, p, isLinear, u0, u0), [0 100], [x0 v0]');
figure; stackedplot(t, y(:, 1:M), '.-')

%% Solve uncontrolled Non-Linear system using MATLAB's ODE.
u0 = @(t) 0;  % Zero control signal.
isLinear = false;
[tNL, yNL] = ode45(@(t, y) springsmasssystem(t, y, p, isLinear, u0, u0), [0 100], [x0 v0]');
figure; stackedplot(tNL, yNL(:, 1:M), '.-')

%% Plot linear vs NL (not too much difference there).
points = (0:0.1:100)';
yi = interp1(t, y, points);
yNLi = interp1(tNL, yNL, points);
t1 = array2table(yi(:, 1:M), 'VariableNames', "e"+string(1:M));
t2 = array2table(yNLi(:, 1:M), 'VariableNames', "eNL"+string(1:M));
tt = [t1, t2];
tt.time = seconds(points);
tt = table2timetable(tt);
figure;
stackedplot(tt, {{'e1', 'eNL1'}, {'e2', 'eNL2'}, {'e3', 'eNL3'}})


%% Prepare MPC solver for closed-loop system iterations.
[MpcSolver, time_mpc] = mpc_linear_opt(p);  % Prepare solver.

%% Iterating through the closed loop controlled system.
% Control 'u' is either:
%   - white noise (uncontrolled system)
%   - controller + white noise

%%% Prepare figure for plotting.
addpath('tight_subplot/')
figure('Color', 'w');
ax = tight_subplot(M, 1);
for i = 1:numel(ax)
    hold(ax(i), 'on');
    ax(i).Color = 'none';
    ax(i).YLim = [-1 1];
    ax(i).YTickLabelMode = 'auto';
    grid(ax(i), 'on');
end
ax(M).XTickLabelMode = 'auto';

%%% System simulation parameters.
% Length, in seconds, for each ODE solution, and the interval between MPC 
% soluton recalculation.
ode_length = 0.2;
% Noise magnitude.
noise_mag = 0.2 * 2 * p.u_max;
% Is Spring Mass System linear or not.
smsIsLinear = false;  

%%% Iteration parameters.
x0mpc = x0;  % Initial position.
v0mpc = v0;  % Initial speed.

%%% Variables for "bookkeeping" and analysis afterwards.
% Matrix to accumulate time (t) and positions (y) from ODE solutions 
% on each iteration.
tx_controlled = [];
tx_mpc = [];
% Time at the start of the iteration.
tstart = 0;  

for iter = 1:500
    fprintf("%d ", iter)  % To keep track of the progress.
    [sol, errorcode] = MpcSolver({x0mpc, v0mpc});    
    x_mpc = value(sol{1});
    u_mpc = value(sol{2});
    noise = @() noise_mag * (rand(size(u_mpc,1), 1) - 0.5);
    u1f = griddedInterpolant(time_mpc, u_mpc(:,1) + noise());  % control&noise
    u2f = griddedInterpolant(time_mpc, u_mpc(:,2) + noise());
    %u1f = griddedInterpolant(time, noise());  % no control, only noise
    %u2f = griddedInterpolant(time, noise());
    [t, y] = ode45(@(t, y) springsmasssystem(t, y, p, smsIsLinear, u1f, u2f), [0 ode_length], [x0mpc v0mpc]');
    x_ode = y(:, 1:M);  % Positions from ODE solution.
    tx_controlled = [tx_controlled; 
                     tstart+t, x_ode];  % Accumulate times and positions.
    tx_mpc = [tx_mpc;
              tstart+time_mpc, x_mpc;
              nan(1, 1+size(x_mpc,2))];
    % Plot during each iteration.
    for i = 1:M
        hsol = plot(ax(i), tstart + t, x_ode(:, i), '.-');
        hsol.Color(4) = 0.1;
        hsol.LineWidth = 3;
        hmpc = plot(ax(i), tstart + time_mpc, x_mpc(:,i), 'o-');
        hmpc.Color(4) = 0.1;
        hmpc.LineWidth = 0.75;
        hmpc.MarkerSize = 1;
        hmpc.MarkerEdgeColor(4) = 0.05;
        hmpc.MarkerFaceColor = hmpc.MarkerEdgeColor;
    end
    drawnow()
    
    % Update iteration parameters.
    x0mpc = y(end, 1:M);
    v0mpc = y(end, (M+1):(2*M));
    tstart = tstart + ode_length;
end
linkaxes(ax, 'x');

%%
tn = tx_controlled(:, 1);
[C, IA, IC] = unique(tn);
xx = interp1(tn(IA), tx_controlled(IA, 2:end), points);
for i = 1:M
    tt.(compose("e%dcontrolled", i)) = xx(:, i);
end

%%
figure;
hs = stackedplot(tt, {...
    {'e1', 'eNL1', 'e1controlled'}, ...
    {'e2', 'eNL2', 'e2controlled'},...
    {'e3', 'eNL3', 'e3controlled'}});
grid on
hs.LineWidth = 1.5;
%%
hFig = figure('Color', 'w', 'Position', [50 50 800 800]);
ax = tight_subplot(M, 1, 0, [0.08 0.01]);
for i = 1:numel(ax)
    hold(ax(i), 'on');
    ax(i).Color = 'none';
    ax(i).YLim = [-3 3];
    ax(i).YTickLabelMode = 'auto';
    grid(ax(i), 'on');
    ax(i).XTickLabel = {};
end

for i = 1:M
    hunc = plot(ax(i), seconds(tt.time), tt.(compose("eNL%d", i)));
    hunc.Color(4) = 0.1;
    hunc.LineWidth = 4;

    hsol = plot(ax(i), seconds(tt.time), tt.(compose("e%dcontrolled", i)), 'Color', [0.4940 0.1840 0.5560]);
    hsol.Color(4) = 0.4;
    hsol.LineWidth = 2;

    hmpc = plot(ax(i), (tx_mpc(:,1)), tx_mpc(:,i+1));
    hmpc.Color(4) = 0.3;
    hmpc.LineWidth = 0.5;
end
linkaxes(ax, 'x');
ax(M).XTickLabelMode = 'auto';
xlabel(ax(M), 'Time, sec');
for i = 1:M
    ylabel(ax(i), sprintf('x_%d', i));
end
xlim([0 50])
hl = legend(ax(1), {"Uncontrolled", "Controlled", "MPC prediction"});
%% 
addpath('export_fig')
export_fig(hFig, 'simulated-positions-800x1.5.png', '-m1.5', '-a4', '-transparent');
 