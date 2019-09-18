function [P, time] = mpc_linear_opt(p)
% MPC_LINEAR_OPT  returns YALMIP Optimizer for a spring-mass MPC.

m = p.m;
k = p.k;
c = p.c;
M = p.M;
u_max = p.u_max;
N = 100;         % Number of discretization points.
dt = 0.100;      % Duration of each discretization step, in seconds.

% Penalties.
R = 50;
Q = 1;

% Variables.
x = sdpvar(N,M);
v = sdpvar(N,M);
a = sdpvar(N,M);
u = sdpvar(N,2);
x0 = sdpvar(1,M);
v0 = sdpvar(1,M);

% Indexing for convenience.
next = 2:N;
prev = 1:N-1;
inner = 2:M-1;

% Constraints.
% Note:  x(:, "0") == x(:, "M+1") ==  0
Constraints = [
x(1, :) == x0;
v(1, :) == v0;
abs(u) <= u_max;
x(next, :) == x(prev, :) + 0.5 * (v(prev, :) + v(next, :)) * dt;
v(next, :) == v(prev, :) + 0.5 * (a(prev, :) + a(next, :)) * dt;
(          m * a(:, inner) + 2 * c * v(:, inner) + k * (x(:, inner) - x(:, inner-1)) + k * (x(:, inner) - x(:, inner+1))) == 0;% soft_f(:, inner);
(u(:, 1) + m * a(:,     1) + 2 * c * v(:,     1) + k *  x(:,     1)                  + k * (x(:,     1) - x(:,     1+1))) == 0;% soft_f(:, 1);
(u(:, 2) + m * a(:,     M) + 2 * c * v(:,     M) + k * (x(:,     M) - x(:, M-1))     + k *  x(:,     M)                 ) == 0;% soft_f(:, M);
];
Objective = R * norm(x, 1) + Q * norm(u, 1);

% Set some options for YALMIP and solver
%options = sdpsettings('verbose',1,'solver','quadprog','quadprog.maxiter',100);
options = sdpsettings('verbose', 1,'solver','linprog');

% Return a solver.
P = optimizer(Constraints,Objective,options, {x0, v0}, {x, u});
time = (0:dt:dt*(N-1))';

end