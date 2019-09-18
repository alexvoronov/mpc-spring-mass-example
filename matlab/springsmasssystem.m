function dydt = springsmasssystem(t, y, p, isLinear, u1f, u2f)
% SPRINGSMASSSYSTEM  Spring-mass system equations for the ODE solver.
%
%     $m*a_i + 2c vi + k(xi-xim1) + k(xi - xip1) = 0$
%
% ODE solver requires function f such that  y' = f(t,y)
% Here, y = [x; v], x' = v, and v' = a.
% Then function is [v; a] = f(t, [x; v])
%
% Input: [x_in; v_in]. Output: dydt = [x'; v'] = [v_out; a_out]
%
% v_out = v_in
% a_out = - 1/m * (2cv + k(x-x) + k(x-x))

M = p.M;
assert(M == numel(y)/2, compose("bad number of spring masses %d %d", M, numel(y)));
m = p.m;
k = p.k;
k_nl = p.k_nl;
c = p.c;

x = y(1:M);
v = y(M+1:2*M);
a = zeros(M, 1);

u1 = u1f(t);
u2 = u2f(t);

isNonLinear = 1 - isLinear;
h = @(r) 5 * r - isNonLinear * k_nl * r .^ 3;  % Nonlinear spring function.

% Inner spring-masses (2..M-1).
i = 2:M-1;  % Indices of inner springmasses.
a(i) = - 1/m * (2 * c * v(i) + h(x(i) - x(i-1)) + h(x(i) - x(i+1)));

% Outer (leftmost and rightmost) springmasses.
a(1) = - 1/m * (u1 + 2 * c * v(1) + k*(x(1)         ) + h(x(1) - x(1+1)));
a(M) = - 1/m * (u2 + 2 * c * v(M) + h(x(M) - x(M-1)) + k*(x(M)         ));

dydt = [v;a];

end