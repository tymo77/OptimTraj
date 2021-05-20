function [dx] = dynamics(t, x, u, params)

Nt = numel(t);
I = 1:Nt;
fun = @(i) computeOpenSimModelXdot(x(:, i), u(:, i), t(i), params.model, params.state, params.Iyinv);
dx = arrayfun(fun, I, 'UniformOutput', false);
dx = cell2mat(dx.').';
end