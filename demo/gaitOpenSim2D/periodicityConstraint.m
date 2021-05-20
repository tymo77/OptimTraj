function [c, ceq] = periodicityConstraint(x0, xF, params)

c = [];

periodicMap = params.periodicMap;

% Periodic states should be periodic.
ceq = x0(periodicMap(:, 1)) - xF(periodicMap(:, 2));




end