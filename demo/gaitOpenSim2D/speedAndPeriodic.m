function [c, ceq] = speedAndPeriodic(t0, x0, tF, xF, speed, params)

[c1, ceq1] = periodicityConstraint(x0, xF, params);
[c2, ceq2] = speedConstraint(t0, x0, tF, xF, speed, params);

c = [c1; c2];
ceq = [ceq1; ceq2];
end