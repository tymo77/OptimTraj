function [c, ceq] = speedConstraint(t0, x0, tF, xF, speed, params)

c = [];

% Periodic states should be periodic.
pelvis_distance = xF(params.speedIdx) - x0(params.speedIdx);

avg_speed = pelvis_distance/(tF - t0);

ceq = avg_speed - speed;

end