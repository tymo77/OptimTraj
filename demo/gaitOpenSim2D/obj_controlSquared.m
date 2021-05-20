function [dObj] = obj_controlSquared(t,x,u)
% [dObj] = obj_controlSquared(u)
%
% This function computes the torque-squared objective function and its
% gradients.
%

dObj = sum(u.^2, 1)./(t(end) - t(1));

end