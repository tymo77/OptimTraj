function [dObj] = obj_controlCubed(t,x,u)
% [dObj] = obj_controlSquared(u)
%
% This function computes the torque-squared objective function and its
% gradients.
%

dObj = sum(abs(u).^3, 1)./(t(end) - t(1));

end