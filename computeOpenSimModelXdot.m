function x_dot = computeOpenSimModelXdot(states,controls,t,osimModel,osimState,Iyinv)
% This function sets the model to a particular state, applies the controls
% (excitations) and returns the derivaties of the state variables.
%
% Author: Brian Umberger, UMass Amherst
%
% Note: This function draws from the function OpenSimPlantFunction.m by Daniel
% Jacobs (Stanford), which is part of the OpenSim dynamic walking example
%

% Import the OpenSim modeling classes
import org.opensim.modeling.*

% Update model state with current values  
osimState.setTime(t);
numVar = osimState.getNY();
I = 0:numVar-1;
% fset = @(i) osimModel.setStateVariableValue(osimState, osimModel.getStateVariableNames.get(i), states(i+1,1));
% x = osimModel.getStateVariableValues(osimState);
% x.setToZero();
x = Vector().createFromMat(states);
% fset = @(i) x.set(i, states(i + 1));
% arrayfun(fset, I)
osimModel.setStateVariableValues(osimState, x);

% Update the state velocity calculations
osimModel.computeStateVariableDerivatives(osimState);

% Get a reference to the current model controls
modelControls = osimModel.updControls(osimState);

for i = 1:numel(controls)
    modelControls.set(i - 1, controls(i));
end

% Set the control (excitation) values
osimModel.setControls(osimState,modelControls);

% Update the derivative calculations in the state variable
osimModel.computeStateVariableDerivatives(osimState);

% Set output variable to the new state

% fget = @(i) osimModel.getStateVariableDerivativeValue(osimState, osimModel.getStateVariableNames.get(i));
ydot = osimState.getYDot();
% fget = @(i) ydot.get(Iyinv(i + 1) - 1);
% x_dot = arrayfun(fget, I);
y_dot = ydot.getAsMat();
x_dot = y_dot(Iyinv).';
end
