function [Iy, Iyinv] = createSystemYIndexMap(model)

% Import the OpenSim modeling classes
import org.opensim.modeling.*

% DUMB hack-y way to load the opensim moco components
study = MocoStudy();
clear study;
% END OF DUMB HACK

Nstates = model.getNumStateVariables();

% Get a typical state for the model.
s = model.getWorkingState();

% Set all the states to zero.
for i = 1:Nstates
    s.updY.set(i - 1, 0)
end

% Set each state to 1 and find it in the model after mapping.
Imap = zeros([Nstates, 1]) + NaN;
for i_s = 1:Nstates
    s.updY.set(i_s - 1, 1);
    model_vec = model.getStateVariableValues(s);
    for i_mv = 1:model_vec.nrow
        if model_vec.get(i_mv - 1) > 0
            s.updY.set(i_s - 1, 0);
            Imap(i_s) = i_mv;
            break
        end
    end
end

Iy = Imap;
[Iyinv, ~] = find([Imap == (1:numel(Imap))]);