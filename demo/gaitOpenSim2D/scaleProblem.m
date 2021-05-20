problem2 = problem;

%% Scale state.

x1 = problem.bounds.state.low;
x2 = problem.bounds.state.upp;

y1 = 1; y2 = 2;

m = (y2 - y1)./(x2 - x1);
b = y1 - m.*x1;

minv = 1./m;
binv = -b./m;

scaler.state.m = m;
scaler.state.b = b;
scaler.state.minv = minv;
scaler.state.binv = binv;

problem2.bounds.state.low = m.*problem.bounds.state.low + b;
problem2.bounds.state.upp = m.*problem.bounds.state.upp + b;

problem2.bounds.initialstate.low = m.*problem.bounds.initialstate.low + b;
problem2.bounds.initialstate.upp = m.*problem.bounds.initialstate.upp + b;

problem2.bounds.finalstate.low = m.*problem.bounds.finalstate.low + b;
problem2.bounds.finalstate.upp = m.*problem.bounds.finalstate.upp + b;

problem2.guess.state = m.*problem.guess.state + b;


%% Scale Controls
x1 = problem.bounds.control.low;
x2 = problem.bounds.control.upp;

y1 = 1; y2 = 2;

m = (y2 - y1)./(x2 - x1);
b = y1 - m.*x1;

minv = 1./m;
binv = -b./m;

scaler.control.m = m;
scaler.control.b = b;
scaler.control.minv = minv;
scaler.control.binv = binv;

problem2.bounds.control.low = m.*problem.bounds.control.low + b;
problem2.bounds.control.upp = m.*problem.bounds.control.upp + b;

problem2.guess.control = m.*problem.guess.control + b;

%% Scale Time
x1 = problem.bounds.initialTime.low;
x2 = problem.bounds.finalTime.upp;

y1 = 1; y2 = 2;

m = (y2 - y1)./(x2 - x1);
b = y1 - m.*x1;

minv = 1./m;
binv = -b./m;

scaler.time.m = m;
scaler.time.b = b;
scaler.time.minv = minv;
scaler.time.binv = binv;

problem2.bounds.initialTime.low = m.*problem.bounds.initialTime.low + b;
problem2.bounds.initialTime.upp = m.*problem.bounds.initialTime.upp + b;

scaler.finalTime.m = m;
scaler.finalTime.b = b;
scaler.finalTime.minv = minv;
scaler.finalTime.binv = binv;

problem2.bounds.finalTime.low = m.*problem.bounds.finalTime.low + b;
problem2.bounds.finalTime.upp = m.*problem.bounds.finalTime.upp + b;

problem2.guess.time = m.*problem.guess.time + b;

%% Scale the problem functions by wrapping them.

function xdot = dynamicsWrapper(fdyn, t, x, u, param, scaler)
xdot = fdyn
end