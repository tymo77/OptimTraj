% MAIN.m  --  Five Link Biped trajectory optimization
%
% This script sets up and then solves the optimal trajectory for the five
% link biped, assuming that the walking gait is compused of single-stance
% phases of motion connected by impulsive heel-strike (no double-stance or
% flight phases).
%
% The equations of motion and gradients are all derived by:
%   --> Derive_Equations.m 
%

clc; clear; 
addpath ../../
import org.opensim.modeling.*
study = MocoStudy();
clear study;

model = Model('2D_gait.osim');
model.initSystem();
init_state = model.initializeState();



%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%%                       Set up parameters and options                     %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
% Unfortunately, there is no discernable way to get the actuators from the
% component list at the beginning of the XML file after the model is
% imported. I have no earthly idea why.
actuatorNames = {
    'lumbarAct'
    'hamstrings_r'
    'bifemsh_r'
    'glut_max_r'
    'iliopsoas_r'
    'rect_fem_r'
    'vasti_r'
    'gastroc_r'
    'soleus_r'
    'tib_ant_r'
    'hamstrings_l'
    'bifemsh_l'
    'glut_max_l'
    'iliopsoas_l'
    'rect_fem_l'
    'vasti_l'
    'gastroc_l'
    'soleus_l'
    'tib_ant_l'
    };

speed = 1.2;

param.model = model;
param.state = init_state;

[Iy, Iyinv] = createSystemYIndexMap(model);
param.Iy = Iy;
param.Iyinv = Iyinv;
Nstates = model.getNumStateVariables();
Ncontrols = model.getNumControls();

state_names = model.getStateVariableNames();
state_names_cell = cell([Nstates, 1]);
fprintf('States:\n   i: i_y  name\n');
for i_state = 1:Nstates
    sn = state_names.get(i_state - 1).toCharArray()';
    state_names_cell{i_state} = sn;
    fprintf('%4d:  %s\n',i_state, sn);
end


% Only one coordinate is not periodic: the x translation of the pelvis.
periodicMap = zeros([19, 2]);
periodicMap(1, :) = [19 19]; % Pelvis Tilt Value
periodicMap(2, :) = [20 20]; % Pelvis Tilt Speed
periodicMap(3, :) = [22 22]; % Pelvis Tx Speed
periodicMap(4, :) = [23 23]; % Pelvis Ty Value
periodicMap(5, :) = [24 24]; % Pelvis Ty Speed
periodicMap(6, :) = [37 37]; % Lumbar Angle Value
periodicMap(7, :) = [38 38]; % Lumbar Angle Speed
periodicMap(8, :) = [25 27]; % Hip angle L/R
periodicMap(9, :) = [27 25]; % Hip angle R/L
periodicMap(10, :) = [26 28]; % Hip speed L/R
periodicMap(11, :) = [28 26]; % Hip speed R/L
periodicMap(12, :) = [29 31]; % Knee angle L/R
periodicMap(13, :) = [31 29]; % Knee angle R/L
periodicMap(14, :) = [30 32]; % Knee speed L/R
periodicMap(15, :) = [32 30]; % Knee speed R/L
periodicMap(16, :) = [33 35]; % Ankle value L/R
periodicMap(17, :) = [35 33]; % Ankle value R/L
periodicMap(18, :) = [34 36]; % Ankle speed L/R
periodicMap(19, :) = [36 34]; % Ankle speed R/L
activations_L = [1:9].'; activations_R = [10:18].'; % periodic muscles
periodicMap(20:28, :) = [activations_L activations_R];
periodicMap(29:37, :) = [activations_R activations_L];

params.periodicMap = periodicMap;
params.speedIdx = 21;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%%               Set up bounds on time, state, and control                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
problem.bounds.initialTime.low = 0;
problem.bounds.initialTime.upp = 0;
problem.bounds.finalTime.low = 0.2;
problem.bounds.finalTime.upp = 0.8;

x_lb = zeros([Nstates 1]);
x_ub = zeros([Nstates 1]);

x_lb(1:18) = 0.001; % Muscle activation lb.
x_lb(20:2:38) = -50; % Coordinate speeds lb.
x_lb(19) = deg2rad(-90); % pelvis tilt
x_lb(21) = 0; % pelvis tx
x_lb(23) = 0; % pelvis ty
x_lb([25 27]) = deg2rad(-60); % hip flexion l/r
x_lb([29 31]) = deg2rad(-120); % knee angle l/r
x_lb([33 35]) = deg2rad(-40); % ankle angle l/r
x_lb(37) = deg2rad(-90); % lumbar angle

x_ub(1:18) = 1; % Muscle activation lb.
x_ub(20:2:38) = 50; % Coordinate speeds lb.
x_ub(19) = deg2rad(90); % pelvis tilt
x_ub(21) = 1.0; % pelvis tx
x_ub(23) = 1.5; % pelvis ty
x_ub([25 27]) = deg2rad(120); % hip flexion l/r
x_ub([29 31]) = deg2rad(0); % knee angle l/r
x_ub([33 35]) = deg2rad(30); % ankle angle l/r
x_ub(37) = deg2rad(90); % lumbar angle

x_i_lb = x_lb;
x_i_ub = x_ub;

% Start at pelvis tx of 0.
x_i_lb(21) = 0;
x_i_ub(21) = 0;

x_f_lb = x_lb;
x_f_ub = x_ub;

problem.bounds.state.low =          x_lb;
problem.bounds.state.upp =          x_ub;
problem.bounds.initialState.low =   x_i_lb;
problem.bounds.initialState.upp =   x_i_ub;
problem.bounds.finalState.low =     x_f_lb;
problem.bounds.finalState.upp =     x_f_ub;


u_ub = zeros([Ncontrols 1]);
u_lb = zeros([Ncontrols 1]);
u_lb(1) = -1;% Lumbar Actuator lower bound.
u_ub(1) = 1; % Lumbar Actuator upper bound.
u_lb(2:19) = 0.01;% Muscle excitation lower bound.
u_ub(2:19) = 1.0; % Muscle excitation upper bound.

problem.bounds.control.low = u_lb;
problem.bounds.control.upp = u_ub;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%%              Create an initial guess for the trajectory                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

problem.guess.time = [0, 0.5];

x0 = mean([x_i_ub, x_i_lb], 2);
xf = mean([x_f_ub, x_f_lb], 2);
x0(19) = deg2rad(-17.155); % Pelvis tilt value.
x0(20) = deg2rad(97.309); % Pelvis tilt speed.
x0(21) = 0; % Pelvis x translation value.
x0(22) = speed; % Pelvis x translation speed.
x0(23) = 0.897; % Pelvis y translation value.
x0(24) = 0.347; % Pelvis y translation speed.
x0(25) = deg2rad(14.396); % L hip flexion value.
x0(26) = deg2rad(57.530); % L hip flexion speed.
x0(27) = deg2rad(25.273); % R hip flexion value.
x0(28) = deg2rad(-203.455); % R hip flexion speed.
x0(29) = deg2rad(-43.117); % L knee value.
x0(30) = deg2rad(-296.363); % L knee speed.
x0(31) = deg2rad(-2.691); % R knee value.
x0(32) = deg2rad(39.971); % R knee speed.
x0(33) = deg2rad(-15.161); % L ankle value.
x0(34) = deg2rad(-8.441); % L ankle speed
x0(35) = deg2rad(2.310); % R ankle value.
x0(36) = deg2rad(56.272); % R ankle speed
x0(37) = deg2rad(13.636); % Lumbar angle.
x0(38) = deg2rad(-122.867); % Lumbar angle speed.

xf(periodicMap(:, 2)) = x0(periodicMap(:, 1)); % Apply periodicity to guess.
xf(21) = speed*problem.guess.time(2); % Translate by average speed.

problem.guess.state = [x0, xf];

problem.guess.control = zeros(Ncontrols, 2);  %Start with passive trajectory

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%%                       Set up function handles                           %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

problem.func.dynamics =  @(t,x,u)( dynamics(t,x,u,param) );

problem.func.pathObj = @(t,x,u)( obj_controlSquared(t,x,u) );

problem.func.bndCst = @(t0,x0,tF,xF)( speedAndPeriodic(t0, x0, tF, xF, speed, params) );

problem.func.pathCst = [];


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%%                           Options:                                      %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%


%NOTE:  Here I choose to run the optimization twice, mostly to demonstrate
%   functionality, although this can be important on harder problems. I've
%   explicitly written out many options below, but the solver will fill in
%   almost all defaults for you if they are ommitted.

% method = 'trapezoid';
method = 'hermiteSimpson';
% method = 'chebyshev';   
% method = 'rungeKutta';  %slow!
% method = 'gpops';

Nt = 20; 
dirname = 'test'; k = 1;
switch method
    
    case 'trapezoid'
        problem.options(1).method = 'trapezoid'; % Select the transcription method
        problem.options(1).trapezoid.nGrid = Nt;  %method-specific options
        Nmesh = Nt;
        [~, pack] = packDecVar_dc(1:Nmesh, repmat(x_lb, [1 Nmesh]), repmat(u_lb, [1 Nmesh]));
        outputfcn = @(x,optimValues,state) logOptSteps(x, optimValues, dirname, pack, k);
    case 'hermiteSimpson'
        
        % First iteration: get a more reasonable guess
        problem.options(1).method = 'hermiteSimpson'; % Select the transcription method
        problem.options(1).hermiteSimpson.nSegment = Nt;  %method-specific options
        Nmesh = 2*Nt + 1;
        [~, pack] = packDecVar_dc(1:Nmesh, repmat(x_lb, [1 Nmesh]), repmat(u_lb, [1 Nmesh]));
        outputfcn = @(x,optimValues,state) logOptSteps(x, optimValues, dirname, pack, k);

    case 'chebyshev'
        
        % First iteration: get a more reasonable guess
        problem.options.method = 'chebyshev'; % Select the transcription method
        problem.options.chebyshev.nColPts = 9;  %method-specific options

    case 'multiCheb'
        
        % First iteration: get a more reasonable guess
        problem.options.method = 'multiCheb'; % Select the transcription method
        problem.options.multiCheb.nColPts = 6;  %method-specific options
        problem.options.multiCheb.nSegment = 4;  %method-specific options
        
    case 'rungeKutta'
        problem.options(1).method = 'rungeKutta'; % Select the transcription method
        problem.options(1).defaultAccuracy = 'low';
            
    case 'gpops'
        problem.options = [];
        problem.options.method = 'gpops';
        problem.options.defaultAccuracy = 'high';
        problem.options.gpops.nlp.solver = 'snopt';  %Set to 'ipopt' if you have GPOPS but not SNOPT
        
    otherwise
        error('Invalid method!');
end

%%%% Method-independent options:
problem.options(1).nlpOpt = optimset(...
    'Algorithm','interior-point',...
    'Display','iter',...   % {'iter','final','off'}
    'TolX',1e-10,...
    'TolFun',1e-3,...
    'MaxFunEvals',1e6,...
    'MaxIter', 1000,...
    'OutputFcn', outputfcn,...
    'PlotFcn', {'optimplotfval','optimplotconstrviolation','optimplotfirstorderopt'});   %options for fmincon

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%%                           Solve!                                        %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%%%%% THE KEY LINE:
soln = optimTraj(problem);

%% Transcription Grid points:
t = soln(end).grid.time;
X = soln(end).grid.state;
U = soln(end).grid.control;
figure;
tiledlayout(ceil(sqrt(Nstates)), ceil(Nstates/ceil(sqrt(Nstates))));
% Make the figures.
for i = 1:Nstates
    nexttile;
    state_name = state_names_cell{i};
    plot(t, X(i, :))
    title(state_name,'Interpreter','none')
end

[c, ceq] = problem.func.bndCst(t(1), X(:, 1), t(end), X(:, end));

%%
try
    close f1;
catch
end
load('testit_log_1.mat')
Nits = size(X, 3);
it_skip = 2;

f1 = figure;
tiledlayout(ceil(sqrt(Nstates)), ceil(Nstates/ceil(sqrt(Nstates))));

ax_list = cell([Nstates, 1]);
for i = 1:Nstates
    nexttile;
    state_name = state_names_cell{i};
    title(state_name,'Interpreter','none');
    ax_list{i} = gca;
end
for i_it = 1:it_skip:Nits
    t = T(:,:,i_it);
    x = X(:,:,i_it);
    u = U(:,:,i_it);
    for i = 1:Nstates
        ax = ax_list{i};
        hold(ax, 'on');
        plot(ax, t, x(i, :));
        hold(ax, 'off');
    end
end
f1.WindowState = 'maximized';


%%

info.controls_names = actuatorNames;
info.states_active_names = state_names_cell;
writeSTOFiles(soln(2), info, 'asdf')

