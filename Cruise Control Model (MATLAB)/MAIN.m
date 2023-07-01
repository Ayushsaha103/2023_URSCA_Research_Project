
format short g

%% Cruise Control
% Car a tails Car b, using closed loop cruise control algorithm

%  car a   ...    car b
%   +++     ->      +++


% VEHICLE PARAMETERS
a = Vehicle(7000,20000,1578);
% a.autotune_refspd();      % autotune car a's reference speed controller

b = Vehicle(7000, 20000, 1570);          % sports car params
% b = Vehicle(5000,5000,317);                % motorcycle params
% b = Vehicle(40000, 60000, 15000);        % truck params



% CRUISE CONTROL SIMULATION PARAMS
vset0=14;       % car b's speed
vset1=17;       % car a's desired speed
ntimesteps=100;    % num. timesteps         
a.runCruiseCtrl(b, vset0, vset1, ntimesteps);     % run cruise control simulation



%% Autotune Vehicle Reference Speed Controller
% f = Carr(7000,20000,1578);        % initialize new car
% f.autotune_refspd();      % autotune PID controller
% f.run_generic_sim(120);     % simulate and display velocity graphs

