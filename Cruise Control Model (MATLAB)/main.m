
format short g


%% autotune PID controller

f = Carr(7000,20000,1578);
f.autotune_refspd();      % autotune speed controller (uses NEURAL NET)

%% Cruise Control
% Under autonomous control, car a tails car b

a = Carr(7000,20000,1578);
b = Carr(5000,5000,317);                % motorcycle parameters
% b = Carr(7000, 20000, 1500);          % sports car parameters
% b = Carr(40000, 60000, 15000);        % truck parameters

vset0=12; vset1=18; ntimesteps=100;     % cruise ctl params
a.runCruiseCtrl(b, vset0, vset1, ntimesteps);     % call runcruisectrl()


