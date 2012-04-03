function Dynamic_Sim(varargin)
% This program generates a trajectory and controls a quadrotor
addpath('State_Control/utilities');

disp('Running Dynamic Simulation...');

% Initalize params
params = load_params();

global traj log

% Load the trajectory into the Global Workspace
% hover
% Trajectory_Generator

% Load the trajectory
traj = load('traj');
traj = traj.traj;

% logs
% log.w_des = [];
log.phi_des = [];
log.theta_des = [];
log.F = [];
log.w = [];
% log.pos_e = [];
log.pos = [];
log.time = [];
log.u = [];

% Initalize the controller
feval(params.controlHandle);

s0 = [traj.pos(1,:)'; zeros(3,1); traj.phi(1); traj.theta(1); traj.psi(1); zeros(3,1); params.w_hover*ones(4,1)];
tf = traj.keytimes(end)+1;

% We can now determine the output of the system
[time states] = ode45(@Simulator,[0,tf],s0); %#ok<NASGU,ASGLU>

% Find the closest point on the trajectory
% d = bsxfun(@minus, traj.pos, [x y z]);
% [C idx] = min(sum(d.^2,2));
% idx = min(idx+20, size(traj.pos,1))

plots

end