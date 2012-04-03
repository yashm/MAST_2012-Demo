function [pd_cmd,  curr_state] = xyz_traj_J_working(curr_state, quad, gains, varargin)
%function [pd_cmd curr_state] = xyz_traj_J (curr_state, quad, gains, traj)

%Trajectory controller with integral gain
feedforward = true;
max_asin = sin(20*pi/180);
max_thint=50;
max_xyint=0.4;
pd_cmd = asctec_PDCmd('empty');

persistent params

% This is executed the first time xyz_traj_J is ran
if(curr_state.first_run_in_state)
    curr_state.first_run_in_state=0;
    curr_state.reset_vars= 0;
    params = load_params();
end

use_int = 1;
for i = 1:2:length(varargin)%nargin-4              % get optional args
    if isempty(varargin{i})
        continue
    end
    switch varargin{i}
        
        case 'traj', traj = varargin{i+1};
            
        case 'feedforward', feedforward = varargin{i+1};
            
        case 'use_int', use_int = varargin{i+1};
            
        case 'servo'
            
        otherwise, error(['Unkown parameter: ' varargin{i}]);
    end
end

% If we don't have a trajectory passed, just quit now
if ~isfield(traj,'pos')
    warning('No Trajectory provided to xyz_traj.'); %#ok<WNTAG>
    pd_cmd = [];
    return;
end

% Use gains from the gains passed

kp_x = gains.kp_x;
kd_x = gains.kd_x;
ki_x = gains.ki_x;

kp_y = gains.kp_y;
kd_y = gains.kd_y;
ki_y = gains.ki_y;

kp_z = gains.kp_z;
kd_z = gains.kd_z;
ki_z = gains.ki_z;

% Get the current timer for the state
timer=curr_state.state_timer;

% Store our current state estimate into readable variables
x_est=curr_state.x_est;
y_est=curr_state.y_est;
z_est=curr_state.z_est;
xd_est=curr_state.xd_est;
yd_est=curr_state.yd_est;
zd_est=curr_state.zd_est;
psi=curr_state.psi;
phi=curr_state.phi;
theta=curr_state.theta;

% Store trims into readable variables
th_trim = quad.th_trim;
phi_trim = quad.phi_trim;
theta_trim = quad.theta_trim;
yaw_trim = quad.yaw_trim;

delTint = curr_state.delT;

% Find the closest trajectory time index
idx = find(traj.time >= timer, 1, 'first');

% If an index was not found
if isequal(idx,length(traj.time)) || isempty(idx)
    pd_cmd=[];
    return;
end

% Store our desired positions into readable variables
x_des=traj.x(idx);
y_des=traj.y(idx);
z_des=traj.z(idx);
psides=traj.psi(idx);

% Load the desired positions into the curr_state structure
curr_state.x_des = x_des;
curr_state.y_des = y_des;
curr_state.z_des = z_des;
curr_state.psi_des=psides;

% Store our desired velocities into readable variables
xd_des = traj.vel(idx,1);
yd_des = traj.vel(idx,2);
zd_des = traj.vel(idx,3);

% Calculate integral errors
if use_int
    xybodyint = delTint * [cos(psi), sin(psi); -sin(psi), cos(psi)]*[x_des-x_est;y_des-y_est];
    zbodyint = delTint * (z_des-z_est);
    
    phi_int=curr_state.phi_int;
    th_int=curr_state.th_int;
    theta_int=curr_state.theta_int;
    
    phi_int = phi_int + ki_y*-xybodyint(2);
    theta_int = theta_int + ki_x*xybodyint(1);
    th_int = th_int + ki_z*zbodyint;
    
    th_int = max(min(th_int,max_thint),-max_thint);
    phi_int = max(min(phi_int,max_xyint),-max_xyint);
    theta_int = max(min(theta_int,max_xyint),-max_xyint);
    
    curr_state.phi_int = phi_int;
    curr_state.th_int = th_int;
    curr_state.theta_int = theta_int;
else
    phi_int=curr_state.phi_int;
    th_int=curr_state.th_int;
    theta_int=curr_state.theta_int;
end

%% Controller

th_base = 88;

th_cmd = th_base+kp_z*(z_des-z_est) + kd_z*(zd_des - zd_est)+th_int+th_trim;

ux = kp_x*(x_des - x_est) + kd_x*(xd_des - xd_est);
uy = kp_y*(y_des - y_est) + kd_y*(yd_des - yd_est);

% Determine the desired angles to correct for position and velocity
% errors
phides = ux*sin(psi) - uy*cos(psi)+phi_trim + phi_int;
thetades = ux*cos(psi) + uy*sin(psi)+theta_trim + theta_int;

% Enforce limits on our desired angles
phides = asin(max(min(phides,max_asin),-max_asin));
thetades = asin(max(min(thetades,max_asin),-max_asin));

%think about changing these
pd_cmd.kp_pitch = 220*1.1;
pd_cmd.kd_pitch = 20*1.12;
pd_cmd.kp_roll = 220;
pd_cmd.kd_roll = 20;
pd_cmd.kd_yaw = 33;

kp_yaw = 150;
psi_diff = mod(psides - psi,2*pi);
psi_diff = psi_diff - (psi_diff>pi)*2*pi;
pd_cmd.yaw_delta = kp_yaw * (psi_diff)+yaw_trim;


if feedforward
    
    % Add the feedforward roll
    phides = phides + traj.phi(idx);
    
    % Add the feedforward pitch
    thetades = thetades + traj.theta(idx);
    
    % Determine the thrust command (above hover)
%     ff_th_cmd = sqrt((traj.u1(idx) - params.m*params.g)/(4*params.kf))*(1/35);
%     th_cmd = th_cmd + ff_th_cmd;

end

% Put limits on and round the thrust command
th_cmd = max(min(th_cmd,200),0);
pd_cmd.thrust = round(th_cmd);

pd_cmd.roll = phides;
pd_cmd.pitch = thetades;

% Establish angular rates to achieve angular orientation
pd_cmd.roll_delta = pd_cmd.kp_pitch * (phides-phi);
pd_cmd.pitch_delta = pd_cmd.kp_roll * (thetades-theta);

curr_state.phi_des = traj.phi(idx);
curr_state.theta_des = traj.theta(idx);
curr_state.psi_des = traj.psi(idx);

end