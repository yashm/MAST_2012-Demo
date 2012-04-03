function params = load_params()

params.controlHandle = @lqrcontrol;
params.dynamicsHandle = @dynamics;

% Safety
params.maxAngle = 45*pi()/180;

%% Quadrotor Properties
% The angle of the gripper relative to the quadrotor
params.gripperangle = pi/2;
params.armlength = .175;
params.g = 9.81;
params.m = .700; %.614; %.580; % .488
params.kf = 6.107 * 10 ^ -8; % N / rpm^2
params.km = 1.5 * 10 ^ -9; % N*m / rpm^2
params.w_hover = sqrt(params.m*params.g/(4*params.kf));
params.I = [2.32e-3,0,0;0,2.32e-3,0;0,0,4e-3]; % Inertia Tensor

%% Control System Limitations

% Propeller Speeds
params.maxomega = 7800;
params.minomega = 1200;

% Max and min thrust (total)
params.maxForce = params.maxomega^2*4*params.kf;
params.minForce = params.minomega^2*4*params.kf;

% Max rotational moments about body axes
params.maxMpq = params.armlength*(params.maxForce/4 - params.minForce/4);
params.maxMr = 2*(params.maxomega^2*params.km - params.minomega^2*params.km);

end