% This is the simulator for the quadrotor
function sdot = Simulator(t, s0)

global traj params log

log.time = [log.time; t];

% Lets use understandable values
x = s0(1);
y = s0(2);
z = s0(3);
xdot = s0(4);
ydot = s0(5);
zdot = s0(6);
phi = s0(7);
theta = s0(8);
psi = s0(9);
p = s0(10);
q = s0(11);
r = s0(12);
w = s0(13:16);

% Rotation Matrix from the body frame to the inertial frame
Rot = EulToRot(phi,theta,psi);

%% Plant

w = max(1200,min(7800,w));

% Determine the forces and moments from individual rotors
F = params.kf*[w(1) w(2) w(3) w(4)]'.^2;
M = params.km*[w(1) w(2) w(3) w(4)]'.^2;

% Logging
log.F = [log.F; sum(F)];
log.w = [log.w; w'];

% Newton's Equations of Motion
rddot = [0 0 -params.g]' + Rot*[0 0 sum(F)]'/params.m;
xddot = rddot(1);
yddot = rddot(2);
zddot = rddot(3);

% And from Euler's Equations
L = params.armlength;
I = params.I;
alpha_body = I\[L*(F(2)-F(4)); L*(F(3)-F(1)); M(1)-M(2)+M(3)-M(4)]...
    - I\cross([p;q;r;],I*[p;q;r;]);
pdot = alpha_body(1);
qdot = alpha_body(2);
rdot = alpha_body(3);

% p, q, and r can be related to phi, theta, and psi: Change of Coordinates
Conv = [...
    cos(psi),            -sin(psi),            0;
    sec(phi)*sin(psi),   cos(psi)*sec(phi),    0;
    sin(psi)*tan(phi),   cos(psi)*tan(phi),    1];

euler_angles_dot = Conv*[p;q;r;];
phidot = euler_angles_dot(1);
thetadot = euler_angles_dot(2);
psidot = euler_angles_dot(3);


%% Controller

% Find the closest trajectory time index
idx = find(traj.time > t, 1, 'first');
if isempty(idx); idx = length(traj.time); end;

[w_des] = feval(params.controlHandle, s0(1:12), idx);

%% Propeller spinups with saturation constraints
km = 20; % 1/s
wdot = zeros(4,1);
for idx = 1:4
    if w(idx) >= 7800
        wdot(idx) = min(0,km*(w_des(idx) - w(idx)));
    elseif w(idx) <= 1200
        wdot(idx) = max(0,km*(w_des(idx) - w(idx)));
    else
        wdot(idx) = km*(w_des(idx) - w(idx));
    end
end

%% Return sdot

sdot = zeros(16,1);
sdot(1) = xdot;% xdot
sdot(2) = ydot;% ydot
sdot(3) = zdot;% zdot
sdot(4) = xddot;% xddot
sdot(5) = yddot;% yddot
sdot(6) = zddot; % zddot
sdot(7) = phidot;% phidot
sdot(8) = thetadot;% thetadot
sdot(9) = psidot;% psidot
sdot(10) = pdot;% pdot
sdot(11) = qdot;% qdot
sdot(12) = rdot;% rdot
sdot(13) = wdot(1);% w1dot
sdot(14) = wdot(2);% w2dot
sdot(15) = wdot(3);% w3dot
sdot(16) = wdot(4);% w4dot

end