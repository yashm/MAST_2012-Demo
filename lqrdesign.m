% Justin Thomas
% LQR Design for Quadrotor

% The Linearized model: sdot = As + Bu for
% s = [x; y; z; xdot; ydot; zdot; phi; theta; psi; p; q; r;]; and
% u = [Fnet; M1; M2; M3;];

params = load_params();

% Set the diagonal elements of our Q matrix
Q = diag([...
    1/(2/100)^2;            % x
    1/(2/100)^2;            % y
    1/(2/100)^2;             % z
    1/(10/100)^2;                % xdot
    1/(10/100)^2;                % ydot
    1/(5/100)^2;               % zdot
    1/(10*pi/180)^2;       % phi
    1/(10*pi/180)^2;       % theta
    1/(10*pi/180)^2;         % psi
    1/(40*pi/180)^2;         % p
    1/(40*pi/180)^2;         % q
    1/(40*pi/180)^2;]);      % r

Ixx = params.I(1,1);
Iyy = params.I(2,2);
Izz = params.I(3,3);

g = params.g;
m = params.m;

global K
for psideg = 1:360
    
    % Since we need radians
    psi = deg2rad(psideg);
    
    A = [...
        0 0 0 1 zeros(1,8);
        0 0 0 0 1 zeros(1,7);
        0 0 0 0 0 1 zeros(1,6);
        zeros(1,6) 0 g zeros(1,4);  % xddot
        zeros(1,6) -g 0 zeros(1,4);  % yddot
        zeros(1,12);                           % zddot
        zeros(1,9) cos(psi) -sin(psi) 0;    % phidot
        zeros(1,9) sin(psi) cos(psi) 0;   % thetadot
        zeros(1,9)  0 0 1;    % psidot
        zeros(3,12);];   % p, q, r
    
    B = [...
        zeros(3,4);
        zeros(1,4);
        zeros(1,4);
        1/m 0 0 0;
        zeros(3,4);
        0 1/Ixx 0 0;
        0 0 1/Iyy 0;
        0 0 0 1/Izz;];
    
    system = ss(A,B,eye(12),[],'StateName',{'x','y','z','xdot','ydot','zdot','phi','theta','psi','p','q','r'},'InputName',{'F','M1','M2','M3'});
    
    R = [...
        1/(params.maxForce)^2 0 0 0;
        0 1/params.maxMpq^2 0 0;
        0 0 1/params.maxMpq^2 0;
        0 0 0 1/params.maxMr^2;];
    
    K{psideg} = lqr(system,Q,R);
    
end