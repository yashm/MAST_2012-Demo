function traj = Trajectory_Generator(saferegion, start, pickup, finish, speed, pickup_vertical_velocity)
% function traj = Trajectory_Generator(saferegion, start_pos, pickup_pos, end_pos, speed)
%
% This program is used to determine an appropriate trajectory for a
% grasping quadrotor.  The positions are specified as [x; y; z; psi;].  The
% saferegion is [xmin ymin zmin xmax ymax zmax].  speed is the speed at
% which the quadrotor will actually be picking up the crawler.  The
% pickup_vertical_velocity is the vertical velocity that the trajectory
% will have at the pickup point.  Units are m, m/s, and rad.

%% Parameters and Thresholds

% Set the desired thresholds here:
velocity_threshold = 1.8*speed;
acc_threshold = 10;
maxPsidot = pi/4;

% Parameters
tstep = .005;
params = load_params();

%% States

s = zeros(4,3);
s(1:4,:) = [start' pickup' finish'];

sdot = zeros(4,3);   % xdot, ydot, zdot, psidot
sddot = zeros(4,3);  % ddots

% A first guess at keytimes.  Obviously, they are too fast.
keytimes(1) = 0;
keytimes(2) = .5;
keytimes(3) = 1;

% Let's use the pickup velocity and orientation to determine our boundary
% conditions
attackangle = atan2(finish(2)-start(2), finish(1)-start(1));
sdot(1,2) = speed*cos(attackangle); % xdot at pickup
sdot(2,2) = speed*sin(attackangle); % ydot at pickup

% Let's be safe
limit = .2;
sdot(3,2) = max(min(pickup_vertical_velocity,limit),0);

%% Now we determine a trajectory using smooth polynomial curves

polyorder = [7 7 7 3];
coeffs = cell(4, length(keytimes)-1);
val = coeffs;
D1val = coeffs;
D2val = coeffs;
D3val = coeffs;
D4val = coeffs;

for key = 1:length(keytimes)-1                          % Segment Loop
    
    while (1)                                           % Optimization Loop
        
        t = (keytimes(key + 1) - keytimes(key));
        
        for idx = 1:4                                   % Flat Output Loop

            n = polyorder(idx);
            halfnumbcs = round((n + 1)/2);
            
            Start_And_End_Cond = condmatrix(n, t);
            
            % The boundary conditions
            if n >= 7
                bcs = [s(idx,key); sdot(idx,key); sddot(idx,key); zeros(halfnumbcs - 3,1);...
                    s(idx,key + 1); sdot(idx,key + 1); sddot(idx,key + 1); zeros(halfnumbcs - 3,1);];
            elseif n >=3
                bcs = [s(idx,key); sdot(idx,key); zeros(halfnumbcs - 2,1);...
                    s(idx,key + 1); sdot(idx,key + 1); zeros(halfnumbcs - 2,1);];
            end
            
            % Find the analytical solution
            coeffs{idx,key} = flipud(Start_And_End_Cond^(-1)*bcs);
            
            % And find all the interesting derivatives
            val{idx,key} = polyval(coeffs{idx,key},0:tstep:t-tstep);
            D1val{idx,key} = polyval(polyder(coeffs{idx,key}),0:tstep:t-tstep);
            D2val{idx,key} = polyval(polyder(polyder(coeffs{idx,key})),0:tstep:t-tstep);
            D3val{idx,key} = polyval(polyder(polyder(polyder(coeffs{idx,key}))),0:tstep:t-tstep);
            D4val{idx,key} = polyval(polyder(polyder(polyder(polyder(coeffs{idx,key})))),0:tstep:t-tstep);
        end
        
        %% Feedforward control
        
        xdd = D2val{1,key}';
        ydd = D2val{2,key}';
        zdd = D2val{3,key}';
        
        % Each row of zbody is a unit vector at each timestep
        % [xworld, yworld, zworld]
        zb = [xdd, ydd, zdd + params.g];
        norm_zb = rownorm(zb);
        
        % Determine the feedforward control for the thrust
        traj.u1{key} = norm_zb*params.m;
        
        % Normalize the zb vector
        % #### Can I use unit?
        zb_unit = zb ./ (rownorm(zb)*ones(1,3));
        
        % Store psi
        traj.psi{key} = val{4,key}';
%         
%         % Determine xc, the x vector of the frame that accounts only for
%         % yaw angle.
%         xc = [cos(traj.psi{key}), sin(traj.psi{key}), zeros(length(traj.psi{key}),1)];
%         
%         % Determine the unit vectors representing the body frame y axis in
%         % the world coordinates
%         xb_cross_xc = cross(zb_unit, xc);
%         yb = xb_cross_xc ./ (rownorm(xb_cross_xc)*ones(1,3));
%         
%         % And the body frame x axis in world coordinates
%         xb = cross(yb, zb);
        
        % We can determine the euler angles from the Z - X - Y
        % representation.
        traj.phi{key} = asin(-zb_unit(:,2));
        traj.theta{key} = atan2(zb_unit(:,1),zb_unit(:,3));
        
        % And the angular velocity in the body fixed frame
        
        %% Check if our trajectory is too aggressive
        checkthresh = [...
            max(abs(traj.phi{key})) > params.maxAngle ...
            max(abs(traj.theta{key})) > params.maxAngle ...
            max(traj.u1{key}) > .5*params.maxForce ...
            max(rownorm([D1val{1,key}' D1val{2,key}' D1val{3,key}'])) > velocity_threshold ...
            max(rownorm([D2val{1,key}' D2val{2,key}' D2val{3,key}'])) > acc_threshold ...
            max(D1val{4,key}) > maxPsidot];
        %             max(u2{key}) > maxMoment ...
        %             max(u3{key}) > maxMoment ...
        %             max(u4{key}) > params.maxPsiMom ...

        if  any(checkthresh)
            lastcheckthresh = checkthresh;
            
            % Shift all successive keytimes appropriately
            keytimes(key + 1:end) = keytimes(key + 1:end) + .3;
            
        else
            factors = {'Phi', 'Theta', 'Thrust', 'Velocity', 'Acceleration', 'Psidot'};
            % factors = {'Phi', 'Theta', 'Thrust', 'M1', 'M2', 'Velocity', 'Acceleration', 'Psi Moment', 'Psidot'};
            disp(['Segment ', num2str(key), ' Duration: ', num2str(t), ' seconds']);
            disp(['Limiting Factor(s):', factors(lastcheckthresh)]);
            
            break;
        end
        
        if t > 15
            disp('The trajectory is longer than 15 seconds.  You may want to just quit now and change your parameters.');
            keyboard
        end
    end
end

%% Store the trajectory information
traj.tstep = tstep;
traj.keytimes = keytimes;

traj.x = [val{1,:}]';
traj.y = [val{2,:}]';
traj.z = [val{3,:}]';

traj.time = (0:tstep:length(traj.x)*tstep - tstep)';

traj.phi = cell2mat(traj.phi');
traj.theta = cell2mat(traj.theta');
traj.psi = cell2mat(traj.psi');

traj.start = start;
traj.pickup = pickup;
traj.finish = finish;

% traj.p = []';
% traj.q = []';
% traj.r = []';

% traj.u = [[u1{:}]' [u2{:}]' [u3{:}]' [u4{:}]'];
traj.u1 = cell2mat(traj.u1');
% traj.u4 = I(3,3)*

traj.pos = [traj.x traj.y traj.z];
traj.vel = [[D1val{1,:}]' [D1val{2,:}]' [D1val{3,:}]'];
traj.acc = [[D2val{1,:}]' [D2val{2,:}]' [D2val{3,:}]'];

traj.coeffs.x = [coeffs{1,:}];
traj.coeffs.y = [coeffs{2,:}];
traj.coeffs.z = [coeffs{3,:}];
traj.coeffs.psi = [coeffs{4,:}];

save('traj','traj');

disp(['Total Time: ', num2str(keytimes(end)), ' seconds'])

if any(any(traj.pos < ones(length(traj.pos),1)*saferegion(1:3))) || any(any(traj.pos > ones(length(traj.pos),1)*saferegion(4:6)))
    error('The trajectory does not fall within the safe region');
end
end

function norms = rownorm(vec)
norms = sqrt(sum(vec.^2,2));
end

function Start_And_End_Cond = condmatrix(n, t)

persistent lastn basis D

% If we have a different polynomial order, we need to generate a new
% differential linear operator
if ~exist('lastn', 'var') || ~isequal(lastn, n)
    LinDeriv = zeros(n + 1);
    for idx2 = 1:n
        LinDeriv(idx2,:) = [zeros(1,idx2) (n - idx2 + 1) zeros(1,n-idx2)];
    end
    D = cell(4,1);
    D{1} = LinDeriv';
    D{2} = D{1}^2;
    D{3} = D{1}*D{2};
    D{4} = D{1}*D{3};
    lastn = n;
end

% Generate our Basis
basis = basisgen(t, n);
halfnumbcs = round((n + 1)/2);

% Generate the polynomial matrix for the end conditions part of our matrix
cond = zeros(halfnumbcs, n + 1);
cond(1,:) = 1;

for idx2 = 2:halfnumbcs
    % Determine the next line
    cond(idx2,:) = fliplr((D{idx2-1}*ones(n + 1,1))');
end

% This gives us a matrix of the end conditions with times.
% However, we can't just multiply by a coefficient basis because the
% columns are shifted incrementally by one for each row.
tempendcond = cond.*(ones(halfnumbcs,1)*basis');
startcond = [diag(cond(:,1)) zeros(halfnumbcs)];

% To correct this, we have to shift each row of the endcond
% matrix right by row - 1
endcond = zeros(size(cond));
endcond(1,:) = tempendcond(1,:);
for idx2 = 2:size(endcond,1)
    endcond(idx2,idx2:end) = tempendcond(idx2,1:size(tempendcond,2)-idx2+1);
end

Start_And_End_Cond = [startcond; endcond;];
end

function vec = basisgen(t, n)
% This function generates a basis vector of order n at time t.  It has a
% dimension of n+1 by 1 and takes the form [t^n t^(n-1) ... t 1]'

if isequal(t, 0)
    vec = [1; zeros(n,1)];
else
    vec = zeros(n+1,length(t));
    for idx = 0:n
        vec(idx + 1) = t.^(idx);
    end
end

end
