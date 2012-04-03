function [w_des] = lqrcontrol(s,idx)
% Note: sdes(13:15) contains the desired accelerations

global K params traj log

if ~exist('idx','var')
    lqrdesign
else

    sdes = [...
    traj.pos(idx,:)';
    traj.vel(idx,:)';
    traj.phi(idx); traj.theta(idx); traj.psi(idx);
    0; 0; 0;
    traj.acc(idx,:)'];

% if traj.time(idx) > 2
%     keyboard
% end

    % Principal branch and singularities
    for idx = 7:9
        
        s(idx) = mod(s(idx),2*pi);
        
        % Determine the appropriate K to use
        if isequal(idx, 9)
            degree = round(rad2deg(s(9)));
            if isequal(degree, 0)
                degree = 360;
            end
            Kidx = degree;
        end
        
        % And split at 0
        if s(idx) > pi
            s(idx) = s(idx) - 2*pi;
        end
    end

    % Determine the control
    % u(1) = Fnet, u(2:3) = Moments 1, 2, and 3
    u = -K{Kidx}*(s - sdes(1:12)) + [traj.u1(idx); 0; 0; 0;];

    log.u = [log.u; u'];
    % Feedforward control
%     u = u + traj.u(idx,:)';
    
    % Determine individual rotor forces
    f = ([1 1 1 1; 0 1 0 -1; -1 0 1 0; 1 -1 1 -1]^-1)*u;
    
    % Since we can't actually achieve a negative force for our rotors...
    f = max(0,f);
    
    w_des = sqrt(f./params.kf);

    w_des = max(1200,min(7800,w_des));
    
end

end