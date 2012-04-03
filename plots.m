global log traj

% scrsz = get(0,'ScreenSize');

% Toggle plotting of dynamic simulation
if ~isfield(log,'F') || isempty(log.F)
    sim = false;
else
    sim = true;
end

%% Positions
figure
hold on
if sim
    plot(time(1:100:end),states(1:100:end,1),'or',time(1:100:end),states(1:100:end,2),'xg',time(1:100:end),states(1:100:end,3),'.b');
end;
% set(gcf,'position',scrsz);
idx = min(length(traj.time),length(traj.pos));
plot(traj.time(1:idx),traj.pos(1:idx,1),'-.r');
plot(traj.time(1:idx),traj.pos(1:idx,2),'-.g');
plot(traj.time(1:idx),traj.pos(1:idx,3),'-.b');
% plotyy(0,0,log.time(1:100:end),log.F(1:100:end));
grid on
title('Position')
xlabel('time (s)')
ylabel('Position')
legend('x','y','z');
hold off

pause
close

%% Roll, Pitch, and Yaw

if sim
    figure
    hold on
    plot(time,states(:,7),'r');
    plot(time,states(:,8),'g');
    plot(time,states(:,9),'b');
    % set(gcf,'position',scrsz);
    title('phi, theta, psi')
    plot([time(1) time(end)],[params.maxAngle params.maxAngle],'-.k');
    plot([time(1) time(end)],[-params.maxAngle -params.maxAngle],'-.k');
    plot(traj.time,traj.phi,'-.r')
    plot(traj.time,traj.theta,'-.g')
    plot(traj.time,traj.psi,'-.b')
    grid on
    xlabel('time (s)')
    ylabel('Angle (rad)')
    hold off
    
    pause
    close
end

%% Propeller Speeds

if sim
    figure
    hold on
    grid on
    % w = log.w;
    % plot(w(:,1))
    % plot(w(:,2))
    % plot(w(:,3))
    % plot(w(:,4))
    plot(time,states(:,13),'r')
    plot(time,states(:,14),'g')
    plot(time,states(:,15),'b')
    plot(time,states(:,16),'m')
    % set(gcf,'position',scrsz);
    title('w1, w2, w3, w4')
    xlabel('time (s)')
    ylabel('RPM')
    axis([time(1) time(end) 1200 7800]);
    hold off
    
    pause
    close
end

%% 3D plot of actual, desired, and angles
figure
hold on
grid on
if sim
    plot3(states(:,1),states(:,2),states(:,3));
end
title(['x, y, z position;  Total Time: ',num2str(tf), ' seconds']);
xlabel('x')
ylabel('y')
zlabel('z')

view(-161,16);

%% Overlay orientation

if sim
    
    ctimes = 0:time(end)/20:time(end);
    
    % x, y, z as rows so that we can rotate them
    a = params.armlength;
    square = [...
        a  0 -a  0;
        0 -a  0  a;
        0  0  0  0];
    
    cmap = colormap(jet);
    allangles = states(:,7:9);
    maxangle = max(max(abs(allangles)));
    
    for idx = 1:length(ctimes)
        keyidx = find(time >= ctimes(idx),1,'first');
        angles = states(keyidx,7:9);
        
        Rot = EulToRot(angles(1),angles(2),angles(3));
        Rotlevel = EulToRot(0, 0, angles(3));
        
        % Columns of x, y, and z
        vertices = (Rot*square)';
        
        % Now we need to shift to the trajectory
        shift = ones(4,1)*[states(keyidx,1) states(keyidx,2) states(keyidx,3)];
        
        % Store the points
        pts(:,:,idx) = vertices + shift;
        
        attitude_functional(idx) = 1/2*trace(eye(3) - Rotlevel'*Rot);
    end
    
    % Now plot the patches representing the quadrotor
    for idx = 1:length(ctimes)
        cindex = max(1,round(attitude_functional(idx)/max(attitude_functional)*64));
        patch(pts(:,1,idx),pts(:,2,idx),pts(:,3,idx),cmap(cindex,:));
    end
    
    disp(['Maximum Force: ', num2str(max(log.F(10:end))), ' N']);
end

plot3(traj.pos(:,1),traj.pos(:,2),traj.pos(:,3),'-.k');

axis equal

pause
close

if sim
    diffs = bsxfun(@minus,traj.pickup(1:3), states(:,1:3));
    errors = sqrt(sum(diffs.^2,2));
    [C, I] = min(errors);
    disp(['Error at pickup: ', num2str(errors(I)), ' meters']);
end

% print(fig,'-dpng','image.png');