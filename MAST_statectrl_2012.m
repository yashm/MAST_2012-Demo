clear all
close all
clear states

% profile on -history

addpath(genpath('/Users/yashm/Desktop/GRASP/state_control'));

global Midi
global traj

Midi = midictrl('init');
fclose('all');
if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

sim_on = 0;
pickup_failed =0;

cruising_height = 1.5; %1.6

window_len = 1;
% window_pos = [0.745 1.24 cruising_height];
height_above_crawler1 = .3;
height_above_crawler2 = .16;
height_above_crawler3 = .26;
height_above_perch1 = .3;
height_above_perch2 = .03;

if sim_on
    freq = 100;
    quadnames = {'1'};
    
    objnames = ['crawler_dummy'];
    init_pos2 = [];
    log_file = '/Users/yashm/Desktop/GRASP/state_control/logs/sim_Joppa2012_statectrl_test_1';
    
    len = 1;
    window_pos = [0 0 cruising_height];
    plot3(window_pos(1)+len/2*[-1 -1 1 1 -1],window_pos(2)*ones(1,5),window_pos(3)+len/2*[-1 1 1 -1 -1])
    axis equal
    takeoff = [-2 -1 0 0];
    axis([-2 2 -2 2 0 3]);
    grid on
    xlabel('x')
    ylabel('y')
    objnames = {'crawler_dummy'};
    crawler_pos = [1 -1.5 0];
    init_pos2 = [crawler_pos 0];
    window_front = window_pos + [0 -1 0]*window_len;
    window_back = window_pos + [0 1 0]*window_len;
    crawler_dropoff = [1.95 2.27 .3915];
    staging_pt = [-.66 2.11 cruising_height];
    perch_pt = [-.66 1.24 2.48 pi/2];
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end_traj_position = [window_front 0];
    crawler_pickup = [0 0 1 0];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
else
    
    quadnames = {'bravo'};
    %     objnames = [];
    log_file = '/Users/yashm/Desktop/GRASP/state_control/logs/Joppa2012_statectrl_test_1';
    objnames = {'crawler_new'};
    
    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

setgains

speed = 0.5;
accel = 1.0;

height_above_takeoff = .09;

t_inf = 1e6;
%%%%%%%%% BRAVO %%%%%%%%%%%%%
opengripperpos = 9800;
closegripperpos = 4900;
%%%%%%%%%%%%%%%%%%%%%%


% %%%%%%%%% MIKE %%%%%%%%%%%%%
% opengripperpos = 5100;
% closegripperpos = 10000;
% %%%%%%%%%%%%%%%%%%%%%%

gripper_open = [1 opengripperpos t_inf ];
gripper_close = [1 closegripperpos t_inf ];

restpos = [-2.0 0.0];
endpos = [2.0 0.0];

if sim_on
    
    for i=1:length(quadnames)
        quads(i) = start_quad(quadnames{i});
        curr_state{i} = init_state_tester(quads(i),takeoff);
        
    end
    
    for i=1:length(objnames)
        objects(i) = start_object(objnames{i});
        obj_state{i} = init_state_tester(objects(i),init_pos2(i,:));
    end
else
    vicon = start_vicon;
    freq = vicon.freq;
    for i=1:length(quadnames)
        
        quads(i) = start_quad(quadnames{i},vicon);
        curr_state{i} = init_curr_state_vicon(quads(i),vicon);
    end
    
    for i=1:length(objnames)
        objects(i) = start_object(objnames{i},vicon);
        obj_state{i} = init_object_vicon(objects(i),vicon);
    end
    
end
obj_exist = exist('objects','var');

if sim_on
    for i=1:length(quads)
        start_pt(i,:) = [curr_state{i}.x_est curr_state{i}.y_est curr_state{i}.z_est curr_state{i}.psi];
        %         hover_pt(i,:) = start_pt(i,:) + [0 0 .05 0];
        %         hover_pt(i,4) = [-pi/2];
        %         hover_pt2(i,:) = start_pt(i,:) + [0 0 .05 0];
        %         hover_pt2(i,4) = [-pi/2];
        %         perch_pt(i,:) = start_pt(i,:);
        %         perch_pt(i,4) = [-pi/2];
    end
else
    idx = find(ismember({objects.name},'crawler_new'),1);
    crawler_pos = [obj_state{idx}.x_est obj_state{idx}.y_est obj_state{idx}.z_est  obj_state{idx}.psi];
    
    %     idx = find(ismember({objects.name},'Window'),1);
    %     window_pos = [obj_state{idx}.x_est obj_state{idx}.y_est obj_state{idx}.z_est  obj_state{idx}.psi];
    
    %     idx = find(ismember({objects.name},'Crawler_Mat'),1);
    %     perch_pt = [obj_state{idx}.x_est obj_state{idx}.y_est obj_state{idx}.z_est obj_state{idx}.psi];
    
    %     perch_pt = [-2.1126 1.2652 2.3950 pi/2];
    perch_pt = [-0.6367 1.2617 2.4780 pi/2];
    
    window_pos = [0.7145 1.2491 cruising_height pi/2];
    
    crawler_dropoff = [0.7156 2.2482 0.2663];
    %     perch_pt = [2 1.5 0.6];
    staging_pt = [perch_pt(1:2) 2.8];
    
    window_front = window_pos - [cos(window_pos(4)) sin(window_pos(4)) 0 0]*window_len;
    window_front = window_front - [0 0.2 0 0];
    window_back = window_pos + [cos(window_pos(4)) sin(window_pos(4)) 0 0]*window_len;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end_traj_position = [window_front(1:2) cruising_height pi/2];
    crawler_pickup = [crawler_pos(1:2) 0.26 0];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    for i=1:length(quads)
        takeoff(i,:) = [curr_state{i}.x_est curr_state{i}.y_est curr_state{i}.z_est curr_state{i}.psi];
        %         hover_pt(i,:) = start_pt(i,:) + [0 0 .05 0];
        %         hover_pt(i,4) = [-pi/2];
        %         hover_pt2(i,:) = start_pt(i,:) + [0 0 .05 0];
        %         hover_pt2(i,4) = [-pi/2];
        %         perch_pt(i,:) = start_pt(i,:);
        %         perch_pt(i,4) = [-pi/2];
    end
    
    
end


% start_pt = [-1.5 1 .7 0];


takeoff_angle = atan2(crawler_pos(2)-takeoff(2),crawler_pos(1)-takeoff(1));
dropoff_angle = atan2(crawler_dropoff(2)-window_back(2),crawler_dropoff(1)-window_back(1));
window_angle = atan2(window_front(2)-crawler_pos(2),window_front(1)-crawler_pos(1));
through_window_angle = atan2(window_back(2)- window_front(2),window_back(1)- window_front(1));


start_slide = @(curr_state,states,Midi_idx, signal,state) midi_select(curr_state,states,Midi,4,'on');
advance_slide = @(curr_state,states,Midi_idx, signal,state) midi_select(curr_state,states,Midi,3,'on');

phi_45 = @(curr_state) (curr_state.phi>pi/4) || (curr_state.phi<-pi/4);
psi_error = @(curr_state,states,target,max_err,operator)angle_error(curr_state,states, target, max_err, operator);


for i=1:length(quads)
    quads(i).task = 'start';
    states{i} = create_state(@do_nothing,gains,'servo',gripper_open,@n_plus,1);
    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Trajectory Parameters
saferegion = [-2 -3 0.2 1.2 0.3 3];
%     end_traj_position = [1, -.5, cruising_height, -pi/2];
attackangle = atan2(end_traj_position(2)-takeoff(2),end_traj_position(1)-takeoff(1)) - 0; % -pi/2;
startpos = [takeoff(1:2) cruising_height attackangle];
%     crawler_pickup = [crawler_pickup -pi/2];

% Generate the Trajectory
% GOOD!!!    traj = Trajectory_Generator(saferegion, startpos, [crawler_pickup(1) crawler_pickup(2)+0.03 crawler_pickup(3)+0.11 attackangle], end_traj_position, speed);

% for 3*speed, offset = 0.11 ; 2* = 0.12
traj = Trajectory_Generator(saferegion, startpos, [crawler_pickup(1) crawler_pickup(2) crawler_pickup(3) attackangle], [end_traj_position], 1.2, 0.1);

% Run a dynamic simulation
if sim_on
    Dynamic_Sim();
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


while (midi_select(curr_state,states,Midi,9,'off'))
    
    for i=1:length(quads)
        
        finished = (curr_state{i}.state_n == length(states{i}));
        
        switch(quads(i).task)
            case 'start' % safely ascend to crusing height
                if finished && midi_select(curr_state,states,Midi,4,'on')
                    states{i}(1) = create_state(@do_nothing,gains,...
                        'servo',gripper_open,@n_plus,1, start_slide,1,@n_plus,1);
                    
                    states{i}(end+1) = create_state(@xyz_vel, gains, [takeoff(1:2) (takeoff(3)+height_above_takeoff)],...
                        'servo',gripper_open,'speed',speed,'accelrate',accel,@n_plus,1);
                    
                    states{i}(end+1) = create_state(@xyz_hover, gains,[takeoff(1:2) (takeoff(3)+height_above_takeoff)],...
                        'servo',gripper_open,@n_plus,1, @ec_timer , 1,@n_plus,1);
                    
                    states{i}(end+1) = create_state(@xyz_vel, gains, [takeoff(1:2) cruising_height],...
                        'servo',gripper_open, 'speed',speed,'accelrate',accel,@n_plus,1);
                    
                    states{i}(end+1) = create_state(@xyz_hover, gains,[takeoff(1:2) cruising_height],...
                        'servo',gripper_open,@n_plus,1, @ec_timer , 1,@n_plus,1);
                    
                    states{i}(end+1) = create_state(@rotate_at_xyz, gains,[takeoff(1:2) cruising_height attackangle], ...
                        'servo',gripper_open,'yawrate',0.2,@n_plus,1,psi_error,attackangle,deg2rad(5),'<=',@n_plus,1);
                    
                    states{i}(end+1) = create_state(@xyz_hover, gains,[takeoff(1:2) cruising_height],...
                        'servo',gripper_open,@n_plus,1, @ec_timer , 1,@n_plus,1);
                    
                    quads(i).task = 'crawler_pickup';
                    
                end
                
            case 'crawler_pickup' % Go to crawler, pick it up and wait in front of the window
                
                if (finished) && pickup_failed
                    % do xyz_traj again
                    'PICKUP FAILED !!!' %#ok<NOPTS>
                    quads(i).task = 'crawler_pickup';
                    continue
                end
                
                if (finished)
                    % this will be go thru window sequence
                    
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    
                    % Establish correct yaw
                    states{i}(end+1) = create_state(@rotate_at_xyz, gains, [takeoff(1:2) cruising_height attackangle], ...
                        'servo',gripper_close,'yawrate', 0.2, @n_plus,1,psi_error,attackangle,deg2rad(5),'<=',@n_plus,1);
                    
                    states{i}(end+1) = create_state(@xyz_hover, gains,[takeoff(1:2) cruising_height attackangle],...
                        'servo',gripper_close,@n_plus,1,advance_slide, 1,@n_plus,1);
                    
                    % Create the Swooping Trajectory states
                    states{i}(end+1) = create_state(@xyz_traj_J_working, gains, 'traj', traj, 'feedforward',true,'servo',gripper_close,@n_plus, 1);
                    
                    % Use some soft gains to stabilize
                    states{i}(end+1) = create_state(@xyz_hover, gains, [end_traj_position], 'servo',gripper_close,@n_plus,1, @ec_timer,1,@n_plus,1);
                    %     ^ IDEALLY should be
                    %     states{i}(end+1) = create_state(@xyz_hover, soft_gains, [], 'servo',gripper_close,@n_plus,1, @ec_timer,1,@n_plus,1);
                    
                    
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    
                    states{i}(end+1) = create_state(@xyz_vel, gains, [window_front(1:2) cruising_height],...
                        'servo',gripper_close, 'speed',speed,'accelrate',accel,@n_plus,1);
                    
                    states{i}(end+1) = create_state(@xyz_hover, gains,[window_front(1:2) cruising_height],...
                        'servo',gripper_close,@n_plus,1, @ec_timer , 2,@n_plus,1);
                    
                    states{i}(end+1) = create_state(@rotate_at_xyz, gains,[window_front(1:2) cruising_height through_window_angle], ...
                        'servo',gripper_close,'yawrate',0.2,@n_plus,1,psi_error,through_window_angle,deg2rad(5),'<=',@n_plus,1);
                    
                    states{i}(end+1) = create_state(@xyz_hover, gains, [window_front(1:2) cruising_height through_window_angle],...
                        'servo',gripper_close,@n_plus,1, @ec_timer, 1,@n_plus,1);
                    
                    quads(i).task = 'go_thru_window';
                end
                
            case 'go_thru_window' % Go throught the window
                if (finished)
                    states{i}(end+1) = create_state(@xyz_vel, gains, [window_back(1:2) cruising_height through_window_angle],...
                        'servo',gripper_close,'speed',speed,'accelrate',accel,@n_plus,1);
                    
                    states{i}(end+1) = create_state(@xyz_hover, gains, [window_back(1:2) cruising_height through_window_angle],...
                        'servo',gripper_close,@n_plus,1, @ec_timer , 2,@n_plus,1);
                    
                    quads(i).task = 'crawler_dropoff';
                end
                
            case 'crawler_dropoff' % Make the dropoff
                
                if (finished)
                    
                    states{i}(end+1) = create_state(@xyz_vel, gains, [crawler_dropoff(1:2) (crawler_dropoff(3)+height_above_crawler3)],...
                        'servo',gripper_close,'speed',speed,'accelrate',accel,@n_plus,1);
                    
                    states{i}(end+1) = create_state(@xyz_hover, gains, [crawler_dropoff(1:2) (crawler_dropoff(3)+height_above_crawler3)],...
                        'servo',gripper_open,@n_plus,1, @ec_timer , 1,@n_plus,1);
                    
                    states{i}(end+1) = create_state(@xyz_vel, gains, [crawler_dropoff(1:2) staging_pt(3)],...
                        'servo',gripper_open,'speed',speed,'accelrate',accel,@n_plus,1);
                    
                    states{i}(end+1) = create_state(@xyz_hover, gains, [crawler_dropoff(1:2) staging_pt(3)],...
                        'servo',gripper_open,@n_plus,1, @ec_timer , 1,@n_plus,1);
                    
                    quads(i).task = 'perch';
                end
                
            case 'perch' % Go to staging point and perch
                
                if (finished)
                    
                    %move to staging point
                    states{i}(end+1) = create_state(@xyz_vel, gains, [staging_pt],...
                        'servo',gripper_open,'speed',speed,'accelrate',accel,@n_plus,1);
                    states{i}(end+1) = create_state(@xyz_hover, gains, [staging_pt],...
                        'servo',gripper_open,@n_plus,1, @ec_timer , 1,@n_plus,1);
                    
                    %ascend to above perch height above staging point
                    
                    %                     states{i}(end+1) = create_state(@xyz_vel, gains, [staging_pt(1:2) perch_pt(3)+height_above_perch1],...
                    %                         'servo',gripper_open,'speed',speed,'accelrate',accel,@n_plus,1);
                    %                     states{i}(end+1) = create_state(@xyz_hover, gains, [],...
                    %                         'servo',gripper_open,@n_plus,1, @ec_timer , 1,@n_plus,1);
                    
                    %move to above perch point
                    
                    states{i}(end+1) = create_state(@xyz_vel, gains, [perch_pt(1:2) perch_pt(3)+height_above_perch1],...
                        'servo',gripper_open,'speed',speed,'accelrate',accel,@n_plus,1);
                    
                    states{i}(end+1) = create_state(@xyz_hover, gains, [perch_pt(1:2) perch_pt(3)+height_above_perch1],...
                        'servo',gripper_open,@n_plus,1, @ec_timer , 2,@n_plus,1);
                    
                    %rotate in place
                    states{i}(end+1) = create_state(@rotate_at_xyz, gains, perch_pt(4), ...
                        'servo',gripper_open,'yawrate',0.1,@n_plus,1,psi_error,perch_pt(4),deg2rad(5),'<=',@n_plus,1);
                    
                    states{i}(end+1) = create_state(@xyz_hover, gains, [perch_pt(1:2) perch_pt(3)+height_above_perch1],...
                        'servo',gripper_open,@n_plus,1, @ec_timer , 2,@n_plus,1);
                    
                    %descend to slightly above perch point
                    
                    states{i}(end+1) = create_state(@xyz_vel, gains, [perch_pt(1:2) perch_pt(3)+height_above_perch2 perch_pt(4)],...
                        'servo',gripper_open,'speed',0.5*speed,'accelrate',accel,@n_plus,1);
                    states{i}(end+1) = create_state(@xyz_hover, gains, [perch_pt(1:2) perch_pt(3)+height_above_perch2 perch_pt(4)],...
                        'servo',gripper_open,@n_plus,1, @ec_timer , 2,@n_plus,1);
                    
                    %descend to perch height
                    states{i}(end+1) = create_state(@xyz_vel, gains, perch_pt+[0 0 -0.05 0],...
                        'servo',gripper_open,'speed',0.1,'accelrate',accel,@n_plus,1,@robust_landing,deg2rad(5),@n_minus,5);
                    
                    %do nothing until advance slide
                    states{i}(end+1) = create_state(@do_nothing,gains,...
                        'servo',gripper_open,@n_plus,1, advance_slide,1,@n_plus,1,@robust_landing,deg2rad(5),@n_minus,6);
                    quads(i).task = 'return_to_home_pos';
                    
                end
                
            case 'vicon_crawler_pickup' % after advance slide, go back to the crawler and pick it up
                
                if finished
                    
                    states{i}(end+1) = create_state(@xyz_vel, gains, [perch_pt(1:2) perch_pt(3)+height_above_perch2+.03],...
                        'servo',gripper_open, 'speed',speed,'accelrate',accel,@n_plus,1);
                    
                    states{i}(end+1) = create_state(@xyz_hover, gains, [perch_pt(1:2) perch_pt(3)+height_above_perch2+.03],...
                        'servo',gripper_open,@n_plus,1, @ec_timer , 2,@n_plus,1);
                    
                    %%%%%%%%%%%%%%%%*** CLEAR LEDGE BEFORE GOING TO CRAWLER ***%%%%%%%%%%%%%%%%%%
                    states{i}(end+1) = create_state(@xyz_vel, gains, [crawler_dropoff(1) crawler_dropoff(2) staging_pt(3)],...
                        'servo',gripper_open, 'speed',speed,'accelrate',accel,@n_plus,1);
                    
                    states{i}(end+1) = create_state(@xyz_hover, gains, [staging_pt(1)-0.3 staging_pt(2)-0.5 staging_pt(3)],...
                        'servo',gripper_open,@n_plus,1, @ec_timer , 2,@n_plus,1);
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    
                    states{i}(end+1) = create_state(@xyz_vel, gains, [crawler_dropoff(1:2) cruising_height],...
                        'servo',gripper_open,'speed',1.5*speed,'accelrate',accel,@n_plus,1);
                    
                    states{i}(end+1) = create_state(@xyz_vel, gains, [crawler_dropoff(1:2) (crawler_dropoff(3)+ height_above_crawler1)],...
                        'servo',gripper_open,'speed',speed,'accelrate',accel,@n_plus,1);
                    
                    states{i}(end+1) = create_state(@xyz_hover, gains, [],...
                        'servo',gripper_open,@n_plus,1, @ec_timer , 1,@n_plus,1);
                    
                    states{i}(end+1) = create_state(@xyz_vel, gains, [crawler_dropoff(1:2) (crawler_dropoff(3)+ height_above_crawler2)],...
                        'servo',gripper_open,'speed',speed,'accelrate',accel,@n_plus,1);
                    
                    states{i}(end+1) = create_state(@xyz_hover, gains, [crawler_pos(1:2) (crawler_pos(3)+ height_above_crawler2)],...
                        'servo',gripper_close,@n_plus,1, @ec_timer, 2,@n_plus,1);
                    
                    quads(i).task = 'come_out_from_window';
                    
                end
                
            case 'come_out_from_window' % come out of the window and drop crawler off
                
                if (finished)
                    
                    states{i}(end+1) = create_state(@xyz_hover, gains, [crawler_pos(1:2) (crawler_pos(3)+ height_above_crawler2)],...
                        'servo',gripper_close,@n_plus,1, @ec_timer, 2,@n_plus,1);
                    
                    states{i}(end+1) = create_state(@xyz_vel, gains, [window_back(1:2) cruising_height],...
                        'servo',gripper_close,'speed',speed,'accelrate',accel,@n_plus,1);
                    
                    states{i}(end+1) = create_state(@xyz_hover, gains,[window_back(1:2) cruising_height],...
                        'servo',gripper_close,@n_plus,1, @ec_timer, 2,@n_plus,1);
                    
                    states{i}(end+1) = create_state(@xyz_vel, gains, [window_front(1:2) cruising_height],...
                        'servo',gripper_close,'speed',speed,'accelrate',accel,@n_plus,1);
                    
                    states{i}(end+1) = create_state(@xyz_hover, gains,[window_front(1:2) cruising_height],...
                        'servo',gripper_close,@n_plus,1, @ec_timer, 2,@n_plus,1);
                    
                    states{i}(end+1) = create_state(@xyz_vel, gains, [crawler_pos(1:2) (crawler_pos(3)+ height_above_crawler2)],...
                        'servo',gripper_close,'speed',speed,'accelrate',accel,@n_plus,1);
                    
                    states{i}(end+1) = create_state(@xyz_hover, gains, [crawler_pos(1:2) (crawler_pos(3)+ height_above_crawler2)],...
                        'servo',gripper_close,@n_plus,1, @ec_timer, 2,@n_plus,1);
                    
                    states{i}(end+1) = create_state(@xyz_hover, gains, [],...
                        'servo',gripper_close,@n_plus,1, @ec_timer , 2,@n_plus,1);
                    
                    quads(i).task = 'return_to_home_pos';
                end
                
                
            case 'return_to_home_pos' % Go back to start and end routine
                
                if (finished)
                    
                    %                     states{i}(end+1) = create_state(@xyz_hover, gains, [],...
                    %                         'servo',gripper_open,@n_plus,1, @ec_timer , 2,@n_plus,1);
                    
                    states{i}(end+1) = create_state(@xyz_vel, gains, [perch_pt(1:2) perch_pt(3)+height_above_perch1],...
                        'servo',gripper_open,'speed',speed,'accelrate',accel,@n_plus,1);
                    states{i}(end+1) = create_state(@xyz_hover, gains, [perch_pt(1:2) perch_pt(3)+height_above_perch1],...
                        'servo',gripper_open,@n_plus,1, @ec_timer , 1,@n_plus,1);
                    
                    
                    states{i}(end+1) = create_state(@xyz_vel, gains, [takeoff(1:2) perch_pt(3)+height_above_perch1],...
                        'servo',gripper_open,'speed',speed,'accelrate',accel,@n_plus,1);
                    states{i}(end+1) = create_state(@xyz_hover, gains, [takeoff(1:2) perch_pt(3)+height_above_perch1],...
                        'servo',gripper_open,@n_plus,1, @ec_timer , 1,@n_plus,1);
                    
                    %descend to slightly above takeoff height
                    
                    states{i}(end+1) = create_state(@xyz_vel, gains, [takeoff(1:2) (takeoff(3)+height_above_takeoff)],...
                        'servo',gripper_open,'speed',1.5*speed,'accelrate',accel,@n_plus,1);
                    states{i}(end+1) = create_state(@xyz_hover, gains, [takeoff(1:2) (takeoff(3)+height_above_takeoff)],...
                        'servo',gripper_open,@n_plus,1, @ec_timer ,0.1,@n_plus,1);
                    
                    states{i}(end+1) = create_state(@xyz_vel, gains,takeoff(1:3)+[0 0 -0.05] ,...
                        'servo',gripper_open,'speed',0.1,'accelrate',accel,@n_plus,1);
                    
                    states{i}(end+1) = create_state(@do_nothing,gains,...
                        'servo',gripper_open,@n_no,1);
                    
                    quads(i).task = 'finish';
                end
                
            case 'finish' %Waiting until bin is clear
                
                
                if (finished) %%%%%%%%%%%%% TAKE CREATE STATE OUT...
                    states{i}(end+1) = create_state(@xyz_hover, gains, [],...
                        'servo',gripper_open,@n_plus,1, @ec_timer , 1,@n_plus,1);
                end
        end
        
    end
    
    
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if sim_on
        if obj_exist
            [entity_state states]=run_states({quads objects},{curr_state obj_state}, states, {@sim_update @sim_object_update}, {1/freq 1/freq}, log_file);
            curr_state = entity_state{1};
            obj_state = entity_state{2};
        else
            [curr_state states]=run_states({quads []},curr_state, states, @sim_update, 1/freq, log_file);
        end
    else
        if obj_exist
            [entity_state states]=run_states({quads objects},{curr_state obj_state}, states, {@vicon_update @vicon_object_update}, {vicon vicon}, log_file);
            curr_state = entity_state{1};
            obj_state = entity_state{2};
        else
            [curr_state states]=run_states({quads []},curr_state, states, @vicon_update, vicon, log_file);
        end
    end
    
    
    %     [obj_state{1}.x_est obj_state{1}.y_est obj_state{1}.z_est obj_state{1}.psi]
    for i=1:length(quads)
        display(sprintf('quad %s in state %s @ (%2.4f %2.4f %2.4f %2.4f) says %s',quads(i).name,quads(i).task,...
            curr_state{i}.x_est,curr_state{i}.y_est,curr_state{i}.z_est,curr_state{i}.psi,curr_state{i}.txt))
    end
    %     display(sprintf('setpoints height: %2.4f velocity: %2.4f\n',set_height,set_velocity))
    
end

clean_states
if ~sim_on
    stop_quad(quads)
    stop_vicon(vicon)
end


midictrl('close',Midi);
