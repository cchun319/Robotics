function [ desired_state ] = trajectory_generator(t, qn, mapp, path)
% TRAJECTORY_GENERATOR: Turn a Dijkstra or A* path into a trajectory
%
% NOTE: This function would be called with variable number of input
% arguments. At first, it will be called with arguments
% trajectory_generator([], [], map, path) and later, in test_trajectory,
% it will be called with only t and qn as arguments, so your code should
% be able to handle that. This can be done by checking the number of
% arguments to the function using the "nargin" variable, check the
% MATLAB documentation for more information.
%
% parameters:
%   map: The map structure returned by the load_map function
%   path: This is the path returned by your planner (dijkstra function)
%   desired_state: Contains all the information that is passed to the
%                  controller, as in Phase 1
%
% NOTE: It is suggested to use "persistent" variables to store map and path
% during the initialization call of trajectory_generator, e.g.
% persistent map0 path0
% map0 = map;
% path0 = path;

persistent via_pts;
persistent res;
persistent ch;

% avel = [0.5,0.6,1.3,0.8];
avel = [1,1,1,0.7];
if isempty(t)
    via_pts = path;
    num_session = size(via_pts ,1);
    v_dir = diff(via_pts);
    d = sqrt(sum(v_dir.^2,2));
    total_d = sum(d);
%     if total_d > 76
%         t_each = 1 * d;
%         ch = 1;
%     elseif total_d > 35 && total_d < 45
%         t_each = 1 * d;
%         ch = 2;
%     elseif total_d > 10 && total_d < 12
%         t_each = 1 * d;
%         ch = 3;
%     else 
%         t_each = avel(4) * d;
%         ch = 4;
%     end
    t_each = avel(4) * d;
    time = [0; cumsum(t_each, 1)];
    cons = zeros( 8*(num_session - 1) ,3);
    pos_c = 5: 8: 8*num_session - 4;
    pos_c2 = pos_c + 1;
    pos_c2(end) = [];
    cons(1,:) = via_pts(1,:);
    cons(pos_c, :) = via_pts(2:end, :);
    cons(pos_c2, :) = via_pts(2:end - 1, :);
    
    
    t_ma = zeros(8 * (length(time) - 1), 8 * (length(time) - 1) );
    
    t_ma(1:4,1:8 ) = [time(1)^7, time(1)^6, time(1)^5, time(1)^4, time(1)^3, time(1)^2, time(1), 1;
        7*time(1)^6, 6*time(1)^5, 5*time(1)^4, 4*time(1)^3, 3*time(1)^2, 2*time(1), 1, 0;
        42*time(1)^5, 30*time(1)^4, 20*time(1)^3, 12*time(1)^2, 6*time(1), 2, 0, 0;
        210*time(1)^4, 120*time(1)^3, 60*time(1)^2, 24*time(1), 6, 0, 0, 0];
    
    t_ma(end - 3 :end,end - 7:end ) = [time(end)^7, time(end)^6, time(end)^5, time(end)^4, time(end)^3, time(end)^2, time(end), 1;
        7*time(end)^6, 6*time(end)^5, 5*time(end)^4, 4*time(end)^3, 3*time(end)^2, 2*time(end), 1, 0;
        42*time(end)^5, 30*time(end)^4, 20*time(end)^3, 12*time(end)^2, 6*time(end), 2, 0, 0;
        210*time(end)^4, 120*time(end)^3, 60*time(end)^2, 24*time(end), 6, 0, 0, 0];
    
    ro = 1: 8: 8*(num_session - 2);
    
    for i = 1 : num_session - 2
        t_ma(pos_c(i): pos_c(i)+7, ro(i): ro(i) + 15) = [time(i+1)^7, time(i+1)^6, time(i+1)^5, time(i+1)^4, time(i+1)^3, time(i+1)^2, time(i+1), 1,zeros(1,8);
            zeros(1,8),time(i+1)^7, time(i+1)^6, time(i+1)^5, time(i+1)^4, time(i+1)^3, time(i+1)^2, time(i+1), 1;
            7*time(i+1)^6, 6*time(i+1)^5, 5*time(i)^4, 4*time(i+1)^3, 3*time(i+1)^2, 2*time(i+1), 1, 0,...
            -7*time(i+1)^6, -6*time(i+1)^5, -5*time(i+1)^4, -4*time(i+1)^3, -3*time(i+1)^2, -2*time(i+1), -1, 0;
            42*time(i+1)^5, 30*time(i+1)^4, 20*time(i+1)^3, 12*time(i+1)^2, 6*time(i+1), 2, 0, 0,...
            -42*time(i+1)^5, -30*time(i+1)^4, -20*time(i)^3, -12*time(i+1)^2, -6*time(i+1), -2, 0, 0;
            210*time(i+1)^4, 120*time(i+1)^3, 60*time(i+1)^2, 24*time(i+1), 6, 0, 0, 0,...
            -210*time(i+1)^4, -120*time(i+1)^3, -60*time(i+1)^2, -24*time(i+1), -6, 0, 0, 0;
            840*time(i+1)^3, 360*time(i+1)^2, 120*time(i+1), 24, 0, 0, 0, 0,...
            -840*time(i+1)^3, -360*time(i+1)^2, -120*time(i+1), -24, 0, 0, 0, 0;
            2520*time(i+1)^2, 720*time(i+1), 120, 0, 0, 0, 0, 0,...
            -2520*time(i+1)^2, -720*time(i+1), -120, 0, 0, 0, 0, 0;
            5040*time(i+1), 720, 0, 0, 0, 0, 0, 0,...
            -5040*time(i+1), -720, 0, 0, 0, 0, 0, 0];
    end
    res = pinv(t_ma) * cons;
end

num_session = size(via_pts ,1);

pos = [0; 0; 0];
vel = [0; 0; 0];
acc = [0; 0; 0];

v_dir = diff(via_pts);
d = sqrt(sum(v_dir.^2,2));
% t_each = avel(ch) * d; % 1m/3sec;
t_each = avel(4) * d; % 1m/3sec;

time = [0; cumsum(t_each, 1)];

c_x = res(:,1);
c_y = res(:,2);
c_z = res(:,3);


for i = 1 : num_session - 1
    if isempty(t) == 0
        if (t > time(i) && t <= time(i+1))
            t_now = t;
            
            pos_x = [t_now^7, t_now^6, t_now^5, t_now^4, t_now^3, t_now^2, t_now, 1] * c_x(8*(i-1)+1:8*i);
            pos_y = [t_now^7, t_now^6, t_now^5, t_now^4, t_now^3, t_now^2, t_now, 1] * c_y(8*(i-1)+1:8*i);
            pos_z = [t_now^7, t_now^6, t_now^5, t_now^4, t_now^3, t_now^2, t_now, 1] * c_z(8*(i-1)+1:8*i);
            
            vel_x = [7*t_now^6, 6*t_now^5, 5*t_now^4, 4*t_now^3, 3*t_now^2, 2*t_now, 1, 0] * c_x(8*(i-1)+1:8*i);
            vel_y = [7*t_now^6, 6*t_now^5, 5*t_now^4, 4*t_now^3, 3*t_now^2, 2*t_now, 1, 0] * c_y(8*(i-1)+1:8*i);
            vel_z = [7*t_now^6, 6*t_now^5, 5*t_now^4, 4*t_now^3, 3*t_now^2, 2*t_now, 1, 0] * c_z(8*(i-1)+1:8*i);
            
            acc_x = [42*t_now^5, 30*t_now^4, 20*t_now^3, 12*t_now^2, 6*t_now, 2, 0, 0] * c_x(8*(i-1)+1:8*i);
            acc_y = [42*t_now^5, 30*t_now^4, 20*t_now^3, 12*t_now^2, 6*t_now, 2, 0, 0] * c_y(8*(i-1)+1:8*i);
            acc_z = [42*t_now^5, 30*t_now^4, 20*t_now^3, 12*t_now^2, 6*t_now, 2, 0, 0] * c_z(8*(i-1)+1:8*i);
            
            pos = [pos_x; pos_y; pos_z];
            vel = [vel_x; vel_y; vel_z];
            acc = [acc_x; acc_y; acc_z];
            
        elseif t > time(end, 1)
            pos = via_pts(end,:)';
            vel = [0; 0; 0];
            acc = [0; 0; 0];
        end
    end
    
end

yaw = 0;
yawdot = 0;

desired_state = [];

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end