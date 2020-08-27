function [X, Z] = ekf1(sensor, vic, varargin)
% EKF1 Extended Kalman Filter with Vicon velocity as inputs
%
% INPUTS:
%   sensor - struct stored in provided dataset, fields include
%          - is_ready: logical, indicates whether sensor data is valid
%          - t: sensor timestamp
%          - rpy, omg, acc: imu readings
%          - img: uint8, 240x376 grayscale image
%          - id: 1xn ids of detected tags
%          - p0, p1, p2, p3, p4: 2xn pixel position of center and
%                                four corners of detected tags
%            Y
%            ^ P3 == P2
%            | || P0 ||
%            | P4 == P1
%            o---------> X
%   vic    - struct for storing vicon linear velocity in world frame and
%            angular velocity in body frame, fields include
%          - t: vicon timestamp
%          - vel = [vx; vy; vz; wx; wy; wz]
%   varargin - any variables you wish to pass into the function, could be
%              a data structure to represent the map or camera parameters,
%              your decision. But for the purpose of testing, since we don't
%              know what inputs you will use, you have to specify them in
%              init_script by doing
%              ekf1_handle = ...
%                  @(sensor, vic) ekf1(sensor, vic, your input arguments);
%
% OUTPUTS:
% X - nx1 state of the quadrotor, n should be greater or equal to 6
%     the state should be in the following order
%     [x; y; z; qw; qx; qy; qz; other states you use]
%     we will only take the first 7 rows of X
% OPTIONAL OUTPUTS:
% Z - mx1 measurement of your pose estimator, m shoulb be greater or equal to 7
%     the measurement should be in the following order
%     [x; y; z; qw; qx; qy; qz; other measurement you use]
%     note that this output is optional, it's here in case you want to log your
%     measurement
persistent t_pre

t_cd = -[0.04, 0.0, 0.03]';
rad = 44.5 * pi / 180;

id = sensor.id;

if isempty(id)
    pos = [];
    q = [];
    return;
end

u1 = sensor.p1';
u2 = sensor.p2';
u3 = sensor.p3';
u4 = sensor.p4';
u5 = sensor.p0';

R_cd = [cos(rad), -sin(rad), 0;
    -sin(rad), -cos(rad), 0;
    0, 0, -1];

K = [311.0520 0        201.8724;
    0        311.3885 113.6210;
    0        0        1];

tag_id = [0, 12, 24, 36, 48, 60, 72, 84,  96;
    1, 13, 25, 37, 49, 61, 73, 85,  97;
    2, 14, 26, 38, 50, 62, 74, 86,  98;
    3, 15, 27, 39, 51, 63, 75, 87,  99;
    4, 16, 28, 40, 52, 64, 76, 88, 100;
    5, 17, 29, 41, 53, 65, 77, 89, 101;
    6, 18, 30, 42, 54, 66, 78, 90, 102;
    7, 19, 31, 43, 55, 67, 79, 91, 103;
    8, 20, 32, 44, 56, 68, 80, 92, 104;
    9, 21, 33, 45, 57, 69, 81, 93, 105;
    10, 22, 34, 46, 58, 70, 82, 94, 106;
    11, 23, 35, 47, 59, 71, 83, 95, 107];

id_ind = zeros(length(id),2);
for i = 1 : length(id)
    [r, c] = find(id(i) == tag_id);
    id_ind(i,:) = [c, r];
end

u = [u1;u2;u3;u4;u5];

p = size(u,1);

x_num = [0, ones(1,17)];
x_gap = 0.152 * x_num;
x_gap(:,[7,13]) = 0.178;
x_pos = cumsum(x_gap)';
x_cen = x_pos(1:end-1,:) + 0.152/2;

y_num = [0, ones(1,23)];
y_gap = 0.152 * y_num;
y_pos = cumsum(y_gap)';
y_cen = y_pos(1:end-1,:) + 0.152/2;

p1_ind = [2*id_ind(:,1) - 1, 2*id_ind(:,2)];
p2_ind = [2*id_ind(:,1), 2*id_ind(:,2)];
p3_ind = [2*id_ind(:,1), 2*id_ind(:,2) - 1];
p4_ind = [2*id_ind(:,1) - 1, 2*id_ind(:,2) - 1];
p5_ind = p4_ind;

p_ind = [p1_ind; p2_ind; p3_ind; p4_ind];

p_pos = [y_pos(p_ind(:,2)), x_pos(p_ind(:,1));y_cen(p5_ind(:,2)), x_cen(p5_ind(:,1))];

x = p_pos;

u = [u, ones(size(u,1),1)];
x = [x, ones(size(x,1),1)];

A = zeros(2*size(u,1), 9);
A1_p = 1:2:2*p;
A2_p = 2:2:2*p;

A1 = [x, zeros(size(x,1),3), -u(:,1).*x];
A2 = [zeros(size(x,1),3), x, -u(:,2).*x];
A(A1_p,:) = A1;
A(A2_p,:) = A2;

[~,~,V1] = svd(A);
h_col = V1(:,9);

h = reshape(h_col,3,3)';

h = h / h(3,3);

r_temp = K \ h;

r_build = [r_temp(:,1), r_temp(:,2), cross(r_temp(:,1), r_temp(:,2))];

[U, ~, V] = svd(r_build);

r_cw = U * [1,0,0;
    0,1,0;
    0,0,det(U*V')] *V';

average = 0.5* (norm(r_temp(:,1)) + norm(r_temp(:,2)));

T = r_temp(:,3) / average; % pos of c in

R_wd = r_cw' * R_cd;

T_wd = r_cw' * ( t_cd - T);

pos = T_wd';
q = rotm2quat(R_wd);

X = zeros(7,1);
Z = zeros(7,1);

%% prediciton

phi_s = q(1);
the_s = q(2);
sci_s = q(3);

qToR = [cos(sci_s)*cos(the_s)- sin(phi_s)*sin(sci_s)*sin(the_s), - cos(phi_s)*sin(sci_s), cos(sci_s)*sin(the_s)+cos(the_s)*sin(phi_s)*sin(sci_s);...
    cos(the_s)*sin(sci_s)+ cos(sci_s)*sin(phi_s)*sin(the_s), cos(phi_s)*cos(sci_s), sin(sci_s)*sin(the_s)-cos(sci_s)*cos(the_s)*sin(phi_s);...
    -cos(phi_s)*sin(the_s), sin(phi_s), cos(phi_s)*cos(the_s)];

G = [cos(the_s), 0, -cos(phi_s)*sin(the_s);...
    0, 1, sin(phi_s);...
    sin(the_s), 0, cos(phi_s)*cos(the_s)];

f_x = [ 0, 0, 0,                                                                                                                                                                                                                                0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        0,                                                                                                                                                                                                                                                                        0, 0, 0, 0;
    0, 0, 0,                                                                                                                                                                                                                                0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        0,                                                                                                                                                                                                                                                                        0, 0, 0, 0;
    0, 0, 0,                                                                                                                                                                                                                                0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        0,                                                                                                                                                                                                                                                                        0, 0, 0, 0;
    0, 0, 0,                                                                                                                                                                                                                                0, (cos(conj(the_s))^2*sin(conj(phi_s))*conj(bi_2) + cos(conj(the_s))^2*sin(conj(phi_s))*conj(nsy_g) - cos(conj(the_s))^2*sin(conj(phi_s))*conj(oy_m) + sin(conj(phi_s))*sin(conj(the_s))^2*conj(bi_2) + sin(conj(phi_s))*sin(conj(the_s))^2*conj(nsy_g) - sin(conj(phi_s))*sin(conj(the_s))^2*conj(oy_m) + cos(conj(phi_s))*cos(conj(the_s))*conj(bi_3) + cos(conj(phi_s))*cos(conj(the_s))*conj(nsz_g) - cos(conj(phi_s))*cos(conj(the_s))*conj(oz_m) - cos(conj(phi_s))*sin(conj(the_s))*conj(bi_1) - cos(conj(phi_s))*sin(conj(the_s))*conj(nsx_g) + cos(conj(phi_s))*sin(conj(the_s))*conj(ox_m))/(cos(conj(phi_s))*(cos(conj(the_s))^2 + sin(conj(the_s))^2)) - (sin(conj(phi_s))*(cos(conj(phi_s))*cos(conj(the_s))^2*conj(nsy_g) - cos(conj(phi_s))*cos(conj(the_s))^2*conj(oy_m) + cos(conj(phi_s))*sin(conj(the_s))^2*conj(bi_2) + cos(conj(phi_s))*sin(conj(the_s))^2*conj(nsy_g) - cos(conj(phi_s))*sin(conj(the_s))^2*conj(oy_m) - cos(conj(the_s))*sin(conj(phi_s))*conj(bi_3) - cos(conj(the_s))*sin(conj(phi_s))*conj(nsz_g) + cos(conj(the_s))*sin(conj(phi_s))*conj(oz_m) + sin(conj(phi_s))*sin(conj(the_s))*conj(bi_1) + sin(conj(phi_s))*sin(conj(the_s))*conj(nsx_g) - sin(conj(phi_s))*sin(conj(the_s))*conj(ox_m) + cos(conj(phi_s))*cos(conj(the_s))^2*conj(bi_2)))/(cos(conj(phi_s))^2*(cos(conj(the_s))^2 + sin(conj(the_s))^2)), -(sin(conj(phi_s))*(cos(conj(the_s))*conj(bi_3) + cos(conj(the_s))*conj(nsz_g) - cos(conj(the_s))*conj(oz_m) - sin(conj(the_s))*conj(bi_1) - sin(conj(the_s))*conj(nsx_g) + sin(conj(the_s))*conj(ox_m)))/(cos(conj(phi_s))^2*(cos(conj(the_s))^2 + sin(conj(the_s))^2)), 0, 0, 0;
    0, 0, 0, -(cos(conj(the_s))*conj(bi_3) + cos(conj(the_s))*conj(nsz_g) - cos(conj(the_s))*conj(oz_m) - sin(conj(the_s))*conj(bi_1) - sin(conj(the_s))*conj(nsx_g) + sin(conj(the_s))*conj(ox_m))/(cos(conj(the_s))^2 + sin(conj(the_s))^2),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                -(cos(conj(the_s))*sin(conj(phi_s))*conj(bi_1) + cos(conj(the_s))*sin(conj(phi_s))*conj(nsx_g) - cos(conj(the_s))*sin(conj(phi_s))*conj(ox_m) + sin(conj(phi_s))*sin(conj(the_s))*conj(bi_3) + sin(conj(phi_s))*sin(conj(the_s))*conj(nsz_g) - sin(conj(phi_s))*sin(conj(the_s))*conj(oz_m))/(cos(conj(phi_s))*(cos(conj(the_s))^2 + sin(conj(the_s))^2)),                       (cos(conj(the_s))*conj(bi_1) + cos(conj(the_s))*conj(nsx_g) - cos(conj(the_s))*conj(ox_m) + sin(conj(the_s))*conj(bi_3) + sin(conj(the_s))*conj(nsz_g) - sin(conj(the_s))*conj(oz_m))/(cos(conj(phi_s))*(cos(conj(the_s))^2 + sin(conj(the_s))^2)), 0, 0, 0;
    0, 0, 0,                                                                                                                                                                                                                                0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        0,                                                                                                                                                                                                                                                                        0, 0, 0, 0;
    0, 0, 0,                                                                                                                                                                      -cos(conj(the_s))/(cos(conj(the_s))^2 + sin(conj(the_s))^2),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        -(sin(conj(phi_s))*sin(conj(the_s)))/(cos(conj(phi_s))*(cos(conj(the_s))^2 + sin(conj(the_s))^2)),                                                                                                                                                                                            sin(conj(the_s))/(cos(conj(phi_s))*(cos(conj(the_s))^2 + sin(conj(the_s))^2)), 0, 0, 0;
    0, 0, 0,                                                                                                                                                                                                                                0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                -(cos(conj(phi_s))*cos(conj(the_s))^2 + cos(conj(phi_s))*sin(conj(the_s))^2)/(cos(conj(phi_s))*(cos(conj(the_s))^2 + sin(conj(the_s))^2)),                                                                                                                                                                                                                                                                        0, 0, 0, 0;
    0, 0, 0,                                                                                                                                                                      -sin(conj(the_s))/(cos(conj(the_s))^2 + sin(conj(the_s))^2),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         (cos(conj(the_s))*sin(conj(phi_s)))/(cos(conj(phi_s))*(cos(conj(the_s))^2 + sin(conj(the_s))^2)),                                                                                                                                                                                           -cos(conj(the_s))/(cos(conj(phi_s))*(cos(conj(the_s))^2 + sin(conj(the_s))^2)), 0, 0, 0];

f_u = [ 1, 0, 0,                                                          0,                                                                                                                                        0,                                                                              0, 0, 0, 0;
    0, 1, 0,                                                          0,                                                                                                                                        0,                                                                              0, 0, 0, 0;
    0, 0, 1,                                                          0,                                                                                                                                        0,                                                                              0, 0, 0, 0;
    0, 0, 0, cos(conj(the_s))/(cos(conj(the_s))^2 + sin(conj(the_s))^2),                                         (sin(conj(phi_s))*sin(conj(the_s)))/(cos(conj(phi_s))*(cos(conj(the_s))^2 + sin(conj(the_s))^2)), -sin(conj(the_s))/(cos(conj(phi_s))*(cos(conj(the_s))^2 + sin(conj(the_s))^2)), 0, 0, 0;
    0, 0, 0,                                                          0, (cos(conj(phi_s))*cos(conj(the_s))^2 + cos(conj(phi_s))*sin(conj(the_s))^2)/(cos(conj(phi_s))*(cos(conj(the_s))^2 + sin(conj(the_s))^2)),                                                                              0, 0, 0, 0;
    0, 0, 0, sin(conj(the_s))/(cos(conj(the_s))^2 + sin(conj(the_s))^2),                                        -(cos(conj(the_s))*sin(conj(phi_s)))/(cos(conj(phi_s))*(cos(conj(the_s))^2 + sin(conj(the_s))^2)),  cos(conj(the_s))/(cos(conj(phi_s))*(cos(conj(the_s))^2 + sin(conj(the_s))^2)), 0, 0, 0];

f_n = [ -1,  0,  0,                                                           0,                                                                                                                                         0,                                                                              0, 0, 0, 0;
    0, -1,  0,                                                           0,                                                                                                                                         0,                                                                              0, 0, 0, 0;
    0,  0, -1,                                                           0,                                                                                                                                         0,                                                                              0, 0, 0, 0;
    0,  0,  0, -cos(conj(the_s))/(cos(conj(the_s))^2 + sin(conj(the_s))^2),                                         -(sin(conj(phi_s))*sin(conj(the_s)))/(cos(conj(phi_s))*(cos(conj(the_s))^2 + sin(conj(the_s))^2)),  sin(conj(the_s))/(cos(conj(phi_s))*(cos(conj(the_s))^2 + sin(conj(the_s))^2)), 0, 0, 0;
    0,  0,  0,                                                           0, -(cos(conj(phi_s))*cos(conj(the_s))^2 + cos(conj(phi_s))*sin(conj(the_s))^2)/(cos(conj(phi_s))*(cos(conj(the_s))^2 + sin(conj(the_s))^2)),                                                                              0, 0, 0, 0;
    0,  0,  0, -sin(conj(the_s))/(cos(conj(the_s))^2 + sin(conj(the_s))^2),                                          (cos(conj(the_s))*sin(conj(phi_s)))/(cos(conj(phi_s))*(cos(conj(the_s))^2 + sin(conj(the_s))^2)), -cos(conj(the_s))/(cos(conj(phi_s))*(cos(conj(the_s))^2 + sin(conj(the_s))^2)), 0, 0, 0;
    1,  0,  0,                                                           0,                                                                                                                                         0,                                                                              0, 0, 0, 0;
    1,  0,  0,                                                           0,                                                                                                                                         0,                                                                              0, 0, 0, 0;
    1,  0,  0,                                                           0,                                                                                                                                         0,                                                                              0, 0, 0, 0];

%% update



end
