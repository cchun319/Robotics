clear all;
close all;
addpath('data');
stu = load('studentdata1.mat');
t_all = stu.time;

number = size(stu.data, 2);
all_out = zeros(7, number);
v = stu.vicon;
for i = 1: number
    [vel, omg] = estimate_vel(stu.data(i));
    all_out(1:3,i) = vel; 
    all_out(4:6,i) = omg; 
    all_out(7,i) = stu.data(i).t;
end 

[~, ind] = ismember(all_out(7,:), t_all);
v_part = v(7:9, ind);

figure
subplot(3,1,1)
plot(all_out(7,:),v_part(1,:),'g',all_out(7,:),all_out(1,:),'b');
axis([0 inf -2 2])

subplot(3,1,2)
plot(all_out(7,:),v_part(2,:),'g',all_out(7,:),all_out(2,:),'b');
axis([0 inf -2 2])

subplot(3,1,3)
plot(all_out(7,:),v_part(3,:),'g',all_out(7,:),all_out(3,:),'b');
axis([0 inf -2 2])