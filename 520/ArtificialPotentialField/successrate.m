% map1
y1 = [0 0.5 -0.3 0 0 0];
y2 = [0 0.5 -1.25 0 0 0];
y3 = [-0.2 0.7 0.5 -1 0 0];
y4 = [0.7 0.5 0.3 -1 0 0];
y5 = [0 0.5 -0.5 0 0 0];

% map 2
s1 = [0 0.5 -0.5 0 0 0];
s2 = [0 0.5 -1.2 0 0 0];
s3 = [-0.8 0.8 -1 0 0 0];
s4 = [-0.2 0.3 0.5 0 0 0];

% map 3
z1 = [-0.8 0.9 -1.2 -0.2 0 0];
z2 = [0 0.2 -0.9 0.5 0 0];
z3 = [0.6 1 -1.1 -0.8 0 0];
z4 = [0.6 1 -0.2 -0.3 0 0];
z5 = [-1 0.8 -0.2 -0.3 0 0];

q_start=z1;
q_final=z3;
testingtime=30;
successtime=0;
failuretime=0;
t=0;
for(k=1: testingtime)
    [time,T_start,q_cool,state]=findpath(q_start,q_final);
    if (time == 0)
        failuretime=failuretime+1;
    else
        t=t+time;
        successtime=successtime+1;
    end
end
averagetime=t/successtime;