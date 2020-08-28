 % Plots the RRT result
function RRTshowing(map, q_start, q_goal, T_start, lines, path)

    plotmap(map, 5);
    xlabel('x axis');
    ylabel('y axis');
    zlabel('z axis');
    title('RRT (Rapidly-Exploring Random Trees)');
    % imagesc(1 - map);
    % colormap(gray);
    
    hold on;
  
  
countnumber= size(lines,1);
    for j = 1 : 1: countnumber
        point=T_start(j,:);
        point=[point,0];
        [jointPositions,T0e] = calculateFK_sol(point);
        position(1:3,1)=T0e(1:3,4);
        position=position';
        plot3(position(1), position(2), position(3), 'red*', 'linewidth', 1);
        hold on;
        number1=lines(j,1);
        q1=T_start(number1,:);
        q1=[q1,0];
        [jointPositions,Tq] = calculateFK_sol(q1);
        p1(1:3,1)=Tq(1:3,4);
        p1=p1';
  
        number2=lines(j,2);
        q2=T_start(number2,:);
        q2=[q2,0];
        [jointPositions,Tq2] = calculateFK_sol(q2);
        p2(1:3,1)=Tq2(1:3,4);
        p2=p2';
        plot3([p1(1),p2(1)],[p1(2),p2(2)],[p1(3),p2(3)],'red', 'linewidth', 1);
        hold on;  
    end
    q_start=[q_start,0];
    [jointPositions,T01] = calculateFK_sol(q_start);
    p_start(1:3,1)=T01(1:3,4);
    p_start=p_start';
    plot3(p_start(1), p_start(2),p_start(3), 'r*', 'linewidth', 1);
    hold on;
    q_goal=[q_goal,0];
    [jointPositions,T02] = calculateFK_sol(q_goal);
    p_goal(1:3,1)=T02(1:3,4);
    p_goal=p_goal';
    plot3(p_goal(1), p_goal(2),p_goal(3), 'g*', 'linewidth', 3);
   
     pathnumber = size(path,2);
    for h = 1 : 1 : pathnumber - 1
        point=T_start(h,:);
        point=[point,0];
        [jointPositions,T0e] = calculateFK_sol(point);
        position(1:3,1)=T0e(1:3,4);
        position=position';
        plot3(position(1), position(2), position(3), 'red*', 'linewidth', 1);
        hold on;
        n3=path(h);
        q3=T_start(n3,:);
        q3=[q3,0];
        [jointPositions,Tq3] = calculateFK_sol(q3);
        p3(1:3,1)=Tq3(1:3,4);
        p3=p3';
        
        m=h+1;
        n4=path(m);
        q4=T_start(n4,:);
        q4=[q4,0];
        [jointPositions,Tq4] = calculateFK_sol(q4);
        p4(1:3,1)=Tq4(1:3,4);
        p4=p4';
        plot3([p3(1),p4(1)],[p3(2),p4(2)],[p3(3),p4(3)],'black', 'linewidth',3);
        hold on;
    end 
  end