 function RRTfailureshowing(map, q_start, q_goal, T_start, lines)
    plotmap(map, 5);
    title('RRT failure showing');
    % imagesc(1 - map);
    % colormap(gray);
    
    hold on;
  
    countnumber= size(lines,1);
    
    for j = 1 : countnumber
        point=T_start(j,:);
        point=[point,0];
        [jointPositions,T0e] = calculateFK_sol(point);
        position(1:3,1)=T0e(1:3,4);
        position=position';
        plot3(position(1), position(2), position(3), 'redo', 'linewidth', 1);
  
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
        plot3([p1(1),p2(1)],[p1(2),p2(2)],[p1(3),p2(3)],'red', 'linewidth', 4);
      end
    q_start=[q_start,0];
    [jointPositions,T01] = calculateFK_sol(q_start);
    p_start(1:3,1)=T01(1:3,4);
    p_start=p_start';
    plot3(p_start(1), p_start(2),p_start(3), 'g*', 'linewidth', 1);
   
    q_goal=[q_goal,0];
    [jointPositions,T02] = calculateFK_sol(q_goal);
    p_goal(1:3,1)=T02(1:3,4);
    p_goal=p_goal';
    plot3(p_goal(1), p_goal(2),p_goal(3), 'g*', 'linewidth', 1);
     pathnumber = size(path,2);
 end