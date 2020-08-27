function plotrobot(map,p_start,p_final,q_cool,T_start)

number=size(q_cool,1);
lynxStart()
plotmap(map,1);
hold on;
xlabel('x axis');
ylabel('y axis');
zlabel('z axis');
 scatter3(p_start(5,1), p_start(5,2), p_start(5,3),'red','filled');
 hold on;
 pause 
% scatter3(p_final(5,1), p_final(5,2), p_final(5,3),'green','filled');
% hold on;
for j=2:(number-1)
   plotLynx(q_cool(j,:));
   hold on;
   if j < number-1
   scatter3(T_start((j+1),1),T_start((j+1),2),T_start((j+1),3),'black','filled');
    hold on;
   end
    plot3([T_start(j,1),T_start((j+1),1)],...
    [T_start(j,2),T_start((j+1),2)],[[T_start(j,3),T_start((j+1),3)]],'blue', 'linewidth', 1);
    hold on;
end
 plotLynx(q_cool(end,:));
end
