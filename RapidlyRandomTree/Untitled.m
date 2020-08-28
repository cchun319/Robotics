
   for h = 1 : 1 : path_number
     n3=path(h);
     q3=T_start(n3,:);
     q3=[q3,0,0];
     lynxServo(q3);
     pause(1);
   end