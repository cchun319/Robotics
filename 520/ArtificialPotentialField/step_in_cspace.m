function [deltaq]=step_in_cspace(q1,ftotal)
J = calcJacobianjoint(q1);
 for(i=1:5)
     delta(i,:)=(J(:,:,i)')*(ftotal(i,:)');
 end
 deltaq=sum(delta,1);
end
 