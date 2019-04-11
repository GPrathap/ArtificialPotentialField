function [value,isterminal,direction] = nf_event(t,rob_pos)

global epsilon_gr_nf;
global epsilon_goal;
global epsilon;
global qgoal;

Dummy = [0 0 0];
Dummy(1) = rob_pos(1)-qgoal(1);
Dummy(2) = rob_pos(2)-qgoal(2);
Dummy(3) = rob_pos(3)-qgoal(3);
m=norm(Dummy);

 value = [ (m<=epsilon_goal)*1 ];  

isterminal = [ 1 ]; 
direction = [ 0 ]; 

end