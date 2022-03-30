function cost = global_trajectory_cost_function(inputs,goal,A,B,C,Q,R,horizon,number_of_inputs)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
x=zeros(6,1);
cost=0;
for i=1:horizon
    u=inputs((i-1)*number_of_inputs+(1:3))';
    x=A*x+B*u;
    y=C*x;
    error=goal-y;
    cost=cost+error'*C*Q*C'*error+u'*R*u;

end
end

