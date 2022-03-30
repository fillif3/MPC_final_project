function cost = reference_cost_function(inputs,x,waypoint,A,B,Q,R)
number_of_inputs=length(inputs);
size_of_B=size(B);
horizon=number_of_inputs/size_of_B(2);
cost=0;
for i=1:horizon
    u=inputs(i*size_of_B(2)+((-size_of_B(2)+1):0));
    x=A*x+B*u;
    e=x-waypoint;
    cost=cost+e'*Q*e+u'*R*u;
end
end

