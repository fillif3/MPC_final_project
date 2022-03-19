function cost = reference_cost_function(u,x0,trajectory,F,H,Q,R,horizon)
number_of_states=length(x0);
x=zeros(number_of_states*horizon,1);
for i=1:horizon
    x((1:number_of_states)+(number_of_states)*(i-1),:)=F((i-1)*number_of_states+(1:number_of_states),:)*(x0-trajectory(1:number_of_states,i))+H((i-1)*number_of_states+(1:number_of_states),:)*u;
end
cost=x'*Q*x+u'*R*u;
end

