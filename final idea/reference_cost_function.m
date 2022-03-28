function cost = reference_cost_function(u,x0,F,H,Q,R)
x=F*x0+H*u;
cost=x'*Q*x+u'*R*u;
end

