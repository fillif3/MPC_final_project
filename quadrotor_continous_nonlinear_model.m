function derivative = quadrotor_continous_nonlinear_model(u,x,g,m,interiaX,interiaY,interiaZ)
%UNTITLED11 Summary of this function goes here
%   Detaile91d explanation goes here

R=[cos(x(5))*cos(x(6)),sin(x(4))*sin(x(5))*cos(x(6))-cos(x(4))*sin(x(6)),cos(x(4))*sin(x(5))*cos(x(6))-sin(x(4))*sin(x(6));...
    cos(x(5))*sin(x(6)),sin(x(4))*sin(x(5))*sin(x(6))+cos(x(4))*cos(x(6)),cos(x(4))*sin(x(5))*sin(x(6))-sin(x(4))*cos(x(6));...
    -sin(x(5)),sin(x(4))*cos(x(5)),cos(x(4))*cos(x(5))];

derivative(1:3)=R*x(7:9);

R2=[1,sin(x(4))*tan(x(5)),cos(x(4))*tan(x(5));...
    0,cos(x(4)),-sin(x(4));...
    0,sin(x(4))/cos(x(5)),cos(x(4))/cos(x(5))];

derivative(4:6)=R2*x(10:12);

R3=[x(12)*x(8)-x(11)*x(9);
    x(10)*x(9)-x(12)*x(11);
    x(11)*x(7)-x(10)*x(8)];

R4=[-g*sin(x(5));
    g*cos(x(5))*sin(x(4));
    g*cos(x(5))*cos(x(4))];

F=m*(g-u(1));

derivative(7:9)=R3+R4+[0;0;-F]/m;

R5=[(interiaY-interiaZ)/interiaX*x(11)*x(12);
    (interiaZ-interiaX)/interiaY*x(10)*x(12);
    (interiaX-interiaY)/interiaZ*x(11)*x(10)];
R6=[1/interiaX;1/interiaY;1/interiaZ].*u(2:4);

derivative(10:12)=R5+R6;






end