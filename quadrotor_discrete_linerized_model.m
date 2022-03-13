function xPlus = quadrotor_discrete_linerized_model(u,x,Ts,g,interiaX,interiaY,interiaZ)
%UNTITLED12 Summary of this function goes here
%   Detailed explanation goes here

A=eye(12);
A(1,7)=Ts;
A(2,8)=Ts;
A(3,9)=Ts;
A(4,10)=Ts;
A(5,11)=Ts;
A(6,12)=Ts;
A(7,5)=-g*Ts;
A(8,4)=g*Ts;
B=zeros(12,1);
B(9:12,1:4)=diag([1,interiaX^(-1),interiaY^(-1),interiaZ^(-1)])'*Ts;
xPlus=A*x+B*u;
end