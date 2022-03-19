function [A,B,B_noise]=get_trasnlation_model(g,Ts,m)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
A=eye(6);
A(1,4)=Ts;
A(2,5)=Ts;
A(3,6)=Ts;
B=zeros(6,3);
B(4:6,1:3)=eye(3)*Ts/m;
B_noise=diag([0,0,0,1,1,1])*Ts;
end