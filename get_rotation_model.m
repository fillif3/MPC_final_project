function [A,B,B_noise]=get_rotation_model(g,Ts,Ix,Iy,Iz)
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here
A=eye(6);
A(1,4)=Ts;
A(2,5)=Ts;
A(3,6)=Ts;
A(7,5)=-g*Ts;
A(8,4)=g*Ts;
B=zeros(6,3);
B(4:6,1:3)=diag([Ix^(-1),Iy,Iz^(-1)])*Ts;
B_noise=diag([0,1,0,1,0,1])*Ts;
end