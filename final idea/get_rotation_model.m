function [A,B,B_noise]=get_rotation_model(g,Ts,Ix,Iy,Iz)
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here
A=eye(6);
A(1,4)=Ts;
A(2,5)=Ts;
A(3,6)=Ts;
B=zeros(6,3);
B(4:6,1:3)=diag([Ix^(-1),Iy^(-1),Iz^(-1)])*Ts;
B_noise=zeros(6,3);
B_noise(4:6,1:3)=eye(3)*Ts;
end