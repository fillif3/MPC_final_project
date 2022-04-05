function [A,B,B_noise,C_noise] = get_full_model(Ix,Iy,Iz,g,Ts)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
A=eye(12);
A(1,4)=Ts;
A(2,5)=Ts;
A(3,6)=Ts;
A(4,7)=-g*Ts;
A(5,8)=g*Ts;
B=zeros(12,4);
B(6,1)=Ts;
A(7,10)=Ts;
A(8,11)=Ts;
A(9,12)=Ts;
B(10:12,2:4)=diag([Ix^(-1),Iy^(-1),Iz^(-1)])*Ts;
B_noise=zeros(12,6);
B_noise(4:6,1:3)=eye(3)*Ts;
B_noise(10:12,4:6)=eye(3)*Ts;
C_noise=zeros(6,6);
%C_noise(1:6,5)=ones(6,1);
%C_noise(4:6,6)=ones(3,1);
end