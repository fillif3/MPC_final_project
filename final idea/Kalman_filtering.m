function [x_estimated,P_estimated] = Kalman_filtering(u,x,y,P,A,B,C,Q,R)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
x_predicted=A*x+B*u;
P_predicted=A*P*A'+Q;
K=P_predicted*C'*inv(C*P_predicted*C'+R);
x_estimated=x_predicted+K*(y-C*x_predicted);
P_estimated=(eye(length(x))-K*C)*P_predicted;
end