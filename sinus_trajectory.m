function trajectory = sinus_trajectory(t,state,ts,horizon)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
trajectory=zeros(3,horizon);
for i=1: horizon
    trajectory(1,i)=t;%cos(t);
    trajectory(2,i)=t;%sin(t);
    trajectory(3,i)=t;
    t=t+ts;
end
end