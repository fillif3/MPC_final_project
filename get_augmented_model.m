function [A_a,B_a,B_noise_a,C_a]=get_augmented_model(A,B,B_noise,C)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
dimenstion_of_state_vector=length(A);
size_of_C=size(C);
dimenstion_of_output_vector=size_of_C(1);
A_a=eye(dimenstion_of_state_vector+dimenstion_of_output_vector);
A_a(1:dimenstion_of_state_vector,1:dimenstion_of_state_vector)=A;
A_a((dimenstion_of_state_vector+1):(dimenstion_of_state_vector+dimenstion_of_output_vector),1:dimenstion_of_state_vector)=C*A;
B_a=[B;C*B];
B_noise_a=[B_noise;C*B_noise];
C_a=zeros(dimenstion_of_output_vector,dimenstion_of_state_vector+dimenstion_of_output_vector);
C_a(:,(dimenstion_of_state_vector+1):dimenstion_of_state_vector+dimenstion_of_output_vector)=eye(dimenstion_of_output_vector);
end