function res = check_if_obsrvable_and_controlable(A,B,C)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
dimenstion_of_state_vector=length(A);
con=ctrb(A,B);
obs=ctrb(A',C');
res = (rank(con)==dimenstion_of_state_vector)&&(rank(obs)==dimenstion_of_state_vector);
end