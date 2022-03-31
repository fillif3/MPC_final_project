function [constraints_A_accelration,constraints_b_accelration]=prepare_constraints_rotation_input(horizon,number_of_inputs,max_angles_a)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
constraints_A_accelration=([eye(number_of_inputs*horizon);-eye(number_of_inputs*horizon)]);
constraints_b_accelration=repmat(eye(3),2*horizon,1)*max_angles_a;
end

