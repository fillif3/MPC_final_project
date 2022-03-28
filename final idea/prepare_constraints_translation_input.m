function [constraints_A_accelration,constraints_b_accelration]=prepare_constraints_translation_input(horizon,number_of_inputs,max_a)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
constraints_A_accelration=([eye(number_of_inputs*horizon);-eye(number_of_inputs*horizon)]);
constraints_b_accelration=max_a*ones(2*number_of_inputs*horizon,1);
end

