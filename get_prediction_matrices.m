function [F,H] = get_prediction_matrices(A,B,alpha,horizon,horizon_control)
dimenstion_of_state_vector=length(A);
size_of_B=size(B);
dimenstion_of_input_vector=size_of_B(2);
A_alpha=A/alpha;
B_alpha=B/alpha;
F=zeros(dimenstion_of_state_vector*horizon,dimenstion_of_state_vector);
for i=1:horizon
   F(((i-1)*dimenstion_of_state_vector+1):(i*dimenstion_of_state_vector),:)=A_alpha^i;
end
H=zeros(dimenstion_of_state_vector*horizon,horizon_control*dimenstion_of_input_vector);
for i=1:horizon
   for j=1:min(i,horizon_control)
        H(((i-1)*dimenstion_of_state_vector+1):(i*dimenstion_of_state_vector),horizon_control*dimenstion_of_input_vector*(j-1)+(1:3))=A_alpha^(i-j)*B_alpha;
   end
end 
end