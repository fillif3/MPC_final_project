function [Q_full,R_full,Q_matrix,R_matrix]=get_full_weight_matrices(A,B,Q_scalar,R_scalar,penalized_states,horizon)
size_of_B=size(B);
dimenstion_of_input_vector=size_of_B(2);
Q_matrix=diag(penalized_states*Q_scalar);
R_matrix=eye(dimenstion_of_input_vector)*R_scalar;
Q_full=Q_matrix;
for i=1:(horizon-1)
    Q_full=blkdiag(Q_full,Q_matrix);
end
R_full=R_matrix;
for i=1:(horizon-1)
    R_full=blkdiag(R_full,R_matrix);
end
end