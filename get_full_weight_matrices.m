function [Q_full,R_full,Q_matrix,R_matrix]=get_full_weight_matrices(A,B,Q_scalar,R_scalar,penalized_states,lambda,alpha,horizon,horizon_control)
mi=lambda/alpha;
dimenstion_of_state_vector=length(A);
size_of_B=size(B);
dimenstion_of_input_vector=size_of_B(2);
Q_matrix=diag(penalized_states*Q_scalar);
R_matrix=eye(dimenstion_of_input_vector)*R_scalar;
[P,~,~]=dare(A/alpha,B/alpha,Q_matrix,R_matrix);
Q_matrix_mi=mi^2*Q_matrix+(1-mi^2)*P;
R_matrix_mi=mi^2*R_matrix;
Q_full=Q_matrix_mi;
for i=1:(horizon-1)
    Q_full=blkdiag(Q_full,Q_matrix_mi);
end
R_full=R_matrix_mi;
for i=1:(horizon_control-1)
    R_full=blkdiag(R_full,R_matrix_mi);
end
end