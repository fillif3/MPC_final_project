function result = check_observability_linear_quadrotor(Ts,g)
%UNTITLED12 Summary of this function goes here
%   Detailed explanation goes here

A=eye(12);
A(1,7)=Ts;
A(2,8)=Ts;
A(3,9)=Ts;
A(4,10)=Ts;
A(5,11)=Ts;
A(6,12)=Ts;
A(7,5)=-g*Ts;
A(8,4)=g*Ts;
C=eye(12);
C=C(1:6,:);
observ=ctrb(A',C');
result= rank(observ)==12;
end