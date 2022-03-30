plot3(trajectory_global(1,:),trajectory_global(2,:),trajectory_global(3,:),'o');
axis([-1 max(goal) -1 max(goal) -1 max(goal)])
hold on
% plot obstacles
[X,Y,Z] = sphere;
for j=obstacles
    o=j{1};
    surf(X*treshold/2+o(1),Y*treshold/2+o(2),Z*treshold/2+o(3))
end
plot3(x_translation_history(1,1:i),x_translation_history(2,1:i),x_translation_history(3,1:i));