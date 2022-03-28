for i=1:15
    point=points(:,i);
    min_dist=1;
    for j=obstacles
        min_dist=min(norm(point-j{1}'),min_dist);
    end
end