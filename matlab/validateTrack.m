function validateTrack(t,c)
if min(t.width)<c.vehicle.width+0.8, error('FS:Track','Track is too narrow for this vehicle.'); end
% Dense boundary polygon: reject nonadjacent intersections on the same deck.
for side=1:2
    if side==1, p=t.left; else, p=t.right; end
    ids=unique([1:4:size(p,1),size(p,1)]); p=p(ids,:); z=t.z(ids);
    for i=1:size(p,1)-1
        for j=i+2:size(p,1)-1
            if t.closed && i==1 && j==size(p,1)-1, continue; end
            if abs(z(i)-z(j))<1 && segmentsCross(p(i,:),p(i+1,:),p(j,:),p(j+1,:))
                error('FS:Track','Boundary self-intersection. Increase turn radius or reduce width.');
            end
        end
    end
end
% Boundaries must also not intersect each other.
ids=unique([1:5:size(t.left,1),size(t.left,1)]);
for i=1:numel(ids)-1
    for j=1:numel(ids)-1
        if abs(t.z(ids(i))-t.z(ids(j)))<1 && segmentsCross( ...
                t.left(ids(i),:),t.left(ids(i+1),:),t.right(ids(j),:),t.right(ids(j+1),:))
            error('FS:Track','Opposite boundaries overlap.');
        end
    end
end
p=t.cones.xy;
for i=1:size(p,1)-1
    d=vecnorm(p(i+1:end,:)-p(i,:),2,2);
    sameDeck=abs(t.cones.z(i+1:end)-t.cones.z(i))<1;
    if any(d(sameDeck)<0.45), error('FS:Track','Overlapping cones; adjust spacing or control points.'); end
end
end
