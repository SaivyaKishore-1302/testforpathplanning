function distance=pathPhysicalClearance(path,physical)
%PATHPHYSICALCLEARANCE Exact point-to-segment distance, no waypoint tunnelling.
distance=inf;
if isempty(physical), return; end
if size(path,1)==1, distance=min(vecnorm(physical-path,2,2)); return; end
for i=1:size(path,1)-1
    a=path(i,:); d=path(i+1,:)-a;
    t=max(0,min(1,(physical-a)*d'/max(sum(d.^2),eps)));
    distance=min(distance,min(vecnorm(physical-(a+t.*d),2,2)));
end
end
