function trace=traceBoundary(points,c)
%TRACEBOUNDARY Follow local geometry and heading, not a global coordinate sort.
trace=zeros(0,2); if isempty(points), return; end
cost=vecnorm(points,2,2)+1.5*max(0,-points(:,1));
cost(points(:,1)<-4)=inf;
[value,start]=min(cost); if ~isfinite(value) || value>12, return; end
trace=points(start,:); used=false(size(points,1),1); used(start)=true;
direction=[1 0];
for step=1:35
    delta=points-trace(end,:); distance=vecnorm(delta,2,2);
    unit=delta./max(distance,eps); alignment=unit*direction';
    turn=acos(max(-1,min(1,alignment)));
    score=distance+3*turn.^2;
    score(used | distance<0.35 | distance>c.hybrid.maxTraceGap | ...
        turn>c.hybrid.maxTraceTurn)=inf;
    [value,j]=min(score); if ~isfinite(value), break; end
    trace(end+1,:)=points(j,:); used(j)=true; %#ok<AGROW>
    direction=0.3*direction+0.7*unit(j,:); direction=direction/norm(direction);
end
% Retain actual behind-car context. Extrapolating only the first forward
% segment can otherwise turn a corner ahead into a false wall across the car.
back=points(start,:); direction=[-1 0];
for step=1:6
    delta=points-back(end,:); distance=vecnorm(delta,2,2);
    unit=delta./max(distance,eps); turn=acos(max(-1,min(1,unit*direction')));
    score=distance+3*turn.^2;
    score(used | distance<0.35 | distance>c.hybrid.maxTraceGap | ...
        turn>c.hybrid.maxTraceTurn)=inf;
    [value,j]=min(score); if ~isfinite(value), break; end
    back(end+1,:)=points(j,:); used(j)=true; %#ok<AGROW>
    direction=0.3*direction+0.7*unit(j,:); direction=direction/norm(direction);
end
trace=[flipud(back(2:end,:));trace];
end
