function path=planCurvatureEscape(physical,b,c,ego)
%PLANCURVATUREESCAPE Short curvature-bounded alternatives from the actual pose.
% A geometric offset search may be trapped by its reference curve near a cone.
% Sample direct forward arcs and check the full oriented footprint, including
% overhang, at 0.15 m spacing. Runtime braking still checks steering slew.
path=zeros(0,2); if ~b.usable, return; end
maximum=tan(c.hybrid.maxSteer)/c.hybrid.wheelbase;
best=-inf; stations=(0:0.15:6)';
for curvature=linspace(-maximum,maximum,31)
    heading=stations*curvature;
    if abs(curvature)<1e-6, candidate=[stations,zeros(size(stations))];
    else, candidate=[sin(heading)/curvature,(1-cos(heading))/curvature]; end
    allowed=insideBoundaryCorridor(candidate,b,c.avoidance.clearance-0.3);
    for i=1:numel(stations)
        if ~allowed(i), break; end
        local=toLocal(physical,[candidate(i,:),heading(i)]);
        dx=max(abs(local(:,1))-c.hybrid.halfLength,0);
        dy=max(abs(local(:,2))-c.hybrid.halfWidth,0);
        if any(hypot(dx,dy)<c.avoidance.coneRadius+c.avoidance.margin)
            allowed(i:end)=false; break;
        end
    end
    bad=find(~allowed,1); if isempty(bad), count=numel(stations); else, count=bad-1; end
    if count<2 || stations(count)<1.5, continue; end
    candidate=candidate(1:count,:);
    if ~controlCandidateFeasible(candidate,physical,c,ego), continue; end
    score=candidate(end,1)-0.8*projectPolyline(candidate(end,:),b.centre)-1.5*abs(curvature);
    if score>best, best=score; path=candidate; end
end
end
