function safe=controlCandidateFeasible(path,physical,c,ego)
%CONTROLCANDIDATEFEASIBLE Follow a candidate with the actual pursuit/bicycle
% model before accepting it near a hazard. Inputs are ego state and map only.
safe=true; if size(path,1)<2, safe=false; return; end
v=ego; v.x=0; v.y=0; v.yaw=0;
plan=struct('valid',true,'worldPath',path,'smoothStatus','Controller rollout', ...
    'speedLimit',1.5,'cautious',true);
for time=0:0.08:1.6 %#ok<NASGU>
    local=toLocal(physical,[v.x v.y v.yaw]);
    dx=max(abs(local(:,1))-c.hybrid.halfLength,0);
    dy=max(abs(local(:,2))-c.hybrid.halfWidth,0);
    if any(hypot(dx,dy)<c.avoidance.coneRadius+c.avoidance.margin)
        safe=false; return;
    end
    command=purePursuitController(v,[v.x v.y v.yaw],plan,c.hybrid.vehicle,0);
    v=updateVehicle(v,command,c.hybrid.vehicle,0.08);
end
end
