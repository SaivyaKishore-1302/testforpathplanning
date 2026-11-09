function s=movePhysicalCone(s,index,position)
%MOVEPHYSICALCONE Edit the physical scene without erasing perception history.
% The map, current plan, vehicle state and clock are deliberately preserved.
% Subsequent sensor scans discover the change; no selected ID or old/new
% ground-truth coordinates are supplied to association or planning.
validateattributes(index,{'numeric'},{'scalar','integer','positive','<=',size(s.environment.cones.xy,1)});
validateattributes(position,{'numeric'},{'vector','numel',2,'finite','real'});
s.environment.cones.xy(index,:)=reshape(position,1,2);
if isfield(s,'eventOriginalXY') && isfield(s.config.sim,'scenario') && ...
        s.config.sim.scenario.eventIndex==index
    s.eventOriginalXY=reshape(position,1,2);
end
s.nextSensor=min(s.nextSensor,s.time); s.nextPlan=min(s.nextPlan,s.time);
s.nextControl=min(s.nextControl,s.time);
% Older saved interactive states may have terminated after a planning wait.
% Editing cannot undo completion, a collision or the configured time limit.
if s.done && strncmp(s.reason,'Stopped:',8)
    s.done=false; s.lastValid=s.time; s.reason='Scene changed: scanning for a supported route';
end
end
