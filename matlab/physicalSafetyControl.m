function [u,blocked]=physicalSafetyControl(v,pose,u,map,c)
%PHYSICALSAFETYCONTROL Predict oriented footprint through reaction + braking.
% Uses perceived physical landmarks only. Includes front/rear overhang and
% steering slew. A rejected manoeuvre commands braking; no teleportation.
blocked=false;
if isempty(map) || isempty(map.xy) || ~c.planner.avoidance.enabled, return; end
points=map.xy(abs(map.z-v.z)<0.9,:);
trial=v; trial.x=pose(1); trial.y=pose(2); trial.yaw=pose(3);
dt=0.04; reaction=0.2; horizon=reaction+max(v.speed,u.targetSpeed)/c.vehicle.brake+0.3;
for time=0:dt:horizon
    p=toLocal(points,[trial.x trial.y trial.yaw]);
    dx=max(abs(p(:,1))-c.vehicle.length/2,0);
    dy=max(abs(p(:,2))-c.vehicle.width/2,0);
    if any(hypot(dx,dy)<c.planner.avoidance.coneRadius+c.planner.avoidance.margin)
        blocked=true; u.accel=-c.vehicle.brake; u.targetSpeed=0; return;
    end
    command=u;
    if time>=reaction, command.accel=-c.vehicle.brake; end
    trial=updateVehicle(trial,command,c.vehicle,dt);
end
end
