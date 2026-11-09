function s=stepSimulation(s)
if s.done, return; end
c=s.config; dt=c.sim.dt;
% Optional scene event: perception must discover the changed cone normally.
if isfield(c.sim,'scenario') && s.time>=c.sim.scenario.eventTime && ...
        ~isfield(s,'eventApplied')
    event=c.sim.scenario;
    if event.eventIndex>0
        s.eventOriginalXY=s.environment.cones.xy(event.eventIndex,:);
        s.environment.cones.xy(event.eventIndex,:)=s.environment.cones.xy(event.eventIndex,:)+event.eventOffset;
    end
    s.eventApplied=true;
end
s.vehicle.z=environmentElevation(s.environment,[s.vehicle.x s.vehicle.y],s.vehicle.z);
if s.time+1e-9>=s.nextSensor
    s.observations=simulateSensor(s.environment,s.vehicle,c.sensor,s.time,s.sensorStream);
    remember=strcmpi(c.sensor.mode,'Memory');
    s.map=updateConeMap(s.map,s.observations,s.pose,s.time,c.map,remember);
    s.nextSensor=s.nextSensor+1/c.sensor.frequency;
    if s.recordFrames
        s.frames{end+1,1}=struct('map',s.map,'pose',s.pose,'z',s.vehicle.z, ...
            'time',s.time,'obs',s.observations);
    end
end
planned=false;
if s.time+1e-9>=s.nextPlan
    s.plan=planPath(s.map,s.pose,s.vehicle.z,c.planner,s.plan.worldPath,s.vehicle);
    s.plan.stamp=s.time; s.nextPlan=s.nextPlan+1/c.planner.frequency; planned=true;
    if s.plan.valid, s.lastValid=s.time; end
end
if s.time+1e-9>=s.nextControl
    s.control=purePursuitController(s.vehicle,s.pose,s.plan,c.vehicle,s.time-s.plan.stamp);
    s.nextControl=s.nextControl+1/c.vehicle.controlFrequency;
end
% Recheck each integration step, including fresh scans between control ticks.
[s.control,s.safetyBraking]=physicalSafetyControl(s.vehicle,s.pose,s.control,s.map,c);
old=s.vehicle; s.vehicle=updateVehicle(s.vehicle,s.control,c.vehicle,dt);
movement=[s.vehicle.x-old.x,s.vehicle.y-old.y];
% Odometry integrates local motion; localization random walks are independent
% of cone measurement noise. No correction from hidden reference is applied.
odom=toLocal([s.vehicle.x s.vehicle.y],[old.x old.y old.yaw]);
s.pose(1:2)=toWorld(odom,s.pose);
s.pose(3)=wrapAngle(s.pose(3)+wrapAngle(s.vehicle.yaw-old.yaw));
s.pose(1:2)=s.pose(1:2)+sqrt(dt)*c.localization.positionWalk*randn(s.localizationStream,1,2);
s.pose(3)=wrapAngle(s.pose(3)+sqrt(dt)*c.localization.headingWalk*randn(s.localizationStream));
s.time=s.time+dt; s.distance=s.distance+norm(movement);
s.trajectory(end+1,:)=[s.vehicle.x s.vehicle.y];
s.evaluation=evaluatePerformance(s.environment,s.vehicle,old,s.plan,s.evaluation,c,planned);
e=s.evaluation; f=s.plan.filter;
s.waitingForPlan=~s.plan.valid && s.vehicle.speed<0.03;
if planned
    nmap=size(s.map.xy,1);
    s.log(end+1,:)=[s.time,s.vehicle.x,s.vehicle.y,s.vehicle.speed,s.vehicle.steer, ...
        e.cte,e.pathMean,e.pathMax,e.pathRMS,1000*s.plan.time, ...
        double(s.plan.valid),size(s.observations.xy,1),nmap,numel(s.plan.route.edgeIds), ...
        e.gateValid,sum(f.opposite)];
end
if e.complete && c.sim.stopOnLap, s.done=true; s.reason='Course completed';
elseif c.sim.stopOnCollision && e.collisionCount>0, s.done=true; s.reason='Cone collision';
elseif s.time>=c.sim.maxTime, s.done=true; s.reason='Time limit';
elseif (~isfield(c.sim,'stopOnPlannerFailure') || c.sim.stopOnPlannerFailure) && ...
        s.time-s.lastValid>max(8,c.sensor.outageDuration+2) && s.vehicle.speed<0.03
    s.done=true; s.reason=['Stopped: ' s.plan.status];
elseif s.waitingForPlan, s.reason='Waiting for a supported route; scanning and replanning';
elseif s.safetyBraking, s.reason='Safety braking for perceived physical obstacles';
else, s.reason=s.plan.status;
end
end
