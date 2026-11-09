function s=initializeSimulation(c,track)
if nargin<1, c=defaultConfig(); end
if nargin<2, track=generateTrack(c); end
c.map.sensorRange=c.sensor.range; c.map.sensorFov=c.sensor.fov;
c.planner.hybrid.wheelbase=c.vehicle.wheelbase; c.planner.hybrid.maxSteer=c.vehicle.maxSteer;
c.planner.hybrid.halfLength=c.vehicle.length/2; c.planner.hybrid.halfWidth=c.vehicle.width/2;
c.planner.hybrid.vehicle=c.vehicle;
% Keep avoidance dimensions consistent with Settings and programmatic changes.
c.planner.avoidance.clearance=c.vehicle.width/2+c.planner.avoidance.coneRadius+ ...
    c.planner.avoidance.margin+c.planner.avoidance.trackingMargin;
c.planner.avoidance.maxOffset=max(0,c.planner.width/2-c.planner.avoidance.clearance);
assert(c.sim.dt>0 && c.sim.dt<=0.1,'Timestep must be in (0,0.1].');
assert(c.sensor.frequency>0 && c.planner.frequency>0 && c.vehicle.controlFrequency>0);
assert(c.sim.dt<=1/max([c.sensor.frequency,c.planner.frequency,c.vehicle.controlFrequency])+1e-9, ...
    'Integration timestep must resolve the highest configured frequency.');
s=struct(); s.config=c; s.environment=track; s.time=0; s.distance=0;
s.vehicle=struct('x',track.start(1),'y',track.start(2),'yaw',track.start(3), ...
    'z',track.z(1),'speed',0,'steer',0,'accel',0);
s.pose=track.start; s.map=[];
s.sensorStream=RandStream('mt19937ar','Seed',c.seed+1);
s.localizationStream=RandStream('mt19937ar','Seed',c.seed+2);
s.observations=struct('xy',zeros(0,2),'colour',zeros(0,1),'sigma',zeros(0,1),'z',zeros(0,1));
s.plan=planPath([],s.pose,s.vehicle.z,c.planner,zeros(0,2));
s.control=struct('steer',0,'accel',0,'target',nan(1,2),'targetSpeed',0);
s.nextSensor=0; s.nextPlan=0; s.nextControl=0;
s.safetyBraking=false;
s.waitingForPlan=false;
s.trajectory=[s.vehicle.x s.vehicle.y];
s.done=false; s.reason='Ready'; s.lastValid=0; s.frames={}; s.recordFrames=false;
s.evaluation=struct('progress',0,'cte',0,'lap',0,'complete',false, ...
    'collisionCount',0,'hitCones',false(size(track.cones.xy,1),1), ...
    'violations',0,'outside',false,'pathMean',NaN,'pathMax',NaN,'pathRMS',NaN, ...
    'gateValid',NaN,'reconstructionSamples',zeros(0,1));
s.log=zeros(0,16);
end
