function report=recoveryTest()
%RECOVERYTEST Deterministic regression tests for recovery and physical safety.
c=defaultConfig(); x=(0:3:30)';
points=[x,2+zeros(size(x));x,-2+zeros(size(x))];
colour=[ones(size(x));2*ones(size(x))]; moved=6;
damaged=points; damaged(moved,2)=0.6;
r=recoverBoundaryCones(damaged,colour,c.planner);
assert(r.anomaly(moved) && sum(r.anomaly)==1,'Single displaced cone not isolated.');
assert(norm(r.virtual-points(moved,:))<1e-8,'Nominal boundary was not recovered.');
assert(isequal(r.physical,damaged),'Recovery changed the physical obstacle map.');
% Rotation and permutation must not change the result (closed tracks have no
% useful global X ordering). Rotate even beyond 90 degrees.
pose=[20 -7 2.1]; order=[12:22 1:11];
rotated=toWorld(damaged(order,:),pose);
rr=recoverBoundaryCones(rotated,colour(order),c.planner);
assert(sum(rr.anomaly)==1 && norm(rr.virtual-toWorld(points(moved,:),pose))<1e-8);
% No evidence means no invented cone. A clean curved boundary stays intact.
empty=recoverBoundaryCones(damaged(1:3,:),colour(1:3),c.planner);
assert(isempty(empty.virtual));
for kind={'Oval','Hairpin','Figure eight'}
    cc=c; cc.track.type=kind{1}; env=generateTrack(cc);
    for station=0:12:env.length
        xyz=interp1(env.s,env.reference,station);
        keep=vecnorm(env.cones.xy-xyz,2,2)<12;
        clean=recoverBoundaryCones(env.cones.xy(keep,:),env.cones.colour(keep),c.planner);
        assert(~any(clean.anomaly),['False anomaly on clean ' kind{1}]);
    end
end
% Continuous clearance, including an obstacle between sparse waypoints.
[path,ok]=avoidPhysicalCones([0 0;20 0],[10 0.6],c.planner);
assert(ok && max(abs(path(:,2)))>0.2,'Offset path not generated.');
assert(min(projectPolyline([10 0.6],path))>=c.planner.avoidance.clearance-1e-8);
[~,ok]=avoidPhysicalCones([0 0;20 0],[10 -1;10 0;10 1],c.planner);
assert(~ok,'Blocked corridor was incorrectly declared feasible.');
v=struct('x',0,'y',0,'yaw',0,'z',0,'speed',4,'steer',0,'accel',0);
u=struct('steer',0,'accel',2,'target', [10 0],'targetSpeed',4);
obstacle=struct('xy',[3 0],'z',0);
[braking,blocked]=physicalSafetyControl(v,[0 0 0],u,obstacle,c);
assert(blocked && braking.targetSpeed==0 && braking.accel==-c.vehicle.brake, ...
    'Footprint prediction did not command braking before contact.');
% Integration: a displaced physical cone inside a straight open circuit.
c.track.type='Custom'; c.track.closed=false;
c.track.controls=[0 0;20 0;40 0;60 0]; c.sensor.mode='Memory';
c.sensor.sigma=0; c.sensor.distanceNoise=0; c.sensor.colourAccuracy=1;
c.sensor.missRate=0; c.sim.maxTime=40; c.sim.stopOnCollision=true;
track=generateTrack(c); [~,id]=min(vecnorm(track.cones.xy-[18 2],2,2));
track.cones.xy(id,2)=0.6;
state=initializeSimulation(c,track); detected=false; detoured=false; braked=false;
while ~state.done
    state=stepSimulation(state);
    detected=detected || any(state.plan.recovery.anomaly);
    detoured=detoured || contains(state.plan.smoothStatus,'offset route');
    braked=braked || state.safetyBraking;
end
result=summarizeRun(state);
fprintf('Displacement regression: x %.2f m, speed %.2f m/s, %s\n', ...
    state.vehicle.x,state.vehicle.speed,state.reason);
save('verification/recovery_latest.mat','state','result');
assert(detected,'Integrated sensor/map pipeline did not detect displacement.');
assert(detoured,'Integrated planner did not produce avoidance path.');
assert(result.collisionCount==0,'Physical displaced cone collision.');
assert(state.vehicle.x>23,'Car failed to pass the displaced cone.');
% Full obstruction: preserve all physical cones and stop before contact.
blockedTrack=generateTrack(c);
blockedTrack.cones.xy=[blockedTrack.cones.xy;18 -1;18 0;18 1];
blockedTrack.cones.colour=[blockedTrack.cones.colour;1;1;2];
blockedTrack.cones.z=[blockedTrack.cones.z;0;0;0];
blockedTrack.cones.station=[blockedTrack.cones.station;18;18;18];
[blocked,stopped]=runHeadless(c,blockedTrack);
assert(blocked.collisionCount==0 && stopped.vehicle.x<18 && stopped.vehicle.speed<0.05);
report=struct('displacedRun',result,'detected',detected,'detoured',detoured, ...
    'safetyBraked',braked,'blockedRun',blocked);
save('recovery_test_results.mat','report');
disp(report); fprintf('All recovery assertions passed.\n');
end
