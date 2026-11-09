function report=liveMoveTest()
%LIVEMOVETEST Reproduce editing during the moving-cone scene without map loss.
[c,t,meta]=makeScenario('Moving cone event',2685);
c.sim.scenario=meta; c.sim.stopOnPlannerFailure=false; c.sim.maxTime=55;
s=initializeSimulation(c,t);
while s.vehicle.x<14 && ~s.done, s=stepSimulation(s); end
assert(~s.done && ~isempty(s.map.xy));
ids=find(s.environment.cones.colour==2);
[~,j]=min(vecnorm(s.environment.cones.xy(ids,:)-[33 -2],2,2)); index=ids(j);
before=s; s=movePhysicalCone(s,index,[33.27 -3.47]);
assert(isequal(s.map,before.map),'Editing discarded perceived cone history.');
assert(isequal(s.plan,before.plan),'Editing replaced the plan before a sensor update.');
assert(isequal(s.vehicle,before.vehicle) && isequal(s.pose,before.pose));
assert(s.time==before.time && s.distance==before.distance);
assert(isequal(s.observations,before.observations),'Editing fabricated sensor detections.');
editTime=s.time;
while s.vehicle.x<40 && ~s.done, s=stepSimulation(s); end
assert(s.vehicle.x>=40,'Car failed to pass the live-edited section.');
assert(s.evaluation.collisionCount==0 && s.evaluation.violations==0);
edited=summarizeRun(s);
% A real obstruction must stop the vehicle, but must not kill the GUI loop.
[c,t,meta]=makeScenario('Blocked corridor',2685);
c.sim.scenario=meta; c.sim.stopOnPlannerFailure=false; c.sim.maxTime=60;
blocked=initializeSimulation(c,t);
while blocked.time<20 && ~blocked.done, blocked=stepSimulation(blocked); end
assert(~blocked.done && blocked.time>=20,'Planning wait permanently ended the simulation.');
assert(blocked.vehicle.speed<0.05 && blocked.evaluation.collisionCount==0);
waitingTime=blocked.time; map=blocked.map;
N=size(blocked.environment.cones.xy,1);
for k=0:2, blocked=movePhysicalCone(blocked,N-2+k,[18 8+2*k]); end
assert(isequal(blocked.map,map),'Obstacle removal used edit coordinates to alter perception.');
while blocked.vehicle.x<25 && ~blocked.done, blocked=stepSimulation(blocked); end
assert(blocked.vehicle.x>=25,'Vehicle did not recover when the obstruction was removed.');
assert(blocked.evaluation.collisionCount==0 && blocked.evaluation.violations==0);
report=struct('editTime',editTime,'editedRun',edited,'waitedUntil',waitingTime, ...
    'recoveredRun',summarizeRun(blocked));
save('verification/live_move_results.mat','report');
disp(report); fprintf('Live-edit map preservation, safe waiting and automatic recovery passed.\n');
end
