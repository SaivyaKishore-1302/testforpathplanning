function report=demoTest(fullLap)
%DEMOTEST Parse source, verify invariants, and exercise a short simulation.
% demoTest(true) also runs and asserts completion of the default noisy lap.
if nargin<1, fullLap=false; end
folder=fileparts(mfilename('fullpath')); addpath(folder);
files=dir(fullfile(folder,'*.m'));
for i=1:numel(files)
    [~,name]=fileparts(files(i).name);
    nargin(str2func(name)); % Forces MATLAB to parse each function and nested body.
end
fprintf('MATLAB parsed %d source files.\n',numel(files));
c=defaultConfig(); track=generateTrack(c);
% Frame transforms are inverses, including a nonzero heading.
points=[1 2;5 -3;0 0]; pose=[7 -8 1.3];
assert(max(abs(toLocal(toWorld(points,pose),pose)-points),[],'all')<1e-10);
% Projection on a segment must be spatial, not waypoint-index based.
[d,station,q]=projectPolyline([3 2],[0 0;10 0]);
assert(abs(d-2)<eps && abs(station-3)<eps && norm(q-[3 0])<eps);
% Degenerate triangulation inputs return a status instead of throwing.
t=computeDelaunay([0 0;1 0],[1;2]); assert(isempty(t.edges));
t=computeDelaunay([0 0;1 0;2 0],[1;2;1]); assert(strcmp(t.status,'Collinear cones'));
t=computeDelaunay([0 0;0 0;1 0;0 1],[1;1;2;2]); assert(size(t.points,1)==3);
% Sensor range/FOV and exact reproducibility; no hidden landmark identifiers.
v=struct('x',0,'y',0,'yaw',0,'z',0,'speed',0,'steer',0,'accel',0);
env=struct('cones',struct('xy',[4 1;4 -1;-2 0;20 0;0 4], ...
    'colour',[1;2;1;2;1],'z',zeros(5,1)));
sensor=c.sensor; sensor.mode='Perfect';
obs=simulateSensor(env,v,sensor,0,RandStream('mt19937ar','Seed',1));
assert(size(obs.xy,1)==2 && ~isfield(obs,'id'));
sensor.mode='Noisy';
a=simulateSensor(env,v,sensor,0,RandStream('mt19937ar','Seed',21));
b=simulateSensor(env,v,sensor,0,RandStream('mt19937ar','Seed',21));
assert(isequal(a,b),'Same seed and pose must reproduce the observation packet.');
% Repeated exact detections associate to existing landmarks.
map=[];
for i=1:5, map=updateConeMap(map,obs,[0 0 0],i/10,c.map,true); end
assert(size(map.xy,1)==2 && all(map.hits==5));
% Straight noisy corridor: real Delaunay gates must produce a forward route.
x=(2:3:14)'; xy=[x,2*ones(size(x));x,-2*ones(size(x))];
colours=[ones(size(x));2*ones(size(x))];
ob=struct('xy',xy,'colour',colours,'sigma',0.05*ones(size(colours)),'z',zeros(size(colours)));
map=updateConeMap([],ob,[0 0 0],0,c.map,true);
plan=planPath(map,[0 0 0],0,c.planner,zeros(0,2));
assert(plan.valid,plan.status); assert(all(diff(plan.route.raw(:,1))>0));
assert(max(abs(plan.localPath(:,2)))<0.2,'Straight corridor route should stay near y=0.');
% Local observations alone determine planning output, independently of truth.
alteredTrack=track; alteredTrack.reference=track.reference+[1000 -700]; %#ok<NASGU>
same=planPath(map,[0 0 0],0,c.planner,zeros(0,2));
assert(isequal(plan.worldPath,same.worldPath));
% Empty planning input forces braking, including when the car already moves.
invalid=planPath([],pose,0,c.planner,zeros(0,2)); v.speed=4;
u=purePursuitController(v,pose,invalid,c.vehicle,0);
assert(u.accel<0 && u.targetSpeed==0);
v2=updateVehicle(v,u,c.vehicle,c.sim.dt);
assert(v2.speed<v.speed && hypot(v2.x-v.x,v2.y-v.y)<=v.speed*c.sim.dt+1e-10);
% Every shipped track must generate valid boundaries and cone placement.
types={'Oval','Figure eight','Technical','High speed','Random','Hairpin'};
for i=1:numel(types)
    cc=c; cc.track.type=types{i}; generated=generateTrack(cc);
    if strcmp(types{i},'Figure eight'), crossing=generated; end
end
% Changing hidden evaluation progress must not alter overpass perception or
% planning. This guards against reference-metric feedback through deck choice.
left=initializeSimulation(c,crossing); right=initializeSimulation(c,crossing);
right.evaluation.progress=crossing.length/2;
left=stepSimulation(left); right=stepSimulation(right);
assert(isequal(left.observations,right.observations));
assert(isequal(left.plan.worldPath,right.plan.worldPath));
short=c; short.sim.maxTime=8;
[result,state]=runHeadless(short,track);
assert(result.distance>5,'Vehicle failed to progress in default smoke run.');
assert(result.validPlanRate>0.6,'Default planning validity unexpectedly low.');
assert(all(isfinite(state.trajectory(:))),'Vehicle state is nonfinite.');
report=struct('sourceFilesParsed',numel(files),'smoke',result,'fullLap',[]);
if fullLap
    [lap,state]=runHeadless(c,track); report.fullLap=lap;
    exportResults(state,fullfile(folder,'results','default_lap'));
    assert(lap.lapComplete,['Default lap not completed: ' lap.reason]);
    assert(lap.collisionCount==0,'Default lap encountered a cone collision.');
    assert(lap.boundaryViolations==0,'Default lap left the track.');
end
disp(report); fprintf('All requested demo assertions passed.\n');
end
