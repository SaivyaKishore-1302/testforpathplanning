function report=hybridTest()
%HYBRIDTEST Geometry invariants and adversarial planning regression checks.
files=dir('*.m');
for k=1:numel(files), [~,name]=fileparts(files(k).name); nargin(str2func(name)); end
c=defaultConfig(); x=(0:3:24)';
p=[x,2+zeros(size(x));x+2,-2+zeros(size(x))];
colour=[ones(size(x));2*ones(size(x))];
r=recoverBoundaryCones(p,colour,c.planner); b=estimateBoundaryModel(r,c.planner);
assert(b.usable && max(abs(b.centre(:,2)))<1e-8,'Stagger must not create a lateral shift.');
assert(~any(r.anomaly),'Ordinary staggering is not displacement.');
one=recoverBoundaryCones(p(colour==1,:),ones(sum(colour==1),1),c.planner);
b=estimateBoundaryModel(one,c.planner);
assert(b.usable && b.oneSided && ~isempty(b.virtual));
assert(all(b.virtualConfidence<1),'Assumed cones must retain lower confidence.');
assert(isequal(one.physical,p(colour==1,:)),'Virtual creation changed physical evidence.');
empty=recoverBoundaryCones(zeros(0,2),zeros(0,1),c.planner);
gone=estimateBoundaryModel(empty,c.planner);
assert(isempty(gone.virtual),'Virtuals survived after their physical support disappeared.');
ob=struct('xy',p,'colour',colour,'sigma',zeros(size(colour)),'z',zeros(size(colour)));
map=updateConeMap([],ob,[0 0 0],0,c.map,true);
original=map; plan=planPath(map,[0 0 0],0,c.planner,zeros(0,2));
assert(plan.valid && max(abs(plan.localPath(:,2)))<0.2);
assert(isequal(map,original),'Planning must not modify or populate the physical map.');
% Pose transforms must not change the local plan.
pose=[23 -14 2.2]; rotated=map; rotated.xy=toWorld(map.xy,pose);
same=planPath(rotated,pose,0,c.planner,zeros(0,2));
assert(size(same.localPath,1)==size(plan.localPath,1));
assert(max(abs(same.localPath-plan.localPath),[],'all')<1e-6);
% Actual Voronoi edges exist and stay in the supported corridor.
b=estimateBoundaryModel(r,c.planner); [vp,vg]=planVoronoiRoute(p,b,c.planner);
assert(~isempty(vg.edges) && size(vp,1)>1,'No actual Voronoi graph was constructed.');
assert(pathPhysicalClearance(vp,p)>=c.planner.avoidance.clearance-1e-8);
[escape,ok]=avoidPhysicalCones([-3 0;0 0;10 0],[5 0.6],c.planner);
assert(ok && min(escape(:,1))>=-1e-8,'Avoidance introduced a reversing prefix.');
% Empty input brakes; stale virtual points cannot make a valid blind plan.
blind=planPath([],pose,0,c.planner,plan.worldPath); assert(~blind.valid);
% Sensor streams are independent of synthetic planning perturbations.
[cc,t]=makeScenario('Staggered sides',2685); a=initializeSimulation(cc,t);
cc.planner.hybrid.planningNoise=0.2; z=initializeSimulation(cc,t);
a=stepSimulation(a); z=stepSimulation(z); assert(isequal(a.observations,z.observations));
% Every catalog entry must build, with finite truth geometry.
names=scenarioCatalog();
for k=1:numel(names)
    [~,t,~]=makeScenario(k,2685); assert(all(isfinite(t.cones.xy),'all'));
end
report=struct('parsedFunctions',numel(files),'scenariosConstructed',numel(names), ...
    'staggerInvariant',true,'virtualLifecycle',true,'voronoiChecked',true, ...
    'physicalMapIsolation',true,'rigidTransformInvariant',true);
disp(report); fprintf('All hybrid invariants passed.\n');
end
