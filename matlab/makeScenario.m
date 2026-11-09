function [c,track,meta]=makeScenario(name,seed)
%MAKESCENARIO Truth perturbations belong only to scene construction/evaluation.
if nargin<2, seed=2685; end
names=scenarioCatalog(); if isnumeric(name), name=names{name}; end
c=defaultConfig(); c.seed=seed; c.track.type='Custom'; c.track.closed=false;
c.track.controls=[0 0;20 0;40 0;60 0;80 0];
c.sim.maxTime=40; c.sim.stopOnCollision=true;
meta=struct('name',name,'targetProgress',28,'expected','progress', ...
    'description','','eventTime',inf,'eventIndex',0,'eventOffset',[0 0]);
switch name
    case 'Nominal noisy oval'
        c.track.type='Oval'; c.track.closed=true; c.sim.maxTime=150;
        meta.targetProgress=inf; meta.expected='lap';
    case 'Irregular spacing', c.track.irregularity=0.55;
    case 'Random missing blue', c.track.missingBlue=0.35;
    case 'Variable width', c.track.widthVariation=0.22;
    case 'Colour measurement errors', c.sensor.colourAccuracy=0.7;
    case 'Sparse cones', c.track.spacing=5.5;
    case 'Dense cones', c.track.spacing=1.5;
    case 'False detections', c.sensor.falseRate=0.8;
    case 'Heavy range noise', c.sensor.sigma=0.15; c.sensor.distanceNoise=0.018;
    case 'Localization drift', c.localization.positionWalk=0.035; c.localization.headingWalk=0.006;
    case 'Planner geometry noise', c.planner.hybrid.planningNoise=0.16;
    case 'Delaunay gate dropout', c.planner.hybrid.planningDropout=0.35;
    case 'Short sensor outage', c.sensor.outageStart=4; c.sensor.outageDuration=0.8;
    case 'Long sensor outage'
        c.sensor.outageStart=4; c.sensor.outageDuration=30; meta.expected='stop';
    case 'Narrow field of view', c.sensor.fov=75*pi/180;
    case 'Short sensing range', c.sensor.range=8;
    case 'High requested speed', c.vehicle.speed=8;
    case 'Hairpin'
        c.track.type='Hairpin'; c.track.closed=true; c.sim.maxTime=65; meta.targetProgress=80;
    case 'S bend'
        c.track.controls=[0 0;20 0;40 7;60 -7;80 0;100 0]; meta.targetProgress=55; c.sim.maxTime=60;
    case 'Figure eight overpass'
        c.track.type='Figure eight'; c.track.closed=true; meta.targetProgress=75; c.sim.maxTime=65;
    case 'Too narrow corridor'
        c.track.width=2.2; c.planner.width=2.2; c.planner.minEdge=1.2; meta.expected='stop';
    case 'Combined disturbances'
        c.track.irregularity=0.3; c.track.missingBlue=0.15; c.track.missingYellow=0.1;
        c.sensor.sigma=0.1; c.sensor.colourAccuracy=0.88; c.sensor.missRate=0.12;
        c.planner.hybrid.planningNoise=0.08;
end
c.planner.width=c.track.width;
track=generateTrack(c); stream=RandStream('mt19937ar','Seed',seed+50);
blue=track.cones.colour==1; yellow=~blue; station=track.cones.station;
switch name
    case 'Staggered sides'
        ids=find(yellow); st=station(ids)+2; keep=st<track.length;
        track=removeCones(track,ids(~keep));
        ids=find(track.cones.colour==2); st=track.cones.station(ids)+2;
        track.cones.xy(ids,:)=interp1(track.s,track.right,st);
        track.cones.station(ids)=st; track.cones.z(ids)=interp1(track.s,track.z,st);
    case 'Lateral jitter'
        delta=0.25*(2*rand(stream,size(station))-1);
        normal=interp1(track.s,track.normal,station);
        track.cones.xy=track.cones.xy+normal.*delta;
    case 'Missing yellow section'
        track=removeCones(track,find(yellow & station>=12 & station<=27));
    case 'One side unseen'
        track=removeCones(track,find(yellow));
    case 'Both sides gap'
        track=removeCones(track,find(station>=12 & station<=21));
        meta.expected='stop';
        meta.description='A long unseen gap on both sides requires a conservative stop.';
    case {'Blue displaced inward','Blue displaced outward','Yellow displaced inward','Moving cone event'}
        if strcmp(name,'Yellow displaced inward'), side=2; else, side=1; end
        ids=find(track.cones.colour==side); [~,j]=min(abs(track.cones.station(ids)-18)); id=ids(j);
        delta=[0 -1.4];
        if side==2 || strcmp(name,'Blue displaced outward'), delta=-delta; end
        if strcmp(name,'Moving cone event')
            meta.eventTime=2; meta.eventIndex=id; meta.eventOffset=delta;
        else, track.cones.xy(id,:)=track.cones.xy(id,:)+delta; end
    case 'Adjacent displaced cones'
        ids=find(blue & station>=15 & station<=21);
        track.cones.xy(ids,2)=track.cones.xy(ids,2)-1.3;
    case 'Blocked corridor'
        track=addCones(track,[18 -1;18 0;18 1],[1;1;2]); meta.expected='stop';
    case 'Mislabelled physical cone'
        ids=find(blue); [~,j]=min(abs(station(ids)-18)); track.cones.colour(ids(j))=2;
    case 'Entire blue side shifted'
        track.cones.xy(blue,2)=track.cones.xy(blue,2)+1.3;
        meta.description='Original boundary is unidentifiable from current observations alone.';
    case 'Both sides unseen'
        track=removeCones(track,(1:numel(station))'); meta.expected='stop';
    case 'Duplicate detections'
        ids=find(station>=9 & station<=24);
        xy=track.cones.xy(ids,:)+0.12*randn(stream,numel(ids),2);
        track=addCones(track,xy,track.cones.colour(ids));
end
meta.description=[meta.description ' Physical scene and sensor perturbations are isolated from planning inputs.'];
end

function t=removeCones(t,ids)
keep=true(size(t.cones.colour)); keep(ids)=false; fields=fieldnames(t.cones);
for i=1:numel(fields), t.cones.(fields{i})=t.cones.(fields{i})(keep,:); end
end
function t=addCones(t,xy,colour)
t.cones.xy=[t.cones.xy;xy]; t.cones.colour=[t.cones.colour;colour];
t.cones.z=[t.cones.z;zeros(size(colour))]; t.cones.station=[t.cones.station;xy(:,1)];
end
