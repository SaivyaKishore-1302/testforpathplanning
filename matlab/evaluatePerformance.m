function e=evaluatePerformance(env,v,old,plan,e,c,planned)
% Ground-truth evaluation is one-way: none of these results feed control.
xy=[v.x v.y]; ds=env.s-e.progress;
if env.closed, ds=mod(ds+env.length/2,env.length)-env.length/2; end
ids=find(ds>=-6 & ds<=18);
if isempty(ids), ids=(1:numel(env.s))'; end
nearest=inf; segment=1; station=e.progress;
for k=1:numel(ids)
    j=ids(k); if j>=size(env.reference,1), continue; end
    [d,along]=projectPolyline(xy,env.reference(j:j+1,:));
    if d<nearest
        nearest=d; segment=j; station=env.s(j)+along;
    end
end
if env.closed
    delta=mod(station-mod(e.progress,env.length)+env.length/2,env.length)-env.length/2;
    % Progress can decrease when reversing; no artificial monotonic lap count.
    e.progress=e.progress+delta;
else, e.progress=station;
end
e.cte=nearest; e.lap=max(0,floor(e.progress/env.length));
if env.closed, e.complete=e.progress>=env.length;
else, e.complete=e.progress>=env.length-1 && norm(xy-env.reference(end,:))<2; end
% Collision swept footprint: sample pose interpolation to avoid tunnelling.
span=hypot(v.x-old.x,v.y-old.y); samples=max(2,ceil(span/0.15)+1);
hit=false(size(e.hitCones));
for alpha=linspace(0,1,samples)
    pose=[old.x old.y old.yaw]+alpha*[v.x-old.x,v.y-old.y,wrapAngle(v.yaw-old.yaw)];
    p=toLocal(env.cones.xy,pose);
    dx=max(abs(p(:,1))-c.vehicle.length/2,0);
    dy=max(abs(p(:,2))-c.vehicle.width/2,0);
    hit=hit | (hypot(dx,dy)<0.16 & abs(env.cones.z-v.z)<0.9);
end
e.collisionCount=e.collisionCount+sum(hit & ~e.hitCones); e.hitCones=e.hitCones|hit;
corners=[-1 -1;1 -1;1 1;-1 1].*[c.vehicle.length c.vehicle.width]/2;
corners=toWorld(corners,[v.x v.y v.yaw]);
localRef=referenceWindow(env,e.progress,12);
% At an open-course start the rear overhang extends before the first station;
% this is longitudinal overhang, not a lateral boundary violation.
[~,~,feet,indices]=projectPolyline(corners,localRef);
tangents=localRef(indices+1,:)-localRef(indices,:);
normals=[-tangents(:,2),tangents(:,1)]./max(vecnorm(tangents,2,2),eps);
cornerD=abs(sum((corners-feet).*normals,2));
outside=any(cornerD>env.width(segment)/2);
if outside && ~e.outside, e.violations=e.violations+1; end
e.outside=outside;
if planned
    e.pathMean=NaN; e.pathMax=NaN; e.pathRMS=NaN; e.gateValid=NaN;
    if plan.valid && size(plan.worldPath,1)>1
        ss=[0;cumsum(vecnorm(diff(plan.worldPath),2,2))];
        q=interp1(ss,plan.worldPath,unique([(0:0.25:ss(end))';ss(end)]));
        ref=referenceWindow(env,e.progress,35);
        error=projectPolyline(q,ref);
        e.pathMean=mean(error); e.pathMax=max(error); e.pathRMS=sqrt(mean(error.^2));
        e.reconstructionSamples=[e.reconstructionSamples;error];
        % Gate validity is a labelled spatial proxy, not hidden landmark IDs.
        % Both endpoints must bracket the reference tangent near the midpoint.
        valid=false(numel(plan.route.edgeIds),1);
        for k=1:numel(valid)
            ij=plan.tri.edges(plan.route.edgeIds(k),:);
            pts=toWorld(plan.tri.points(ij,:),plan.pose); mid=mean(pts,1);
            [md,~,foot,j]=projectPolyline(mid,ref);
            tangent=ref(j+1,:)-ref(j,:); normal=[-tangent(2),tangent(1)]/max(norm(tangent),eps);
            offsets=(pts-foot)*normal';
            valid(k)=md<0.25*c.track.width && prod(offsets)<0 && max(abs(offsets))<c.track.width;
        end
        e.gateValid=mean(valid);
    end
end
end

function ref=referenceWindow(env,station,radius)
if ~env.closed
    ids=find(env.s>=max(0,station-radius) & env.s<=min(env.length,station+radius));
    ref=env.reference(ids,:); return
end
% Repeat around seam, preserving spatial order and excluding other overpass.
s=[env.s(1:end-1)-env.length;env.s;env.s(2:end)+env.length];
p=[env.reference(1:end-1,:);env.reference;env.reference(2:end,:)];
station=mod(station,env.length); ref=p(s>=station-radius & s<=station+radius,:);
end
