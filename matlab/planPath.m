function plan=planPath(map,pose,height,c,previousWorld,ego)
if nargin<6
    ego=struct('x',0,'y',0,'yaw',0,'z',height,'speed',0,'steer',0,'accel',0);
end
clock=tic; points=zeros(0,2); colour=zeros(0,1); uncertainty=0;
if ~isempty(map)
    local=toLocal(map.xy,pose);
    radius=vecnorm(local,2,2);
    keep=local(:,1)>-12 & radius<32 & map.hits>=1 & abs(map.z-height)<0.9+c.hybrid.maxGrade*radius;
    points=local(keep,:); colour=map.colour(keep);
    if any(keep), uncertainty=sqrt(mean(map.variance(keep))); end
end
% Injected planner noise does not alter the perceived physical obstacle map.
geometry=points;
seed=mod(round(sum(abs(pose).*[137 199 331])*100),2^32-1);
stream=RandStream('mt19937ar','Seed',seed);
if c.hybrid.planningNoise>0, geometry=geometry+c.hybrid.planningNoise*randn(stream,size(geometry)); end
recovery=recoverBoundaryCones(geometry,colour,c); recovery.physical=points;
b=estimateBoundaryModel(recovery,c);
if ~c.hybrid.enabled, b.usable=false; end
ahead=points(:,1)>-3;
rawTri=computeDelaunay(points(ahead,:),colour(ahead));
augmented=recovery.corrected(ahead,:); labels=colour(ahead);
if c.hybrid.enabled && c.hybrid.virtual
    augmented=[augmented;b.virtual]; labels=[labels;b.virtualColour];
end
tri=computeDelaunay(augmented,labels);
f=filterDelaunayEdges(tri,c,c.strategy);
if c.hybrid.planningDropout>0, f.valid=f.valid & rand(stream,size(f.valid))>=c.hybrid.planningDropout; end
previous=toLocal(previousWorld,pose);
route=reconstructCentreline(tri,f,c,c.strategy,previous);
if ~strcmp(tri.status,'OK'), route.valid=false; route.status=tri.status; end
candidates={}; names={}; qualities=[];
if route.valid
    path=route.raw;
    if c.smooth, [path,~]=smoothPath(route.raw,tri,f); end
    candidates{end+1}=path; names{end+1}='Delaunay'; qualities(end+1)=1;
end
graph=struct('vertices',zeros(0,2),'edges',zeros(0,2));
if c.hybrid.enabled && b.usable
    candidates{end+1}=b.centre; names{end+1}='Boundary matching'; qualities(end+1)=b.confidence;
    [vpath,graph]=planVoronoiRoute(points,b,c);
    if ~isempty(vpath), candidates{end+1}=vpath; names{end+1}='Voronoi'; qualities(end+1)=b.confidence; end
end
best=-inf; selected=zeros(0,2); selectedName='None'; selectedQuality=0;
hazard=any(points(:,1)>-0.5 & points(:,1)<5 & abs(points(:,2))<c.avoidance.clearance+0.5);
smoothStatus='No feasible candidate'; candidateLog=struct('name',{},'valid',{},'score',{},'status',{});
for i=1:numel(candidates)
    if ~isempty(candidates{i}) && candidates{i}(1,1)>0.1 && norm(candidates{i}(1,:))<6
        candidates{i}=[0 0;candidates{i}];
    end
    [candidate,safe,status]=avoidPhysicalCones(candidates{i},points,c);
    if safe && size(candidate,1)>=2
        ss=[0;cumsum(vecnorm(diff(candidate),2,2))];
        [ss,ids]=unique(ss,'stable'); candidate=candidate(ids,:);
        samples=interp1(ss,candidate,unique([(0:0.4:ss(end))';ss(end)]));
        if b.usable
            safe=all(insideBoundaryCorridor(samples,b,c.avoidance.clearance-0.3));
            if ~safe, status='Rejected outside supported boundary corridor'; end
        end
        if safe && hazard && c.hybrid.enabled
            safe=controlCandidateFeasible(candidate,points,c,ego);
            if ~safe, status='Rejected by controller footprint rollout'; end
        end
        if safe
            deviation=0;
            if size(previous,1)>1, deviation=mean(min(2,projectPolyline(samples,previous))); end
            d=diff(candidate); angle=unwrap(atan2(d(:,2),d(:,1)));
            turn=sum(abs(diff(angle)));
            score=ss(end)-2*turn-0.8*deviation+2*qualities(i);
            if strcmp(names{i},'Delaunay'), score=score+1.5; end
            if score>best
                best=score; selected=candidate; selectedName=names{i}; selectedQuality=qualities(i);
                smoothStatus=[names{i} ' | ' status];
            end
        else, score=-inf; end
    else, score=-inf; safe=false; end
    candidateLog(end+1)=struct('name',names{i},'valid',safe,'score',score,'status',status); %#ok<AGROW>
end
if isempty(selected) && c.hybrid.enabled && b.usable
    escape=planCurvatureEscape(points,b,c,ego);
    if ~isempty(escape)
        selected=escape; selectedName='Curvature escape'; selectedQuality=0.65;
        smoothStatus='Curvature escape | footprint checked';
        candidateLog(end+1)=struct('name',selectedName,'valid',true,'score',0,'status',smoothStatus);
    end
end
valid=~isempty(selected);
if valid
    route.valid=true; route.status=['Valid: ' selectedName];
else
    route.valid=false;
    if ~isempty(candidates), route.status='Blocked or insufficient supported corridor'; end
end
if ~strcmp(selectedName,'Delaunay') && valid, route.raw=selected; end
confidence=max(0,min(1,selectedQuality-1.2*uncertainty)); speedLimit=inf;
cautious=b.oneSided || any(recovery.anomaly) || ismember(selectedName,{'Voronoi','Curvature escape'});
if valid
    cautious=cautious || pathPhysicalClearance([0 0;selected],points)<c.avoidance.clearance+0.35;
end
if cautious || confidence<0.7 || contains(smoothStatus,'offset route')
    speedLimit=c.hybrid.uncertainSpeed;
end
plan=struct('tri',tri,'filter',f,'route',route,'localPath',selected, ...
    'worldPath',toWorld(selected,pose),'rawWorld',toWorld(route.raw,pose), ...
    'pose',pose,'valid',valid,'status',route.status,'smoothStatus',smoothStatus, ...
    'time',toc(clock),'stamp',0,'recovery',recovery,'rawTri',rawTri, ...
    'boundary',b,'voronoi',graph,'candidates',candidateLog,'selected',selectedName, ...
    'confidence',confidence,'speedLimit',speedLimit,'cautious',cautious);
end
