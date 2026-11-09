function route=reconstructCentreline(tri,f,c,strategy,previous)
% Gate graph: two cross-colour edges are adjacent only inside one triangle.
% Planner inputs contain no track, global cones, reference path or evaluator.
[p,edgeIds]=generateMidpoints(f);
route=struct('raw',zeros(0,2),'edgeIds',zeros(0,1),'valid',false, ...
    'status','No valid cross-track edges','ambiguous',false,'alternatives',{{}});
N=size(p,1); if N<2, return; end
if strategy<3
    % Deliberately weak, explicit baseline ordering (not used by strategy 3).
    keep=p(:,1)>=-1;
    idx=find(keep); [~,order]=sort(p(idx,1)); idx=idx(order);
    route.raw=p(idx,:); route.edgeIds=edgeIds(idx);
    if numel(idx)<2, route.status='Insufficient forward midpoints'; return; end
    if any(vecnorm(diff(route.raw),2,2)>c.maxGap)
        route.status='Baseline midpoint ordering has a gap'; return
    end
else
    adj=false(N);
    for k=1:size(tri.triangles,1)
        vertices=tri.triangles(k,:);
        inside=find(all(ismember(tri.edges(edgeIds,:),vertices),2));
        if numel(inside)==2, adj(inside(1),inside(2))=true; adj(inside(2),inside(1))=true; end
    end
    dist=vecnorm(p,2,2); startCost=dist+2*abs(p(:,2))+2*max(0,-p(:,1));
    startCost(p(:,1)<-1.5 | dist>5.5)=inf;
    [~,seeds]=sort(startCost); seeds=seeds(isfinite(startCost(seeds)));
    seeds=seeds(1:min(3,numel(seeds)));
    if isempty(seeds), route.status='No nearby forward gate'; return; end
    beam=cell(numel(seeds),1); scores=zeros(numel(seeds),1);
    for k=1:numel(seeds), beam{k}=seeds(k); scores(k)=-0.5*startCost(seeds(k)); end
    allRoutes={}; allScores=zeros(0,1);
    for depth=1:c.maxDepth
        expanded={}; values=zeros(0,1);
        for b=1:numel(beam)
            seq=beam{b}; last=seq(end); neighbours=find(adj(last,:));
            for j=neighbours
                if any(seq==j), continue; end
                step=p(j,:)-p(last,:); gap=norm(step);
                if gap<0.15 || gap>c.maxGap, continue; end
                direction=step/gap;
                if dot(direction,f.direction(edgeIds(last),:))<0.15 || ...
                        dot(direction,f.direction(edgeIds(j),:))<0.15, continue; end
                if numel(seq)>1
                    before=p(last,:)-p(seq(end-1),:); before=before/norm(before);
                else, before=[1 0]; end
                turn=acos(max(-1,min(1,dot(before,direction))));
                if turn>c.maxTurn, continue; end
                if c.useBoundary && crossesBoundary(p(last,:),p(j,:),tri,f), continue; end
                continuity=0;
                if c.useContinuity && size(previous,1)>1
                    continuity=min(1,projectPolyline(p(j,:),previous));
                end
                value=scores(b)+gap-1.5*turn^2-0.25*abs(f.length(edgeIds(j))-c.width)-0.5*continuity;
                expanded{end+1,1}=[seq j]; values(end+1,1)=value; %#ok<AGROW>
            end
        end
        allRoutes=[allRoutes;beam]; allScores=[allScores;scores]; %#ok<AGROW>
        if isempty(expanded), break; end
        [~,order]=sort(values,'descend'); order=order(1:min(c.beamWidth,numel(order)));
        beam=expanded(order); scores=values(order);
    end
    if isempty(allRoutes), route.status='Disconnected gate graph'; return; end
    [~,order]=sort(allScores,'descend'); best=allRoutes{order(1)};
    route.raw=p(best,:); route.edgeIds=edgeIds(best);
    % Report genuinely divergent, similarly scored routes, not shared prefixes.
    for k=2:min(20,numel(order))
        other=allRoutes{order(k)};
        if numel(other)<3 || numel(best)<3, continue; end
        overlap=numel(intersect(other,best))/min(numel(other),numel(best));
        if overlap<0.35
            route.alternatives{end+1}=p(other,:);
            if allScores(order(1))-allScores(order(k))<c.ambiguityMargin*max(1,abs(allScores(order(1))))
                route.ambiguous=true;
            end
        end
    end
    if route.ambiguous, route.status='Ambiguous competing gate routes'; return; end
end
if size(route.raw,1)<2 || sum(vecnorm(diff(route.raw),2,2))<c.minLength
    route.status='Observed route is too short'; return
end
route.valid=true; route.status='Valid';
end

function hit=crossesBoundary(a,b,tri,f)
hit=false;
for k=1:size(f.boundaries,1)
    ij=f.boundaries(k,:);
    if segmentsCross(a,b,tri.points(ij(1),:),tri.points(ij(2),:)), hit=true; return; end
end
end
