function [path,graph]=planVoronoiRoute(physical,b,c)
%PLANVORONOIROUTE Bounded physical-obstacle Voronoi graph clipped to corridor.
path=zeros(0,2); graph=struct('vertices',zeros(0,2),'edges',zeros(0,2));
if ~c.hybrid.voronoi || ~b.usable || size(physical,1)<4, return; end
try
    p=unique(round(physical/0.03)*0.03,'rows');
    if rank(p-mean(p,1))<2, return; end
    dt=delaunayTriangulation(p); [vertices,regions]=voronoiDiagram(dt);
catch
    return;
end
edges=zeros(0,2);
for i=1:numel(regions)
    ids=regions{i}; if numel(ids)<2, continue; end
    edges=[edges;ids(:),circshift(ids(:),-1)]; %#ok<AGROW>
end
edges=unique(sort(edges,2),'rows');
keep=all(isfinite(vertices),2) & vertices(:,1)>-2 & vecnorm(vertices,2,2)<28;
finite=find(keep); keep(finite)=insideBoundaryCorridor(vertices(finite,:),b,c.avoidance.clearance-0.15);
edges=edges(keep(edges(:,1)) & keep(edges(:,2)),:);
accepted=false(size(edges,1),1);
for i=1:size(edges,1)
    segment=vertices(edges(i,:),:);
    probes=segment(1,:)+(0:0.2:1)'*(segment(2,:)-segment(1,:));
    accepted(i)=all(insideBoundaryCorridor(probes,b,c.avoidance.clearance-0.15)) && ...
        pathPhysicalClearance(segment,physical)>=c.avoidance.clearance;
end
edges=edges(accepted,:); graph.vertices=vertices; graph.edges=edges;
if isempty(edges), return; end
used=unique(edges); start=size(vertices,1)+1; vertices(start,:)=[0 0];
[~,order]=sort(vecnorm(vertices(used,:),2,2));
for i=1:min(10,numel(order))
    j=used(order(i));
    if vertices(j,1)<0 || norm(vertices(j,:))>6, continue; end
    if pathPhysicalClearance([0 0;vertices(j,:)],physical)>=c.avoidance.clearance
        edges(end+1,:)=[start j]; %#ok<AGROW>
    end
end
N=size(vertices,1); cost=inf(N,1); cost(start)=0; parent=zeros(N,1); visited=false(N,1);
for k=1:N
    scores=cost; scores(visited)=inf; [value,i]=min(scores); if ~isfinite(value), break; end
    visited(i)=true;
    rows=find(any(edges==i,2));
    for row=rows'
        j=edges(row,1); if j==i, j=edges(row,2); end
        delta=vertices(j,:)-vertices(i,:); gap=norm(delta);
        if gap<0.05 || gap>9 || delta(1)<-1, continue; end
        if parent(i)>0
            before=vertices(i,:)-vertices(parent(i),:);
            turn=acos(max(-1,min(1,dot(before,delta)/max(norm(before)*gap,eps))));
            if turn>c.maxTurn, continue; end
        else
            turn=abs(atan2(delta(2),delta(1))); if turn>c.maxHeadingError, continue; end
        end
        off=projectPolyline(vertices(j,:),b.centre);
        value=cost(i)+gap*(1+0.4*off)+2*turn^2;
        if value<cost(j), cost(j)=value; parent(j)=i; end
    end
end
reachable=find(isfinite(cost) & (1:N)'~=start);
if isempty(reachable), return; end
reward=vecnorm(vertices(reachable,:),2,2)-0.15*cost(reachable);
[~,best]=max(reward); j=reachable(best); ids=j;
for k=1:N
    j=parent(j); if j==0, return; end
    ids=[j ids]; %#ok<AGROW>
    if j==start, break; end
end
path=vertices(ids,:);
if sum(vecnorm(diff(path),2,2))<c.minLength, path=zeros(0,2); end
end
