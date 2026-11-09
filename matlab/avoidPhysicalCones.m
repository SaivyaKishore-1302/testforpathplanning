function [path,ok,status]=avoidPhysicalCones(raw,physical,c)
%AVOIDPHYSICALCONES Offset lattice about the corrected Delaunay route.
% Every transition is checked as a continuous segment against ALL physical
% cones, including rejected landmarks; virtual cones are never obstacles.
path=raw; ok=true; status='Physical clearance checked';
if size(raw,1)<2 || isempty(physical) || ~c.avoidance.enabled, return; end
% Discard already-traversed route stations before anchoring an avoidance
% search. Prepending the car to a behind-car waypoint creates a reversing
% loop, which a forward-only pursuit controller correctly refuses to follow.
[~,~,foot,index]=projectPolyline([0 0],raw);
if index>1 || norm(foot-raw(1,:))>0.05
    raw=raw(index+1:end,:);
    if isempty(raw), raw=foot; end
end
raw=raw([true;vecnorm(diff(raw,1,1),2,2)>1e-5],:); path=raw;
if size(raw,1)<2, ok=false; status='No forward route remaining'; return; end
radius=c.avoidance.clearance;
% If tracking has entered the extra planning allowance, allow recovery while
% retaining the hard vehicle/cone safety clearance. Otherwise the lattice
% could deadlock at its anchored start despite a physically safe escape.
startDistance=min(vecnorm(physical,2,2));
radius=min(radius,max(radius-c.avoidance.trackingMargin,startDistance-0.01));
if startDistance<radius+1
    radius=c.avoidance.clearance-c.avoidance.trackingMargin;
end
% Check the connector from the current vehicle pose too: the first gate may
% lie beyond a physical cone, even when all subsequent route segments clear it.
if clearance([0 0;raw],physical)>=radius, return; end
if norm(raw(1,:))>0.05, raw=[0 0;raw]; end
s=[0;cumsum(vecnorm(diff(raw),2,2))];
[s,ids]=unique(s,'stable'); raw=raw(ids,:);
if numel(s)<2, ok=false; status='Degenerate route'; path=zeros(0,2); return; end
ss=unique([(0:0.5:s(end))';s(end)]); base=interp1(s,raw,ss);
d=[gradient(base(:,1)),gradient(base(:,2))];
n=[-d(:,2),d(:,1)]./max(vecnorm(d,2,2),eps);
offset=linspace(-c.avoidance.maxOffset,c.avoidance.maxOffset,13);
N=numel(ss); K=numel(offset); nodes=zeros(N,K,2);
cost=inf(N,K); parent=zeros(N,K);
for k=1:K, nodes(:,k,:)=reshape(base+n*offset(k),N,1,2); end
% Anchor to the existing route to avoid an instantaneous lateral jump.
[~,middle]=min(abs(offset)); cost(1,middle)=0;
for i=2:N
    for k=1:K
        b=reshape(nodes(i,k,:),1,2);
        if any(vecnorm(physical-b,2,2)<radius), continue; end
        for j=1:K
            if ~isfinite(cost(i-1,j)) || abs(offset(k)-offset(j))>0.36, continue; end
            a=reshape(nodes(i-1,j,:),1,2);
            if clearance([a;b],physical)<radius, continue; end
            candidate=cost(i-1,j)+0.1*offset(k)^2+4*(offset(k)-offset(j))^2;
            if candidate<cost(i,k), cost(i,k)=candidate; parent(i,k)=j; end
        end
    end
end
[best,k]=min(cost(end,:)); ok=isfinite(best);
if ~ok, path=zeros(0,2); status='Blocked: no physical clearance'; return; end
path=zeros(N,2);
for i=N:-1:1
    path(i,:)=reshape(nodes(i,k,:),1,2);
    if i>1, k=parent(i,k); end
end
status='Obstacle avoidance offset route';
end

function d=clearance(path,points)
d=inf;
for i=1:size(path,1)-1
    a=path(i,:); v=path(i+1,:)-a;
    t=max(0,min(1,((points-a)*v')/max(sum(v.^2),eps)));
    d=min(d,min(vecnorm(points-(a+t.*v),2,2)));
end
end
