function map=updateConeMap(map,obs,pose,time,c,remember)
% Greedy nearest-neighbour association, one observation per landmark per scan.
if isempty(map) || ~remember
    map=struct('xy',zeros(0,2),'vote',zeros(0,1),'colour',zeros(0,1), ...
        'variance',zeros(0,1),'hits',zeros(0,1),'last',zeros(0,1),'z',zeros(0,1), ...
        'misses',zeros(0,1));
end
if ~isfield(map,'misses'), map.misses=zeros(size(map.hits)); end
xy=toWorld(obs.xy,pose); used=false(size(map.xy,1),1);
for i=1:size(xy,1)
    distances=vecnorm(map.xy-xy(i,:),2,2);
    distances(used | abs(map.z-obs.z(i))>0.8)=inf;
    if isempty(distances), d=inf; else, [d,j]=min(distances); end
    var=max(0.0025,obs.sigma(i)^2);
    if d<c.gate
        gain=map.variance(j)/(map.variance(j)+var);
        map.xy(j,:)=map.xy(j,:)+gain*(xy(i,:)-map.xy(j,:));
        map.variance(j)=max(0.0004,(1-gain)*map.variance(j)+0.0001);
        map.vote(j)=max(-10,min(10,map.vote(j)+3-2*obs.colour(i)));
        map.hits(j)=map.hits(j)+1; map.last(j)=time; map.z(j)=obs.z(i); used(j)=true; map.misses(j)=0;
    else
        j=size(map.xy,1)+1; map.xy(j,:)=xy(i,:); map.variance(j,1)=var;
        map.vote(j,1)=3-2*obs.colour(i); map.hits(j,1)=1;
        map.last(j,1)=time; map.z(j,1)=obs.z(i); used(j,1)=true; map.misses(j,1)=0;
    end
end
map.colour=ones(size(map.vote)); map.colour(map.vote<0)=2;
keep=time-map.last<=c.ttl & vecnorm(map.xy-pose(1:2),2,2)<c.radius;
if isfield(c,'unconfirmedTTL'), keep=keep & (map.hits>=2 | time-map.last<c.unconfirmedTTL); end
% A complete outage is not evidence that every physical landmark vanished.
if isfield(c,'visibleMisses') && ~isempty(obs.xy)
    local=toLocal(map.xy,pose); bearing=abs(atan2(local(:,2),local(:,1)));
    visible=vecnorm(local,2,2)<0.9*c.sensorRange & bearing<0.9*c.sensorFov/2;
    map.misses(visible & ~used)=map.misses(visible & ~used)+1;
    keep=keep & map.misses<c.visibleMisses;
end
names=fieldnames(map);
for k=1:numel(names), map.(names{k})=map.(names{k})(keep,:); end
% Merge close compatible duplicates after scan association.
i=1;
while i<size(map.xy,1)
    d=vecnorm(map.xy(i+1:end,:)-map.xy(i,:),2,2);
    merge=find(d<c.merge & map.colour(i+1:end)==map.colour(i) & abs(map.z(i+1:end)-map.z(i))<0.8)+i;
    for j=flipud(merge(:))'
        w=map.hits([i j]); map.xy(i,:)=(w'*map.xy([i j],:))/sum(w);
        map.hits(i)=sum(w); map.vote(i)=map.vote(i)+map.vote(j);
        map.last(i)=max(map.last([i j]));
        for k=1:numel(names), map.(names{k})(j,:)=[]; end
    end
    i=i+1;
end
end
