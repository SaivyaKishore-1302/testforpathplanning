function obs=simulateSensor(env,v,c,time,stream)
% ONLY this perception module may inspect environment cones.
% No global cone IDs or true world coordinates are exposed in observations.
obs=struct('xy',zeros(0,2),'colour',zeros(0,1),'sigma',zeros(0,1),'z',zeros(0,1));
if time>=c.outageStart && time<c.outageStart+c.outageDuration, return; end
p=toLocal(env.cones.xy,[v.x v.y v.yaw]);
r=vecnorm(p,2,2); bearing=atan2(p(:,2),p(:,1));
% Range-dependent grade allowance sees sloping road ahead while retaining
% strict deck separation at a nearby projected overpass crossing.
allowance=0.9;
if isfield(c,'maxGrade'), allowance=allowance+c.maxGrade*r; end
visible=r<=c.range & abs(bearing)<=c.fov/2 & abs(env.cones.z-v.z)<allowance;
ids=find(visible); perfect=strcmpi(c.mode,'Perfect');
if ~perfect, ids=ids(rand(stream,numel(ids),1)>=c.missRate); end
r=r(ids); sig=zeros(numel(ids),1);
if ~perfect, sig=c.sigma+c.distanceNoise*r; end
obs.xy=p(ids,:)+randn(stream,numel(ids),2).*sig;
obs.colour=env.cones.colour(ids); obs.sigma=sig; obs.z=env.cones.z(ids);
if ~perfect
    wrong=rand(stream,numel(ids),1)>c.colourAccuracy;
    obs.colour(wrong)=3-obs.colour(wrong);
    count=floor(c.falseRate)+(rand(stream)<mod(c.falseRate,1));
    if count>0
        rr=c.range*sqrt(rand(stream,count,1)); aa=c.fov*(rand(stream,count,1)-0.5);
        obs.xy=[obs.xy;rr.*[cos(aa),sin(aa)]];
        obs.colour=[obs.colour;1+(rand(stream,count,1)>0.5)];
        obs.sigma=[obs.sigma;repmat(c.sigma+0.1,count,1)];
        obs.z=[obs.z;repmat(v.z,count,1)];
    end
end
end
