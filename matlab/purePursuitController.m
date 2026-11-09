function u=purePursuitController(v,pose,plan,c,age)
u=struct('steer',0,'accel',-c.brake,'target',nan(1,2),'targetSpeed',0);
if ~plan.valid || size(plan.worldPath,1)<2 || age>0.35, return; end
path=plan.worldPath;
[distance,station]=projectPolyline(pose(1:2),path);
s=[0;cumsum(vecnorm(diff(path),2,2))]; remaining=s(end)-station;
if remaining<0.7 || distance>3, return; end
look=c.lookahead+c.lookaheadGain*v.speed;
avoiding=contains(plan.smoothStatus,'offset route');
if isfield(plan,'cautious'), avoiding=avoiding || plan.cautious; end
if avoiding, look=min(look,1.6); end
targetS=min(s(end),station+look);
target=interp1(s,path,targetS,'linear'); local=toLocal(target,pose);
if local(1)<=0.1, return; end
kappa=2*local(2)/max(sum(local.^2),0.1);
u.steer=max(-c.maxSteer,min(c.maxSteer,atan(c.wheelbase*kappa)));
% Include forward path curvature, rather than relying only on pursuit steer.
curvature=abs(kappa);
if size(path,1)>2
    d=diff(path); ang=unwrap(atan2(d(:,2),d(:,1)));
    ds=(vecnorm(d(1:end-1,:),2,2)+vecnorm(d(2:end,:),2,2))/2;
    ahead=s(2:end-1)>station & s(2:end-1)<station+look*2;
    kk=abs(diff(ang))./max(ds,0.1);
    if any(ahead), curvature=max(curvature,max(kk(ahead))); end
end
cornerSpeed=sqrt(c.lateralAccel/max(curvature,0.001));
% Leave enough observed path to brake to a stop before its end.
horizonSpeed=sqrt(2*c.brake*max(0,remaining-1));
u.targetSpeed=min([c.speed cornerSpeed horizonSpeed]);
if isfield(plan,'speedLimit'), u.targetSpeed=min(u.targetSpeed,plan.speedLimit); end
if avoiding, u.targetSpeed=min(u.targetSpeed,1.5); end
u.accel=max(-c.brake,min(c.accel,2*(u.targetSpeed-v.speed)));
u.target=target;
end
