function graphics=visualizeSimulation(ax,graphics,s,localView)
% Persistent graphics handles, no repeated axes/figure recreation.
if isempty(graphics)
    hold(ax,'on'); axis(ax,'equal'); grid(ax,'on'); set(ax,'Color','w','FontSize',10);
    xlabel(ax,'X (m)'); ylabel(ax,'Y (m)');
    graphics=struct();
    graphics.truth=line(ax,nan,nan,'Color',[0.6 0.6 0.6],'LineStyle',':','LineWidth',1);
    graphics.fov=patch(ax,nan,nan,[0.8 0.92 1],'FaceAlpha',0.17,'EdgeColor',[0.6 0.8 0.9]);
    graphics.blue=line(ax,nan,nan,'LineStyle','none','Marker','^','MarkerSize',6, ...
        'Color',[0.04 0.3 0.85],'MarkerFaceColor',[0.04 0.3 0.85]);
    graphics.yellow=line(ax,nan,nan,'LineStyle','none','Marker','^','MarkerSize',6, ...
        'Color',[0.8 0.6 0],'MarkerFaceColor',[1 0.8 0.05]);
    graphics.observed=line(ax,nan,nan,'LineStyle','none','Marker','o','MarkerSize',9,'Color',[0.2 0.2 0.2]);
    graphics.anomaly=line(ax,nan,nan,'LineStyle','none','Marker','x','MarkerSize',13,'LineWidth',2,'Color',[0.9 0.1 0.1]);
    graphics.virtual=line(ax,nan,nan,'LineStyle','none','Marker','d','MarkerSize',10,'LineWidth',1.8,'Color',[0.1 0.65 0.8]);
    graphics.voronoi=line(ax,nan,nan,'LineStyle',':','Color',[0.1 0.65 0.65],'LineWidth',1.1);
    graphics.leftBoundary=line(ax,nan,nan,'LineStyle','--','Color',[0.2 0.4 0.85]);
    graphics.rightBoundary=line(ax,nan,nan,'LineStyle','--','Color',[0.85 0.65 0.1]);
    graphics.allEdges=line(ax,nan,nan,'Color',[0.78 0.8 0.83],'LineWidth',0.6);
    graphics.candidate=line(ax,nan,nan,'Color',[0.5 0.35 0.8],'LineStyle',':','LineWidth',1);
    graphics.rejected=line(ax,nan,nan,'Color',[0.9 0.35 0.32],'LineStyle','--');
    graphics.accepted=line(ax,nan,nan,'Color',[0 0.65 0.4],'LineWidth',1.7);
    graphics.midpoints=line(ax,nan,nan,'LineStyle','none','Marker','.','MarkerSize',13,'Color',[0.75 0.2 0.65]);
    graphics.raw=line(ax,nan,nan,'Color',[0.95 0.45 0.06],'LineStyle','--','LineWidth',1.2);
    graphics.path=line(ax,nan,nan,'Color',[0.05 0.6 0.35],'LineWidth',2.4);
    graphics.trajectory=line(ax,nan,nan,'Color',[0.15 0.15 0.2],'LineWidth',1);
    graphics.car=patch(ax,nan,nan,[0.15 0.2 0.27],'EdgeColor','k','LineWidth',1.2);
    graphics.heading=line(ax,nan,nan,'Color',[0.9 0.2 0.15],'LineWidth',2.5);
    graphics.steering=line(ax,nan,nan,'Color',[0.6 0.15 0.75],'LineWidth',2);
    graphics.target=line(ax,nan,nan,'LineStyle','none','Marker','p','MarkerSize',12, ...
        'MarkerFaceColor',[0.85 0.2 0.15],'Color',[0.85 0.2 0.15]);
    names=fieldnames(graphics);
    for k=1:numel(names)
        set(graphics.(names{k}),'HitTest','off','PickableParts','none');
    end
    if ~localView
        set(graphics.blue,'HitTest','on','PickableParts','visible','Tag','PhysicalBlueCones');
        set(graphics.yellow,'HitTest','on','PickableParts','visible','Tag','PhysicalYellowCones');
    end
end
c=s.config; env=s.environment; plan=s.plan; pose=[s.vehicle.x s.vehicle.y s.vehicle.yaw];
if localView && c.display.rotate
    transform=@(p) toLocal(p,pose); xlabel(ax,'Forward (m)'); ylabel(ax,'Left (m)');
else
    transform=@(p) p; xlabel(ax,'X (m)'); ylabel(ax,'Y (m)');
end
if localView
    points=zeros(0,2); colours=zeros(0,1);
    if ~isempty(s.map), points=s.map.xy; colours=s.map.colour; end
else, points=env.cones.xy; colours=env.cones.colour; end
put(graphics.blue,transform(points(colours==1,:)));
put(graphics.yellow,transform(points(colours==2,:)));
put(graphics.observed,transform(toWorld(s.observations.xy,pose)));
put(graphics.anomaly,transform(toWorld(plan.recovery.physical(plan.recovery.anomaly,:),plan.pose)));
put(graphics.virtual,transform(toWorld([plan.recovery.virtual;plan.boundary.virtual],plan.pose)));
put(graphics.leftBoundary,transform(toWorld(plan.boundary.left,plan.pose)));
put(graphics.rightBoundary,transform(toWorld(plan.boundary.right,plan.pose)));
drawEdges(graphics.voronoi,transform(toWorld(plan.voronoi.vertices,plan.pose)),plan.voronoi.edges,true);
truth=env.reference; if ~c.display.truth, truth=zeros(0,2); end
put(graphics.truth,transform(truth));
tri=plan.tri; p=transform(toWorld(tri.points,plan.pose)); e=tri.edges; f=plan.filter;
drawEdges(graphics.allEdges,p,e,c.display.allEdges);
drawEdges(graphics.candidate,p,e(f.opposite,:),c.display.allEdges);
drawEdges(graphics.accepted,p,e(plan.route.edgeIds,:),c.display.accepted);
selected=false(size(e,1),1); selected(plan.route.edgeIds)=true;
drawEdges(graphics.rejected,p,e(~selected,:),c.display.rejected);
mid=transform(toWorld(f.midpoints(f.valid,:),plan.pose));
if ~c.display.midpoints, mid=zeros(0,2); end
put(graphics.midpoints,mid);
path=plan.worldPath; raw=plan.rawWorld;
if ~c.display.path, path=zeros(0,2); raw=path; end
put(graphics.path,transform(path)); put(graphics.raw,transform(raw));
traj=s.trajectory; if ~c.display.trajectory, traj=zeros(0,2); end
put(graphics.trajectory,transform(traj));
angles=linspace(-c.sensor.fov/2,c.sensor.fov/2,50)';
sector=[0 0;c.sensor.range*[cos(angles),sin(angles)];0 0];
if ~c.display.fov, sector=zeros(0,2); end
put(graphics.fov,transform(toWorld(sector,pose)));
L=c.vehicle.length; W=c.vehicle.width;
car=[-L/2 -W/2;L/3 -W/2;L/2 -W/4;L/2 W/4;L/3 W/2;-L/2 W/2];
put(graphics.car,transform(toWorld(car,pose)));
put(graphics.heading,transform(toWorld([0 0;2.8 0;2.2 0.3;2.8 0;2.2 -0.3],pose)));
put(graphics.steering,transform(toWorld([0 0;2.2*cos(s.vehicle.steer),2.2*sin(s.vehicle.steer)],pose)));
put(graphics.target,transform(s.control.target));
if localView
    if c.display.rotate, xlim(ax,[-5 c.sensor.range+3]); ylim(ax,[-12 12]);
    else, xlim(ax,pose(1)+[-17 17]); ylim(ax,pose(2)+[-17 17]); end
    title(ax,sprintf('LOCAL | %s | confidence %.2f',plan.status,plan.confidence),'Interpreter','none');
else
    p=env.reference; xlim(ax,[min(p(:,1))-8,max(p(:,1))+8]); ylim(ax,[min(p(:,2))-8,max(p(:,2))+8]);
    if c.display.truth, suffix=' | grey reference: evaluation only'; else, suffix=''; end
    title(ax,['GLOBAL | ' env.type suffix],'Interpreter','none');
end
end

function put(h,p)
if isempty(p), set(h,'XData',nan,'YData',nan); else, set(h,'XData',p(:,1),'YData',p(:,2)); end
end
function drawEdges(h,p,e,show)
if ~show || isempty(e), put(h,zeros(0,2)); return; end
x=[p(e(:,1),1),p(e(:,2),1),nan(size(e,1),1)]';
y=[p(e(:,1),2),p(e(:,2),2),nan(size(e,1),1)]';
set(h,'XData',x(:),'YData',y(:));
end
