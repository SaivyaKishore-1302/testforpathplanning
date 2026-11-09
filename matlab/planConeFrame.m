function response = planConeFrame(request)
%PLANCONEFRAME Run the existing hybrid planner on one JSON cone-map snapshot.
% Cone observations, ego pose and previous path share a planar world frame.
frame = jsondecode(request);
cones = double(frame.cones);
if isempty(cones), cones = zeros(0,4); end
validateattributes(cones,{'double'},{'2d','ncols',4,'real','finite'});
pose = double(frame.pose(:)');
validateattributes(pose,{'double'},{'numel',3,'real','finite'});
if any(~ismember(cones(:,3),[1 2])) || any(cones(:,4)<0)
    error('planConeFrame:InvalidCones','Use blue=1, yellow=2 and nonnegative variance.');
end
previous = zeros(0,2);
if isfield(frame,'previous_path') && ~isempty(frame.previous_path)
    previous = double(frame.previous_path);
    validateattributes(previous,{'double'},{'2d','ncols',2,'real','finite'});
end
speed = 0;
if isfield(frame,'speed_mps')
    speed = double(frame.speed_mps);
    validateattributes(speed,{'double'},{'scalar','real','finite','nonnegative'});
end
map = struct('xy',cones(:,1:2),'colour',cones(:,3), ...
    'variance',cones(:,4),'hits',ones(size(cones,1),1),'z',zeros(size(cones,1),1));
ego = struct('x',pose(1),'y',pose(2),'yaw',pose(3),'z',0, ...
    'speed',speed,'steer',0,'accel',0);
config = defaultConfig();
plan = planPath(map,pose,0,config.planner,previous,ego);
speedLimit = 0;
if plan.valid
    speedLimit = min(plan.speedLimit,config.planner.hybrid.normalSpeed);
end
response = jsonencode(struct('valid',plan.valid,'status',plan.status, ...
    'method',plan.selected,'path_xy',plan.worldPath,'confidence',plan.confidence, ...
    'speed_limit_mps',speedLimit));
end
