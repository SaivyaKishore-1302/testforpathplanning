function p = toLocal(p,pose)
% Row-vector rotation: world -> vehicle (x forward, y left).
R = [cos(pose(3)) -sin(pose(3)); sin(pose(3)) cos(pose(3))];
p = (p-pose(1:2))*R;
end
