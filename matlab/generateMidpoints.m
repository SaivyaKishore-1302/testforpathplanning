function [points,edgeIds]=generateMidpoints(filtered)
edgeIds=find(filtered.valid); points=filtered.midpoints(edgeIds,:);
if isempty(points), return; end
[~,u]=unique(round(points/0.12),'rows','stable');
points=points(u,:); edgeIds=edgeIds(u);
end
