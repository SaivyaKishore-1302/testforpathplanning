function tri=computeDelaunay(points,colours)
tri=struct('points',zeros(0,2),'colours',zeros(0,1),'triangles',zeros(0,3), ...
    'edges',zeros(0,2),'status','Insufficient cones');
keep=all(isfinite(points),2) & ismember(colours,[1 2]);
points=points(keep,:); colours=colours(keep);
[~,ids]=unique(round(points/0.08),'rows','stable');
tri.points=points(ids,:); tri.colours=colours(ids);
if size(tri.points,1)<3, return; end
if rank(tri.points-mean(tri.points,1),1e-8)<2, tri.status='Collinear cones'; return; end
try
    dt=delaunayTriangulation(tri.points);
    tri.triangles=dt.ConnectivityList; tri.edges=edges(dt); tri.status='OK';
catch ex
    tri.status=['Triangulation failed: ' ex.identifier];
end
end
