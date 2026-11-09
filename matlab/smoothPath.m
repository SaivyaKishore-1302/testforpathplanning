function [path,status]=smoothPath(raw,tri,f)
% PCHIP is shape-preserving cubic interpolation in arc length.
% Validate against raw route and observed boundaries. Fail back visibly to raw.
path=raw; status='Raw midpoint polyline';
if size(raw,1)<3, return; end
s=[0;cumsum(vecnorm(diff(raw),2,2))];
if any(diff(s)<0.05), status='Smoothing rejected: duplicate midpoint'; return; end
samples=unique([(0:0.18:s(end))';s(end)]);
candidate=[pchip(s,raw(:,1),samples),pchip(s,raw(:,2),samples)];
if any(projectPolyline(candidate,raw)>0.3)
    status='Smoothing rejected: deviates from gate route'; return
end
for k=1:size(f.boundaries,1)
    ij=f.boundaries(k,:); a=tri.points(ij(1),:); b=tri.points(ij(2),:);
    for j=1:size(candidate,1)-1
        if segmentsCross(candidate(j,:),candidate(j+1,:),a,b)
            status='Smoothing rejected: crosses observed boundary'; return
        end
    end
end
% Keep the spline inside the union of Delaunay triangles touched by the raw
% midpoint route. This verifies observed free corridor, without hidden truth.
dt=triangulation(tri.triangles,tri.points);
validTriangles=[];
for j=1:size(raw,1)-1
    probe=raw(j,:)+(0:0.1:1)'*(raw(j+1,:)-raw(j,:));
    validTriangles=[validTriangles;pointLocation(dt,probe)]; %#ok<AGROW>
end
containing=pointLocation(dt,candidate);
% At triangle edges pointLocation may choose either adjoining triangle; raw
% route endpoints are allowed. Interior spline points require a known cell.
if any(isnan(containing))
    status='Smoothing rejected: outside observed triangulation'; return
end
if any(~ismember(containing(2:end-1),unique(validTriangles(~isnan(validTriangles)))))
    status='Smoothing rejected: leaves observed gate corridor'; return
end
path=candidate; status='Validated cubic centreline';
end
