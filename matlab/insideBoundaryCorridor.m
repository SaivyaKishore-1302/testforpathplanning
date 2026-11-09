function inside=insideBoundaryCorridor(points,b,clearance)
%INSIDEBOUNDARYCORRIDOR Signed local segments, with bounded end extension.
inside=true(size(points,1),1);
for side=1:2
    if side==1, boundary=b.left; else, boundary=b.right; end
    if size(boundary,1)<2, inside(:)=false; return; end
    for i=1:size(points,1)
        [~,~,q,j]=projectPolyline(points(i,:),boundary);
        t=boundary(j+1,:)-boundary(j,:); t=t/max(norm(t),eps);
        n=[-t(2),t(1)]; lateral=dot(points(i,:)-q,n);
        if side==1, good=lateral<=-clearance; else, good=lateral>=clearance; end
        along=dot(points(i,:)-q,t);
        inside(i)=inside(i) && good && abs(along)<6;
    end
end
end
