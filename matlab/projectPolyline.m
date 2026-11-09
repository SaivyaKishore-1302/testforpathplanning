function [d,s,q,idx] = projectPolyline(p,line)
% Exact closest segment projection; never compares waypoint indices.
if size(line,1)<2
    d=inf(size(p,1),1); s=d; q=nan(size(p)); idx=ones(size(p,1),1); return
end
a=line(1:end-1,:); v=diff(line); vv=sum(v.^2,2);
arc=[0;cumsum(sqrt(vv))];
d=zeros(size(p,1),1); s=d; q=p; idx=d;
for k=1:size(p,1)
    t=max(0,min(1,sum((p(k,:)-a).*v,2)./max(vv,eps)));
    points=a+t.*v; [dd,j]=min(sum((points-p(k,:)).^2,2));
    d(k)=sqrt(dd); s(k)=arc(j)+t(j)*sqrt(vv(j)); q(k,:)=points(j,:); idx(k)=j;
end
end
