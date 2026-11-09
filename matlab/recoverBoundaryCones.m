function r=recoverBoundaryCones(points,colour,c)
%RECOVERBOUNDARYCONES Conservative local geometry, without track truth or IDs.
% Fit a quadratic in a neighbour PCA frame, never sort a circuit by world X.
% Require bracketing same-colour support AND opposite-boundary width evidence.
r=struct('physical',points,'corrected',points,'colour',colour, ...
    'anomaly',false(size(colour)),'virtual',zeros(0,2),'residual',zeros(size(colour)));
if ~c.recovery.enabled, return; end
proposal=points;
for i=1:size(points,1)
    ids=find(colour==colour(i)); ids(ids==i)=[];
    [ds,order]=sort(vecnorm(points(ids,:)-points(i,:),2,2));
    ids=ids(order(ds<c.recovery.fitRadius)); ids=ids(1:min(6,numel(ids)));
    if numel(ids)<4, continue; end
    q=points(ids,:); origin=mean(q,1);
    [~,~,V]=svd(q-origin,0); tangent=V(:,1); normal=[-tangent(2);tangent(1)];
    x=(q-origin)*tangent; y=(q-origin)*normal;
    xi=(points(i,:)-origin)*tangent;
    if min(x)>xi-0.5 || max(x)<xi+0.5, continue; end
    A=[ones(size(x)),x,x.^2];
    if rcond(A'*A)<1e-8, continue; end
    coef=A\y;
    if max(abs(A*coef-y))>c.recovery.fitTolerance, continue; end
    yi=[1 xi xi^2]*coef; nominal=origin+xi*tangent'+yi*normal';
    residual=norm(nominal-points(i,:)); r.residual(i)=residual;
    if residual<c.recovery.threshold || residual>c.width, continue; end
    % Opposite points must span the same local station. This rejects corners
    % and ambiguous endpoints rather than inventing unsupported replacements.
    opposite=points(colour~=colour(i),:);
    xo=(opposite-origin)*tangent; yo=(opposite-origin)*normal;
    good=abs(xo-xi)<c.recovery.fitRadius;
    xo=xo(good); yo=yo(good);
    if numel(xo)<3 || min(xo)>xi || max(xo)<xi, continue; end
    B=[ones(size(xo)),xo,xo.^2];
    if rcond(B'*B)<1e-8, continue; end
    beta=B\yo;
    if max(abs(B*beta-yo))>c.recovery.fitTolerance, continue; end
    oppositeY=[1 xi xi^2]*beta;
    widthNominal=abs(yi-oppositeY);
    widthObserved=abs((points(i,:)-origin)*normal-oppositeY);
    if abs(widthNominal-c.width)<c.recovery.widthTolerance && ...
            abs(widthObserved-c.width)>c.recovery.threshold
        r.anomaly(i)=true; proposal(i,:)=nominal;
    end
end
r.corrected(r.anomaly,:)=proposal(r.anomaly,:);
r.virtual=r.corrected(r.anomaly,:);
end
