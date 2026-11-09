function yes = segmentsCross(a,b,c,d)
% Strict proper intersection; touching endpoints do not count.
u=b-a; v=d-c;
den=u(1)*v(2)-u(2)*v(1);
if abs(den)<1e-10, yes=false; return; end
w=c-a; t=(w(1)*v(2)-w(2)*v(1))/den;
r=(w(1)*u(2)-w(2)*u(1))/den;
yes=t>1e-7 && t<1-1e-7 && r>1e-7 && r<1-1e-7;
end
