function z=environmentElevation(env,xy,previousHeight)
% Environment surface query, independent of all evaluation results.
% Previous physical height keeps a car on its deck at a projected crossing.
% A physical implementation would measure altitude; reference XY is not output.
cost=sum((env.reference-xy).^2,2)+4*(env.z-previousHeight).^2;
[~,j]=min(cost); z=env.z(j);
end
