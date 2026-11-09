function comparison=runHybridAblation(folder,seeds,ids)
%RUNHYBRIDABLATION Compare the combined planner with Delaunay plus same safety.
if nargin<1, folder=fullfile(pwd,'results','ablation'); end
if nargin<2, seeds=2685; end
if nargin<3, ids=[2 3 5 6 7 9 10 18 20 24 32 34]; end
baseline=runHybridSuite(fullfile(folder,'delaunay'),seeds,ids,'delaunay');
hybrid=runHybridSuite(fullfile(folder,'hybrid'),seeds,ids,'hybrid');
comparison=[baseline;hybrid]; writetable(comparison,fullfile(folder,'comparison.csv'));
end
