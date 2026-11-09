function runScenario(name,seed)
%RUNSCENARIO Open a prepared case in the original animated simulator.
if nargin<1, name='Staggered sides'; end
if nargin<2, seed=2685; end
[c,track,meta]=makeScenario(name,seed); c.sim.scenario=meta;
main(c,track);
end
