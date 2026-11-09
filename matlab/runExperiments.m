function results=runExperiments(base,folder,seeds,includeSweeps)
% Independent closed-loop trials. Common seeds do NOT imply equal observations
% after strategies take different routes; use compareStrategies for pairing.
if nargin<1, base=defaultConfig(); end
if nargin<2, folder=fullfile(pwd,'results','experiments'); end
if nargin<3, seeds=base.seed; end
if nargin<4, includeSweeps=true; end
if ~exist(folder,'dir'), mkdir(folder); end
names={'Perfect','Noise 0.1','Noise 0.3','Miss 10 percent','Miss 30 percent', ...
    'Colour errors 5 percent','Range 8 m','Hairpin','Irregular spacing','Combined', ...
    'Missing blue','Missing yellow','Closely spaced','Narrow and wide','Sensor outage','False positives'};
configs=cell(numel(names),1);
for k=1:numel(names)
    d=base; d.sensor.mode='Memory'; d.sensor.sigma=0; d.sensor.distanceNoise=0;
    d.sensor.missRate=0; d.sensor.colourAccuracy=1; d.sensor.falseRate=0;
    d.sensor.outageStart=inf; d.sensor.outageDuration=0;
    switch k
        case 1, d.sensor.mode='Perfect';
        case 2, d.sensor.sigma=0.1;
        case 3, d.sensor.sigma=0.3;
        case 4, d.sensor.missRate=0.1;
        case 5, d.sensor.missRate=0.3;
        case 6, d.sensor.colourAccuracy=0.95;
        case 7, d.sensor.range=8;
        case 8, d.track.type='Hairpin';
        case 9, d.track.irregularity=0.55;
        case 10
            d.sensor.sigma=0.2; d.sensor.missRate=0.2;
            d.sensor.colourAccuracy=0.95; d.sensor.falseRate=0.2;
        case 11, d.track.missingBlue=0.3;
        case 12, d.track.missingYellow=0.3;
        case 13, d.track.spacing=1.1;
        case 14, d.track.widthVariation=0.3;
        case 15, d.sensor.outageStart=8; d.sensor.outageDuration=2;
        case 16, d.sensor.falseRate=1;
    end
    configs{k}=d;
end
group=repmat({'scenario'},numel(names),1); parameter=nan(numel(names),1);
if includeSweeps
    labels={'noise','range','missing'}; values={[0 0.05 0.1 0.2 0.3],[8 12 15 20],[0 0.1 0.3 0.5]};
    for j=1:3
        for value=values{j}
            d=base; d.sensor.mode='Memory';
            switch j
                case 1, d.sensor.sigma=value; d.sensor.distanceNoise=0;
                case 2, d.sensor.range=value;
                case 3, d.sensor.missRate=value;
            end
            names{end+1}=sprintf('%s %.2f',labels{j},value); configs{end+1}=d; %#ok<AGROW>
            group{end+1,1}=labels{j}; parameter(end+1,1)=value; %#ok<AGROW>
        end
    end
end
rows=struct([]); total=numel(configs)*numel(seeds)*3; count=0;
for k=1:numel(configs)
    for seed=seeds(:)'
        d=configs{k}; d.seed=seed; track=generateTrack(d);
        for strategy=1:3
            count=count+1; d.planner.strategy=strategy;
            fprintf('[%d/%d] %s | seed %d | strategy %d\n',count,total,names{k},seed,strategy);
            [row,state]=runHeadless(d,track);
            row.scenario=names{k}; row.group=group{k}; row.parameter=parameter(k);
            rows=[rows;row]; %#ok<AGROW>
            results=struct2table(rows);
            % Incremental checkpoints preserve completed trials if interrupted.
            writetable(results,fullfile(folder,'experiments.csv'));
            save(fullfile(folder,'experiments.mat'),'results','base','seeds');
            if ~row.success
                config=d; telemetry=state.log; reason=state.reason; %#ok<NASGU>
                save(fullfile(folder,sprintf('failure_%03d.mat',count)),'config','telemetry','reason');
            end
            drawnow
        end
    end
end
plotExperimentResults(results,folder);
end
