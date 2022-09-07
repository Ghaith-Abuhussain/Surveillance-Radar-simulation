clear all ; clc ; close all ;

% Radar Parameters
% Radar charactaristic parameters
pd = 0.9;                              % Probability of detection
pfa = 1e-6;                            % Probability of false alarm
max_range = 5000;                     % Maximum unambiguous range
range_res = 100;                        % Required range resolution
tgt_rcs = 10;                          % Required target radar cross section


prop_speed = physconst('LightSpeed');   % Propagation speed
pulse_bw = prop_speed./(2*range_res);               % Pulse bandwidth
pulse_width = 10/pulse_bw;               % Pulse width
prf = prop_speed./(2*max_range);        % Pulse repetition frequency
%prf = prop_speed/(2*(max_range));      % Pulse repetition frequency
fs = 2*pulse_bw;                        % Sampling rate
num_pulse_int = 10;                     % Number of integrated pulses
fc = 10e9;                              % Operating frequency
lambda = prop_speed/fc;                 % The wavelength
tx_gain = 35;
sensorheight = 40;

snr_min = albersheim(pd, pfa, num_pulse_int);
peak_power = 400;%radareqpow(lambda,max_range,snr_min,pulse_width,...
    %'RCS',tgt_rcs,'Gain',tx_gain);

% Create the grids
fast_time_grid = unigrid(0,1/fs,1/max(prf),'[)');
slow_time_grid = (0:num_pulse_int-1)/max(prf);


% Modeling The Basics Of Radar Components
waveform = phased.RectangularWaveform(...
    'PulseWidth',pulse_width,...
    'PRF',prf,...
    'SampleRate',fs);

receiver = phased.ReceiverPreamp(...
    'Gain',tx_gain,...
    'NoiseFigure',0,...
    'SampleRate',fs,...
    'EnableInputPort',true,'PhaseNoiseInputPort',true,...
    'SeedSource','Property','Seed',2010);

transmitter = phased.Transmitter(...
    'Gain',tx_gain,...
    'PeakPower',peak_power,...
    'InUseOutputPort',true,'CoherentOnTransmit',false,...
    'PhaseNoiseOutputPort',true);

azang = -180:180;
elang = -90:90;
gain = 0;
sArray = CreateAntenna1(azang,elang,gain,fc);

% The Radiator and collector objects
radiator = phased.Radiator(...
    'Sensor',sArray,...
'OperatingFrequency',fc);
collector = phased.Collector(...
    'Sensor',sArray,...
    'OperatingFrequency',fc);

% Create FreeSpace Channel Response
channel = phased.FreeSpace(...
    'SampleRate',fs,...
    'TwoWayPropagation',true,...
    'OperatingFrequency',fc);
%%
figure(1);
patternAzimuth(sArray,fc,[0],'Type','powerdb');


%%
% PlatForms
sensormotion = phased.Platform(...
    'InitialPosition',[0; 0; sensorheight],...
    'Velocity',[0; 0; 0]);

% redefine Targets
tgtpos = [[700 0 400]',[2000 500 400]',[-5000 3000 500]',[7800 -1980 300]',[-5800 -5000 300]',[-4300 -4000 300]'];
Acceleration = [[1 0 0]',[3 0 -8]',[0 3 4]',[1 -1 3]',[2 -1 0]',[3 0 -1]'];

tgtvel = [[50 0 0]',[40 30 0]',[-45 30 0]',[40 -35 0]',[33 -25 0]',[32 -14 0]'];
tgtmotion = phased.Platform('MotionModel','Acceleration','InitialPosition',...
    tgtpos,'InitialVelocity',tgtvel,'Acceleration',Acceleration);
tgtrcs = [1 2.2 1.1 1.05 1.2 1.09];
target = phased.RadarTarget('MeanRCS',tgtrcs,'Model','Swerling2',...
    'OperatingFrequency',fc);
target.SeedSource = 'Property';
target.Seed = 2007;


% Modeling Clutter

% blind speeed
prf = waveform.PRF;
wavelength = prop_speed/fc;
blindspd = dop2speed(prf,wavelength)/2; % half to compensate round trip
trgamma = 15;

clutter = phased.ConstantGammaClutter('Sensor',sArray,...
    'PropagationSpeed',radiator.PropagationSpeed,'EarthModel','Curved',...
    'OperatingFrequency',radiator.OperatingFrequency,...
    'SampleRate',waveform.SampleRate,'TransmitSignalInputPort',true,...
    'PRF',waveform.PRF,'Gamma',trgamma,'PlatformHeight',sensorheight,...
    'PlatformSpeed',0,'PlatformDirection',[0;0],...
    'BroadsideDepressionAngle',0,'MaximumRange',max_range,...
    'AzimuthCoverage',360,'PatchAzimuthWidth',5,...
    'SeedSource','Property','Seed',2011);

% CFAR Detector
Ntraining = 50;
Nguard = 8;
Pfa_goal = pfa;
detector = phased.CFARDetector('Method','GOCA',...
    'NumTrainingCells',Ntraining,'NumGuardCells',Nguard,...
    'ProbabilityFalseAlarm',Pfa_goal,'ThresholdOutputPort',true,...
    'ThresholdFactor','custom',...
  'CustomThresholdFactor',3);


% Simulation Loop
range_gates = prop_speed*fast_time_grid/2;

% Time Variant gain
tvg = phased.TimeVaryingGain(...
    'RangeLoss',2*fspl(range_gates,lambda),...
    'ReferenceLoss',2*fspl(max(range_gates),lambda));

%Matched Filter
matchingcoeff = getMatchedFilter(waveform);
matchedfilter = phased.MatchedFilter('Coefficients',matchingcoeff);


% Scan Angles
ang = [-180:180];
num_360_rotations = 10;
numscans = numel(ang);
est_range = [];
est_angle = [];

int_pulses = zeros(numel(fast_time_grid),numscans);
screens = zeros(numel(fast_time_grid),numscans,num_360_rotations);
screens1 = zeros(numel(fast_time_grid),numscans,num_360_rotations);


% Scinario View
BeamSteering = [ang(1);0];
r_update = 1/4;
%%
sceneview = phased.ScenarioViewer('BeamRange',5.0e3,'BeamWidth',[2;60],'UpdateRate',r_update,...
    'PlatformNames',{'Ground Radar','Vehicle1','Vehicle2','Vehicle3'},'ShowPosition',true,...
    'ShowSpeed',true,'ShowAltitude',true,'ShowLegend',true,'ShowRange',true,...
    'Title','Multiplatform Scenario','BeamSteering',BeamSteering);

%%
ratio = 10;
% Configure your tracker here:
numTracks = 50;
gate = 500/ratio;
vol = 1e9;
beta = 1e-14;
pd = 0.9;
far = 1e-6;
ang1 = [0:360];
max_rng = 10000;
FontSize = 7;
dt = 4;
positionSelector = [1 0 0 0 0 0 ;0 0 1 0 0 0; 0 0 0 0 0 0]; % [x, y, 0]
velocitySelector = [0 1 0 0 0 0 ; 0 0 0 1 0 0 ;0 0 0 0 0 0]; % [vx, vy, 0]
tracker = trackerJPDA(...
    'FilterInitializationFcn',@initekfimm,...
    'MaxNumTracks', numTracks, ...
    'MaxNumSensors', 1, ...
    'AssignmentThreshold',gate, ...
    'DeathRate',0.01,...
    'TrackLogic','Integrated',...
    'DetectionProbability', pd, ...
    'ConfirmationThreshold',0.7,...
    'DeletionThreshold',0.1,...
    'ClutterDensity', far/vol, ...
    'NewTargetDensity', beta,...
    'TimeTolerance',10);

% Add a trackPlotter here:
tp = theaterPlot('XLim',[-max_range max_range],'YLim',[-max_range max_range]);
trackp = trackPlotter(tp,'DisplayName','Tracks','HistoryDepth',50,'LabelOffset',[0 0 0],'ConnectHistory','on','ColorizeHistory','on' );
hold on;
    h1 = plot(0.2*max_rng*cosd(ang1),0.2*max_rng*sind(ang1),0.4*max_rng*cosd(ang1),0.4*max_rng*sind(ang1),...
    0.6*max_rng*cosd(ang1),0.6*max_rng*sind(ang1),...
    0.8*max_rng*cosd(ang1),0.8*max_rng*sind(ang1),...
    1*max_rng*cosd(ang1),1*max_rng*sind(ang1),[0 max_rng*cosd(0)],[0 max_rng*sind(0)],[0 max_rng*cosd(30)],[0 max_rng*sind(30)],...
    [0 max_rng*cosd(60)],[0 max_rng*sind(60)],...
    [0 max_rng*cosd(90)],[0 max_rng*sind(90)],...
    [0 max_rng*cosd(120)],[0 max_rng*sind(120)],[0 max_rng*cosd(150)],[0 max_rng*sind(150)],...
    [0 max_rng*cosd(180)],[0 max_rng*sind(180)],[0 max_rng*cosd(210)],[0 max_rng*sind(210)],...
    [0 max_rng*cosd(240)],[0 max_rng*sind(240)],[0 max_rng*cosd(270)],[0 max_rng*sind(270)],...
    [0 max_rng*cosd(300)],[0 max_rng*sind(300)],[0 max_rng*cosd(330)],[0 max_rng*sind(330)]);

    set( h1, 'Color', 'r' );
    text(max_rng/sqrt(2)+0.05,max_rng/sqrt(2)+0.05,num2str(max_rng),'FontSize',FontSize);
    text(0.8*max_rng*(1/sqrt(2)+0.05),0.8*max_rng*(1/sqrt(2)+0.05),num2str(max_rng*4/5),'FontSize',FontSize);
    text(0.6*max_rng*(1/sqrt(2)+0.05),0.6*max_rng*(1/sqrt(2)+0.05),num2str(max_rng*3/5),'FontSize',FontSize);
    text(0.4*max_rng*(1/sqrt(2)+0.05),0.4*max_rng*(1/sqrt(2)+0.05),num2str(max_rng*2/5),'FontSize',FontSize);
    text(0.2*max_rng*(1/sqrt(2)+0.05),0.2*max_rng*(1/sqrt(2)+0.05),num2str(max_rng*1/5),'FontSize',FontSize);

    text(max_rng+0.05,0,'0','FontSize',FontSize);
    text(max_rng*sqrt(3)/2+0.05,max_rng*0.5+0.05,'30','FontSize',FontSize);
    text(max_rng*0.5+0.05,max_rng*sqrt(3)/2+0.05,'60','FontSize',FontSize);
    text(0,max_rng+0.05,'90','FontSize',FontSize);
    text(-max_rng*sqrt(3)/2-0.1,max_rng*0.5+0.05,'150','FontSize',FontSize);
    text(-max_rng*0.5-0.1,max_rng*sqrt(3)/2+0.05,'120','FontSize',FontSize);
    text(-max_rng-0.1,0,'180','FontSize',FontSize);
    text(-max_rng*sqrt(3)/2-0.1,-max_rng*0.5-0.05,'210','FontSize',FontSize);
    text(-max_rng*0.5-0.05,-max_rng*sqrt(3)/2-0.05,'240','FontSize',FontSize);
    text(0-0.05,-max_rng-0.05,'270','FontSize',FontSize);
    text(max_rng*sqrt(3)/2+0.05,-max_rng*0.5-0.05,'330','FontSize',FontSize);
    text(max_rng*0.5+0.05,-max_rng*sqrt(3)/2-0.05,'300','FontSize',FontSize);
   axis off;
%%
X = range_gates'*cosd(ang); Y = range_gates'*sind(ang);
tic;
for i = 1:num_360_rotations
    TH_mat = [];
    pos = [];
    for j = 1:numscans
        
        release(radiator);
        release(collector);
        release(channel);
        release(sArray);
        release(clutter);
        sArray.ElementNormal = [ang(j);10];
        clutter.PlatformDirection = [ang(j);10];
        clutter.PlatformSpeed = 0;%sqrt(sum(sensormotion.Velocity.^2));
        for m = 1:num_pulse_int

            
            pulse = waveform();
            % Update sensor and target positions
            steptime = length(pulse)/fs;
            [sensorpos,sensorvel] = sensormotion(steptime);
            [tgtpos,tgtvel] = tgtmotion(steptime);

            % Calculate the target angles as seen by the sensor
            [~,tgtang] = rangeangle(tgtpos,sensorpos);

            % Simulate propagation of pulse in direction of targets
            [pulse,txstatus,phsnoise] = transmitter(pulse);
            
            txsig = radiator(pulse,tgtang);
            
            txsig = channel(txsig,sensorpos,tgtpos,sensorvel,tgtvel);

            % Reflect pulse off of targets
            tgtsig = target(txsig,true);
            %csig = clutter(pulse(txstatus>0));
            %csig = [csig(length(pulse)*(prf*pulse_width)+1:end);zeros(length(pulse)*(prf*pulse_width),1)];

            % Receive target returns and clutter at sensor
            rxsig = collector(tgtsig,tgtang);

            rxsig = receiver(rxsig,~(txstatus>0),phsnoise);

            % For staggered PRF waveforms, the pulses will be different lengths,
            % here we truncate to the length of the shortest pulse in the PRF
            % stagger
            rxpulses(:,m) = rxsig(1:length(fast_time_grid));
            Pulse_Map(:,j,m) = rxsig(1:length(fast_time_grid));
        end
        rxPulse = rxpulses;
        mfiltOut = matchedfilter(rxPulse);
        
        matchingdelay = size(matchingcoeff,1)-1;
        mfiltOut = buffer(mfiltOut(matchingdelay+1:end),size(mfiltOut,1));
        mfiltOut = tvg(mfiltOut);
        h = [1 -2 1];
        mtiseq = filter(h,1,mfiltOut,[],2);
        mtiseq = pulsint(mtiseq(:,3:end));
        %mtiseq = tvg(mtiseq);
        %mfilt_without_MTI = pulsint(mfiltOut(:,1:end));
        mtiseq(1) = mtiseq(2);
        [x,TH1] = detector(mtiseq,1:length(mtiseq));
        int_pulses(:,j) = x;
        screens1(:,j,i) = mtiseq;
        TH_mat = [TH_mat,TH1];
        
        % display scene
        %sceneview.BeamSteering = [ang(j);10];
        %sceneview(sensorpos,sensorvel,tgtpos,tgtvel);

       % Radar PPI Plot
        figure(2);
        pcolor(X,Y,pow2db(abs(screens1(:,:,i)).^2));
        axis equal tight; 
        shading interp;
        axis off;
        axis([-max_range max_range -max_range max_range]);
        title('Radar PPI Display');
    end
    
    % perform next detection when next update is needed
   	sensormotion(1/r_update);
    tgtmotion(1/r_update);
    screens(:,:,i) = int_pulses;
    C_mat = max(screens1(:,:,i) - TH_mat,0);
    [~,ind_mat] = find_peaks(C_mat);
    %int_pulses1 = int_pulses(1:end-1,:);
    [rngInd,angInd] = ind2sub(size(C_mat),ind_mat);
    est_range = [est_range {range_gates(rngInd)}]; % Estimated range
    est_angle = [est_angle {ang(angInd)}];    % Estimated direction
    for k = 1:length(est_range{i})
        pos = [pos [est_range{i}(k)*cosd(est_angle{i}(k));est_range{i}(k)*sind(est_angle{i}(k));0]];
    end
    time = i*dt;
    for k = 1:length(pos)
        detection(k) = objectDetection(time,pos(:,k)/ratio);
    end
    det = detection(1:length(pos));
    tracks = tracker(det,time);
    
    %State = tracks.State;
    Len = length(pos);
    pos = getTrackPositions(tracks,positionSelector);
    vel = getTrackVelocities(tracks,velocitySelector);
    pos  = pos*ratio;
    vel = vel*ratio;
    labels = string([tracks.TrackID]);
    figure(1);
    trackp.plotTrack(pos,labels);grid on;
end
toc;

%%
% CFAR Detector
Ntraining = 50;
Nguard = 8;
Pfa_goal = pfa;
detector = phased.CFARDetector('Method','GOCA',...
    'NumTrainingCells',Ntraining,'NumGuardCells',Nguard,...
    'ProbabilityFalseAlarm',Pfa_goal,'ThresholdOutputPort',true,...
    'ThresholdFactor','custom',...
  'CustomThresholdFactor',5);

[x,TH] = detector(screens1(:,181),1:length(screens1(:,1)));
%TH = [zeros(24,1);TH];

% Plot the received pulse energy again range
plot(range_gates,pow2db(abs(screens1(:,181)).^2),'b-'...
    ,range_gates,pow2db(abs(TH).^2)); grid on;
title('Fast Time Sequences Using a Uniform PRF');
xlabel('Range (m)'); ylabel('Power (dB)');
legend('After MTI filter');
%%
nn = zeros(200,10);
nn(:,:) = Pulse_Map(:,6,:);
plot(pow2db(abs(Pulse_Map(:,6,7)).^2));
%%

plot(fftshift(abs(fft(nn))));
%%
glo = ones(200,73);
X = range_gates'*cosd(ang); Y = range_gates'*sind(ang);
for ii = 1:num_360_rotations
    for m = 2:length(ang)
        x = X(:,1:m); y = Y(:,1:m);
        figure(1);
        pcolor(x,y,pow2db(abs(screens(:,1:m,ii)).^2));
        axis equal tight; 
        shading interp;
        axis off;
        axis([-max_range max_range -max_range max_range]);
        title('Radar PPI Display');

    end
end
%%
C_mat = max(screens1 - TH_mat,0);
tic;
[peaks,ind_mat] = find_peaks(C_mat);
toc;
[rngInd,angInd] = ind2sub(size(C_mat),ind_mat);
est_range = [est_range {range_gates(rngInd)}]; % Estimated range
est_angle = [est_angle {ang(angInd)}];    % Estimated direction
%Z1 = C_mat(ind_mat(1,1),ind_mat(2,1));

%%
tgtpos = [[700 0 400]',[1600 0 400]',[2900 300 500]',[2900 -200 300]',[9500 200 600]',[6500 300 700]',...
    [5400 -2000 3000]',[4200 -1750 3237]',[1400 900 578]',[1200 -300 4500]'];
tgtvel = [[50 0 0]',[100 0 0]',[-100 0 0]',[200 0 0]',[-100 -150 -40]',[-100 -450 -70]',...
    [-200 +450 -50]',[-150 -400 +50]',[+150 -75 0]',[+30 -80 -40]'];
%%
save ('radar_detections','est_range','est_angle');