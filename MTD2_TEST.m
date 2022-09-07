clear all ; clc ; close all ;

% Radar Parameters
% Radar charactaristic parameters
pd = 0.9;                              % Probability of detection
pfa = 1e-6;                            % Probability of false alarm
max_range = 5000;                     % Maximum unambiguous range
range_res = 20;                        % Required range resolution
tgt_rcs = 10;                          % Required target radar cross section


prop_speed = physconst('LightSpeed');   % Propagation speed
pulse_bw = prop_speed./(2*range_res);               % Pulse bandwidth
pulse_width = 1/pulse_bw;               % Pulse width
prf = prop_speed./(2*[max_range]);        % Pulse repetition frequency
%prf = prop_speed/(2*(max_range));      % Pulse repetition frequency
fs = 2*pulse_bw;                        % Sampling rate
num_pulse_int = 128;                     % Number of integrated pulses
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
    'PulseWidth',1/pulse_bw,...
    'PRF',prf,...
    'SampleRate',fs);
%%

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
sArray = CreateAntenna(azang,elang,gain,fc);

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
pattern(sArray,fc,[0],'Type','powerdb');


%%


% Modeling Clutter

% blind speeed
prf = waveform.PRF;
wavelength = prop_speed/fc;
blindspd = dop2speed(prf,wavelength)/2; % half to compensate round trip

trgamma = 40;


clutter = phased.ConstantGammaClutter('Sensor',sArray,...
    'PropagationSpeed',radiator.PropagationSpeed,'EarthModel','Curved',...
    'OperatingFrequency',radiator.OperatingFrequency,...
    'SampleRate',waveform.SampleRate,...
    'PRF',waveform.PRF,'Gamma',trgamma,'PlatformHeight',sensorheight,...
    'PlatformSpeed',90,'PlatformDirection',[0;0],...
    'BroadsideDepressionAngle',0,'MaximumRange',max_range,...
    'AzimuthCoverage',360,'PatchAzimuthWidth',5,...
    'SeedSource','Property','Seed',2011);

% CFAR Detector
Ntraining = 16;
Nguard = 4;
Pfa_goal = pfa;
detector = phased.CFARDetector('Method','GOCA','NumTrainingCells',16,...
    'NumGuardCells',4,'ProbabilityFalseAlarm',1e-6,'ThresholdOutputPort',true,...
    'ThresholdFactor','custom','CustomThresholdFactor',8);


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
ang = [60:-1:-60];
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
% Configure your tracker here:
ratio = 1;
numTracks = 50;
gate = 500/ratio;
vol = 1e9;
beta = 1e-14;
pd = 0.9;
far = 1e-6;
ang1 = [0:360];
max_rng = 5000;
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
    'ConfirmationThreshold',0.95,...
    'DeletionThreshold',0.15,...
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
% PlatForms
sensormotion = phased.Platform(...
    'InitialPosition',[0; 0; sensorheight],...
    'Velocity',[0; 0; 0]);

% redefine Targets
tgtpos = [[3000 0 400]',[3000 0 400]',[1111 400 400]',[800 -30 400]',[1561 -1000 400]'];
Acceleration = [[0 2 0]',[0 3 0]',[0 0 0]',[1 -2 0]',[2 -2 0]'];

tgtvel = [[50 10 0]',[-40 -40 0]',[80 -80 0]',[40 -10 0]',[24 -20 0]'];
tgtmotion = phased.Platform('MotionModel','Acceleration','InitialPosition',...
    tgtpos,'InitialVelocity',tgtvel,'Acceleration',Acceleration);
tgtrcs = [5.3,5.3 6.1 5.09,5.32];
target = phased.RadarTarget('MeanRCS',tgtrcs,'Model','Swerling2',...
    'OperatingFrequency',fc);
target.SeedSource = 'Property';
target.Seed = 2007;


%%
fft_size = 128;
test_sig = zeros(length(range_gates),fft_size,numscans,num_360_rotations);

load("Clutter_signal202.mat");
load("Clutter_signal203.mat");

C_Sig = zeros(length(fast_time_grid),numscans);
speed_axis = dop2speed([-prf/2 + prf/fft_size:prf/fft_size:prf/2],lambda) ;
density = 8e-4;
rngBinsNum = 500;
rain_speed = 10;
distortion = 2;
radar_steering =120;
rain_steering = 20;
Noise_level = 30;
tic;
position = [];
vel = [];
ang = [60:-1:-60];
for i = 1:num_360_rotations
    TH_mat = [];
    pos = [];
    for j = 1:numscans
        
        release(radiator);
        release(collector);
        release(channel);
        release(sArray);
        release(clutter);
        release(detector);
        sArray.ElementNormal = [ang(j);10];
        clutter.PlatformDirection = [ang(j);10];
        signal = rain_clutter(density,fft_size,rngBinsNum,rain_speed,...
            distortion,ang(j),rain_steering,speed_axis,Noise_level);
        clutter.PlatformSpeed = 50;%sqrt(sum(sensormotion.Velocity.^2));
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
            csig =  [clutter_sig(:,181);zeros(length(txsig) - length(clutter_sig(:,181)),1)];
            cwcig = [clutter_sig_wind(:,181);zeros(length(txsig) - length(clutter_sig_wind(:,181)),1)];

            % Receive target returns and clutter at sensor
            rxsig = collector(tgtsig,tgtang);

            rxsig = receiver(rxsig,~(txstatus>0),phsnoise)+csig;

            % For staggered PRF waveforms, the pulses will be different lengths,
            % here we truncate to the length of the shortest pulse in the PRF
            % stagger
            rxpulses(:,m) = rxsig(1:length(fast_time_grid))+signal(:,m);
            Pulse_Map(:,j,m) = rxsig(1:length(fast_time_grid));
        end
        rxPulse = rxpulses;
        mfiltOut = matchedfilter(rxPulse);
        
        matchingdelay = size(matchingcoeff,1)-1;
        mfiltOut = buffer(mfiltOut(matchingdelay+1:end),size(mfiltOut,1));
        mfiltOut = tvg(mfiltOut);
        test_sig(:,:,j,i) = mfiltOut;
        
        
        
        % display scene
        %sceneview.BeamSteering = [ang(j);10];
        %sceneview(sensorpos,sensorvel,tgtpos,tgtvel);
        % Radar PPI Plot
        
        
    end
    position = [position {tgtpos}];
    vel = [vel {tgtvel}];
    sensormotion(1/r_update);
    tgtmotion(1/r_update);
    
    % perform next detection when next update is needed
    if(abs(ang(end)) == abs(ang(1) && abs(ang(end)) == 180))
        ang = [180:-1:-180];
    else
        if(ang(1) > ang(end))
            ang = sort(ang,'ascend');
        else
            ang = sort(ang,'descend');
        end
    end
   	
    
end
toc;
%%
ang = [60:-1:-60];
fftsize = 128;
fft_size = 128;
%load('tst_case3');
fd = dop2speed([-prf/2 + prf/(fft_size) :prf/(fft_size):prf/2 ],lambda);
peaks = [];
ind =[];
detection = [];
C_map_sig = zeros(200,1);
C_map = zeros(length(range_gates),fftsize);
Z_reports = [];
All_pos = [];
dt = 5;
vel_from_track = [];
all_type_vels = [];
screens1 = zeros(numel(fast_time_grid),numscans,num_360_rotations);
tic;
for m = 1:7
    all_detections = [];
    peaks = [];
    ind =[];
    detection = [];
    All_pos = [];
    C_map_sig = zeros(500,1);
    C_map = zeros(length(range_gates),fftsize);
    for k = 1:numscans
        h = [1 -1];
        mtiseq = filter(h,1,test_sig(:,:,k,m),[],2);
        F_map = fftshift(abs(fft(mtiseq(:,3:end),fftsize,2)),2);
        C_map_sig = (C_map_sig+abs(F_map(:,size(F_map,2)/2+1)))/8;
        C_map = C_map_sig.*ones(size(F_map));
        F = fftshift(abs(fft(mtiseq(:,2:end),fftsize,2)),2);
        detector1 = phased.CFARDetector('Method','GOCA','NumTrainingCells',40,...
        'NumGuardCells',4,'ProbabilityFalseAlarm',1e-6,'ThresholdOutputPort',true,...
        'ThresholdFactor','custom','CustomThresholdFactor',15);

        F(end,:) = F(end-1,:);
        [~,CF_TH] = detector1(F,1:500);
        TH = CF_TH;
        C_Data = max(F - TH,0);
        %figure(1);
        %plot(pow2db(abs(F(:,128))));hold on;plot(pow2db(abs(TH(:,128))));

        detector2 = phased.CFARDetector('Method','GOCA','NumTrainingCells',122,...
        'NumGuardCells',0,'ProbabilityFalseAlarm',1e-6,'ThresholdOutputPort',true,...
        'ThresholdFactor','custom','CustomThresholdFactor',15);
        [~,CF_TH1] = detector2(C_Data',1:128);
        TH = CF_TH1;
        C_Data1 = max(C_Data' - (TH+C_map'),0);
        CC_DATA = max(C_Data1'-0.5*abs(mean(mean(C_Data1(find(C_Data1~=0))))),0);
        screens1(:,k,m) = pulsint(F);
        [~,ind_mat] = find_peaks(CC_DATA);
        ind = [ind,{ind_mat}];
        [rngInd,velInd] = ind2sub(size(CC_DATA),ind_mat);
        peaks = [peaks;{[rngInd;velInd]}];
        if(length([rngInd;velInd]) ~= 0)
            detection = [detection,{[rngInd;velInd;k*ones(1,length(rngInd))]}];
        end


        figure(2);
        X = range_gates'*cosd(ang); Y = range_gates'*sind(ang);
        pcolor(X,Y,pow2db(abs(screens1(:,:,m)).^2));
        axis equal tight; 
        shading interp;
        axis off;
        axis([-max_range max_range -max_range max_range]);
        title('Radar PPI Display');
        %subplot(2,1,1);pcolor(F);shading interp;colorbar;
        %subplot(2,1,2);pcolor(CC_DATA);shading interp;colorbar;
        %pause(0.005)
    end
    
    for i = 1:length(detection)
        for j = 1:size(detection{i},2)
            all_detections = [all_detections detection{i}(:,j)];

        end
    end
    [detection] = merging_detections(all_detections);
    if (length(detection)~=0)
        Z_reports = [Z_reports,{[range_gates(detection(1,:)).*cosd(ang(detection(3,:)));...
            range_gates(detection(1,:)).*sind(ang(detection(3,:)));fd(detection(2,:))]}];
    end
    
        for j = 1:size(detection,2)
            det_pos = [range_gates(detection(1,:)).*cosd(ang(detection(3,:)))...
                ;range_gates(detection(1,:)).*sind(ang(detection(3,:)));zeros(1,size(detection,2))];
        end
        All_pos = [All_pos {det_pos}];
        time = m*dt;
        if(size(det_pos,2) ~=0)
            
               for kn = 1:size(det_pos,2)
                    detectionn(kn) = objectDetection(time,det_pos(:,kn)/ratio);
               end
                det = detectionn(1:size(det_pos,2));
                tracks = tracker(det,time);

                %State = tracks.State;
                Len = length(det_pos);
                positions = getTrackPositions(tracks,positionSelector);
                velocities = getTrackVelocities(tracks,velocitySelector);
                positions = positions*ratio;
                velocities = velocities*ratio;
                vel_from_track = [vel_from_track {velocities}];
                labels = string([tracks.TrackID]);
                figure(1);
                trackp.plotTrack(positions,labels);grid on;
        end
        if(m >= 2)
            velocity = vel{m};
            detection = Z_reports{m};
            PP = position{m};
            [vel_est,vel_err_last,velo2] = velocity_estimation(positions,detection,PP,velocities,velocity,10); 
            figure(3);plot(1:length(vel_est),vel_est,'b*');hold on;plot(1:length(vel_est),velo2,'r*');...
                grid on;title('comparison between real speed and estimated speed');...
                xlabel('target ID');ylabel('speed (m/s)');...
                legend('estimated speed','real speed','Location','best');hold off;
        end
        % perform next detection when next update is needed
    if(abs(ang(end)) == abs(ang(1) && abs(ang(end)) == 180))
        ang = [180:-1:-180];
    else
        if(ang(1) > ang(end))
            ang = sort(ang,'ascend');
        else
            ang = sort(ang,'descend');
        end
    end

end
toc;
%%
fftsize = 128;
fd = dop2speed([-prf/2 + prf/(fftsize) :prf/(fftsize):prf/2 ],lambda) ;
peaks = [];
ind =[];
detection = [];
C_map_sig = zeros(500,1);
C_map = zeros(length(range_gates),fftsize);
tic;
for m = 8
    all_detections = [];
    peaks = [];
    ind =[];
    detection = [];
    C_map_sig = zeros(500,1);
    C_map = zeros(length(range_gates),fftsize);
    for k = find(ang == -9)
        h = [1 -1];
        mtiseq = filter(h,1,test_sig(:,:,k,m),[],2);
        F_map = fftshift(abs(fft(mtiseq(:,3:end),fftsize,2)),2);
        C_map_sig = (C_map_sig+abs(F_map(:,size(F_map,2)/2+1)))/8;
        C_map = C_map_sig.*ones(size(F_map));
        F = fftshift(abs(fft(mtiseq(:,2:end),fftsize,2)),2);
        detector1 = phased.CFARDetector('Method','GOCA','NumTrainingCells',40,...
        'NumGuardCells',4,'ProbabilityFalseAlarm',1e-6,'ThresholdOutputPort',true,...
        'ThresholdFactor','custom','CustomThresholdFactor',15);

        F(end,:) = F(end-1,:);
        [~,CF_TH] = detector1(F,1:500);
        TH = CF_TH;
        C_Data = max(F - TH,0);
        %figure(1);
        %plot(pow2db(abs(F(:,128))));hold on;plot(pow2db(abs(TH(:,128))));
         
        detector2 = phased.CFARDetector('Method','GOCA','NumTrainingCells',122,...
        'NumGuardCells',0,'ProbabilityFalseAlarm',1e-6,'ThresholdOutputPort',true,...
        'ThresholdFactor','custom','CustomThresholdFactor',15);
        [~,CF_TH1] = detector2(C_Data',1:128);
        TH = CF_TH1;
        C_Data1 = max(C_Data' - (TH+C_map'),0);
        A = 0.5*abs(mean(mean(C_Data1(find(C_Data1~=0)))));
        CC_DATA = max(C_Data1'-A,0);
        [~,ind_mat] = find_peaks(CC_DATA);
        ind = [ind,{ind_mat}];
        [rngInd,velInd] = ind2sub(size(CC_DATA),ind_mat);
        peaks = [peaks;{[rngInd;velInd]}];
        if(length([rngInd;velInd]) ~= 0)
            detection = [detection,{[rngInd;velInd;k*ones(1,length(rngInd))]}];
        end


        figure(2);
        subplot(2,1,1);pcolor(fd,range_gates,F);shading interp;colorbar;
        subplot(2,1,2);pcolor(fd,range_gates,CC_DATA);shading interp;colorbar;
        pause(0.005)
    end
    
    for i = 1:length(detection)
        for j = 1:size(detection{i},2)
            all_detections = [all_detections detection{i}(:,j)];

        end
    end
    [detection] = merging_detections(all_detections);

end
toc;
%%
% Tracker_Test 
vel_from_track = [];
dt = 5;
for i = 1:10
    det_pos = position{i};
    
    time = i*dt;
        if(size(det_pos,2) ~=0)
            
               for kn = 1:size(det_pos,2)
                    detectionn(kn) = objectDetection(time,det_pos(:,kn)/ratio);
               end
                det = detectionn(1:size(det_pos,2));
                tracks = tracker(det,time);

                %State = tracks.State;
                Len = length(det_pos);
                positions = getTrackPositions(tracks,positionSelector);
                velocities = getTrackVelocities(tracks,velocitySelector);
                positions = positions*ratio;
                velocities = velocities*ratio;
                vel_from_track = [vel_from_track {velocities}];
                labels = string([tracks.TrackID]);
                figure(1);
                trackp.plotTrack(positions,labels);grid on;
                figure(2);
                plot(sqrt(velocities(:,1).^2 + velocities(:,2).^2),'*')
                text(positions(:,1),positions(:,2),num2str(sqrt(velocities(:,1).^2 + velocities(:,2).^2)))
        end
end
%%
%  Tests 
l = 7;
velocity = vel{l};
detection = Z_reports{l};
PP = position{l};
[vel_est,vel_err_last] = velocity_estimation(positions,detection,PP,velocities,velocity,10);
%%

est_pos   = sqrt(positions(:,1).^2+positions(:,2).^2)';
estm_pos  = sqrt(Z_reports{l}(1,:).^2+Z_reports{l}(2,:).^2);
estm1_pos = sqrt(position{l}(1,:).^2+position{l}(2,:).^2);
estm1_vel = sqrt(vel{l}(1,:).^2 + vel{l}(2,:).^2);
velo = [];
for i = 1:length(est_pos)
    rng = [];
    for k = 1:length(estm_pos)
        rng = [rng abs((estm_pos(k) - est_pos(i)))];
    end
    [~,ind] = min(rng);
    velo = [velo Z_reports{l}(3,ind)/2];
end

velo2 = [];
cos_theta = [];
for i = 1:length(est_pos)
    rng = [];
    for k = 1:length(estm1_pos)
        rng = [rng abs((estm1_pos(k) - est_pos(i)))];
    end
    [~,ind] = min(rng);
    velo2 = [velo2 estm1_vel(ind)];
    if(velo(i) < 0)
        cos_theta = [cos_theta cosd(atand(vel{l}(2,ind)/vel{l}(1,ind))-atand(positions(i,2)/positions(i,1)))*cosd(8)];
    else
        cos_theta = [cos_theta cosd(180+atand(vel{l}(2,ind)/vel{l}(1,ind))-atand(positions(i,2)/positions(i,1)))*cosd(8)];
    end
end

cos_theta2 = cosd(awgn(abs(acosd(velo./velo2)),-9));
vel_est  = abs(velo./cos_theta);
vel_err  = 100*(vel_est - velo2)./velo2;
vel_err2 = 100*(sqrt(velocities(:,1).^2 + velocities(:,2).^2)' - velo2)./velo2;
for i = 1:length(vel_est)
    if(abs(vel_err(i)) >= abs(vel_err2(i)))
        vel_est(i) = sqrt(velocities(i,1).^2 + velocities(i,2).^2)';
    end
end
vel_err_last = 100*(vel_est - velo2)./velo2;
