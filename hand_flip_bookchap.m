clc 
clear all
close all
features = [];

for file_no = 1:10
    file_no2 = int2str(file_no);
    disp(file_no);
    label = load('label_10subs.mat'); %1 is healthy 0 is PD
    analysis_side = 1; %Left:1, Right: 0
    vblNames = {'Timestamp','Accx','Accy','Accz','Gyrx','Gyry','Gyrz'};


    left_fname = dir("handflip_case_study/"+file_no2+"_left*").folder+"/"+dir("handflip_case_study/"+file_no+"_left*").name;
    left_flip = readtable(left_fname);     

    right_fname = dir("handflip_case_study/"+file_no2+"_right*").folder+"/"+dir("handflip_case_study/"+file_no+"_right*").name;
    right_flip = readtable(right_fname);
    % % 
    % Timestamp = left_flip.Timestamp;
    if analysis_side==1
        Timestamp = left_flip.Timestamp;
        Accx = left_flip.Acc_x_;
        Accy = left_flip.Acc_y_;
        Accz = left_flip.Acc_z_;
        Gyrx = left_flip.Gyr_x_ ;
        Gyry = left_flip.Gyr_y_ ;
        Gyrz = left_flip.Gyr_z_ ;
    end

    if analysis_side==0

        Timestamp = right_flip.Timestamp;
        Accx = right_flip.Acc_x_;
        Accy = right_flip.Acc_y_;
        Accz = right_flip.Acc_z_;
        Gyrx = right_flip.Gyr_x_ ;
        Gyry = right_flip.Gyr_y_ ;
        Gyrz = right_flip.Gyr_z_ ;
    end

    %        
    %IMU Resolution Parameters
    gRes = 8.75*1000; %dps/LSB
    aRes = 0.732*1000; %g/LSB

    %Initalize inputs for the Extended Kalman Filter
    Gyro_pitch = 0; %Initialize value
    Accel_pitch = 0; %Initial Estimate
    Predicted_pitch = 0; %Initial Estimate
    Gyro_roll = 0; %Initialize value
    Accel_roll = 0; %Initialize value
    Predicted_roll = 0; %Output of Kalman filter

    Q = 0.038;%0.0338;%0.00143; % Process noise - Prediction Estimate Initial Guess
    R = 0.023;%3.8915;%3.8915;%0.029; % Measurement noise - Prediction Estimate Initial Guess
    P00 = 0.0551;%3.6281;%3.6281;%3.3967; % Prediction Estimate Initial Guess 0.0551
    P11 = 0.0341;%6.221;%6.221;%3.3967; % Prediction Estimate Initial Guess 0.0341
    P01 = 0.324;%-0.1;%0.001;%-0.1737;%1.843; % Prediction Estimate Initial Guess 0.0324

    %previously set values found from calibration on 11-1-19
    %Gloves
    Gyro_cal_x = -399.1456;%-367.2550;%-362.97
    Gyro_cal_y = 174.8742;%111.3600;%108.51
    Gyro_cal_z = 388.2727;%410.6150;%412.5760
    Accel_cal_x = -109.3904;%-97.3;%52.6250
    Accel_cal_y = -61.0129;%-45.97;%30.2530
    Accel_cal_z = 616.4460;%-2095.6;%613;%-2095.6

    %Right shoe
    % Gyro_cal_x = -4125.2;
    % Gyro_cal_y = 1401.6;
    % Gyro_cal_z = 3750.1;
    % Accel_cal_x = 12400;%13132;
    % Accel_cal_y = -2090.1;
    % Accel_cal_z = -588.35;

    %Left shoe
    % Gyro_cal_x = -1521.9;
    % Gyro_cal_y = 2500.7;
    % Gyro_cal_z = -530.45;
    % Accel_cal_x = 12171;%13132;
    % Accel_cal_y = 3321.9;
    % Accel_cal_z = 2047.8;


    %initialize outputs in original
    Gyro_output = [Gyrx(1),Gyry(1),Gyrz(1)];
    Accel_output= [Accx(1),Accy(1),Accz(1)];

    Fs = 1/0.008;
    fs=128;
    T = Timestamp - min(Timestamp);
    t = 0:8:T(end)';
    time = t;%fixed Timestamp
    dt = 0.008; %milli seconds

    %%
    %fix type cast issue with some data
    % Accx = fix_from_uint8(Accx);


    %%
    %interpolation section

    Accx = interp1(T,Accx,t);
    Accy = interp1(T,Accy,t);
    Accz = interp1(T,Accz,t);
    Gyrx = interp1(T,Gyrx,t);
    Gyry = interp1(T,Gyry,t);
    Gyrz = interp1(T,Gyrz,t);

    %%
    %calibration of sensors (leave sensors motionless and level)
    % Gyro_cal_x_sample = 0;
    % Gyro_cal_y_sample = 0;
    % Gyro_cal_z_sample = 0;
    % Accel_cal_x_sample = 0;
    % Accel_cal_y_sample = 0;
    % Accel_cal_z_sample = 0;
    % 
    % for (i = 1:1000)
    %     %getGyroscopeReadings(Gyro_output);
    %     %getAccelerometerReadings(Accel_output);
    %     Gyro_cal_x_sample = Gyro_cal_x_sample + Gyrx(i);%Gyro_output(1);
    %     Gyro_cal_y_sample = Gyro_cal_y_sample + Gyry(i);%Gyro_output(2);
    %     Gyro_cal_z_sample = Gyro_cal_z_sample + Gyrz(i);%Gyro_output(3);
    %     Accel_cal_x_sample = Accel_cal_x_sample + Accx(i);%Accel_output(1);
    %     Accel_cal_y_sample = Accel_cal_y_sample + Accy(i);%Accel_output(2);
    %     Accel_cal_z_sample = Accel_cal_z_sample + Accz(i);%Accel_output(3);
    % end
    % Gyro_cal_x = Gyro_cal_x_sample / 100;
    % Gyro_cal_y = Gyro_cal_y_sample / 100;
    % Gyro_cal_z = Gyro_cal_z_sample / 100;
    % %hands acc
    % Accel_cal_x = Accel_cal_x_sample / 100;
    % Accel_cal_y = Accel_cal_y_sample / 100;
    % Accel_cal_z = (Accel_cal_z_sample / 100) - (aRes*1000); % Raw Accel output in z-direction is in 256 LSB/g due to gravity and must be off-set by approximately 256
    % 
    % %right foot acc
    % % Accel_cal_x = Accel_cal_x_sample / 100 - (aRes*1000); 
    % % Accel_cal_y = Accel_cal_y_sample / 100;
    % % Accel_cal_z = (Accel_cal_z_sample / 100);% Raw Accel output in z-direction is in 256 LSB/g due to gravity and must be off-set by approximately 256
    % 
    % %left foot acc
    % % Accel_cal_x = Accel_cal_x_sample / 100;
    % % Accel_cal_y = Accel_cal_y_sample / 100;
    % % Accel_cal_z = (Accel_cal_z_sample / 100) - (aRes*1000); % Raw Accel output in z-direction is in 256 LSB/g due to gravity and must be off-set by approximately 256
    % 
    % %end calibration
    % 

    %%
    %Filter - extended kalman


    for i=1:length(time)-1

        %PITCH - Update next pitch value
        %Accel_pitch = atan2((Accel_output(1) - Accel_cal_y) / 256, (Accel_output(2) - Accel_cal_z) / 256) * 180 / pi;
        Accel_pitch = atan2((Accy(i) - Accel_cal_y) / (aRes), (Accz(i) - Accel_cal_z) / (aRes)) * 180 / pi;
        %Gyro_pitch = Gyro_pitch + ((Gyro_output(0) - Gyro_cal_x) / 14.375) * dt;
        Gyro_pitch = Gyro_pitch + ((Gyrx(i) - Gyro_cal_x) / (gRes)) * dt;
        %Predicted_pitch = Predicted_pitch + ((Gyro_output(0) - Gyro_cal_x) / 14.375) * dt; % Time Update step 1
        Predicted_pitch = Predicted_pitch + ((Gyrx(i) - Gyro_cal_x) / (gRes))*dt;%14.375) * dt; % Time Update step 1

        if(Gyro_pitch<-180)
            Gyro_pitch = Gyro_pitch + 360; % Keep within range of 0-180 deg to match Accelerometer output
        end
        if(Gyro_pitch>=180)
            Gyro_pitch = Gyro_pitch - 360;
        end
        Comp_pitch = Gyro_pitch;

        %ROLL - update next Roll value
        %Accel_roll = atan2((Accel_output(0) - Accel_cal_x) / 256, (Accel_output(2) - Accel_cal_z) / 256) * 180 / PI;
        Accel_roll = atan2((Accx(i) - Accel_cal_x) / (aRes), (Accz(i) - Accel_cal_z) / (aRes)) * 180 / pi;
        %Gyro_roll = Gyro_roll - ((Gyro_output(1) - Gyro_cal_y) / 14.375) * dt;
        Gyro_roll = Gyro_roll - ((Gyry(i) - Gyro_cal_y) / (gRes)) * dt;
        %Predicted_roll = Predicted_roll - ((Gyro_output(1) - Gyro_cal_y) / 14.375) * dt; % Time Update step 1
        Predicted_roll = Predicted_roll - ((Gyry(i) - Gyro_cal_y) / (gRes)) * dt; % Time Update step 1

        if(Gyro_roll<-180)
            Gyro_roll = Gyro_roll + 360; % Keep within range of 0-180 deg
        end
        if(Gyro_roll>=180)
            Gyro_roll = Gyro_roll - 360;
        end
        Comp_roll = Gyro_roll;

        alpha = 0.9; %adjust for gryo wieghting in comp filter
        Comp_pitch = alpha*(Comp_pitch+Comp_pitch*dt) + (1.0 - alpha)*Accel_pitch; % Complimentary filter
        Comp_roll = alpha*(Comp_roll+Comp_roll*dt) + (1.0 - alpha)*Accel_roll; % Complimentary filter

        P00 = P00 + dt * (2 * P01 + dt * P11); % Projected error covariance terms from derivation result: Time Update step 2
        P01 = P01 + dt * P11; % Projected error covariance terms from derivation result: Time Update step 2
        P00 = P00 + dt * Q; % Projected error covariance terms from derivation result: Time Update step 2
        P11 = P11 + dt * Q; % Projected error covariance terms from derivation result: Time Update step 2
        Kk0 = P00 / (P00 + R); % Measurement Update step 1
        Kk1 = P01 / (P01 + R); % Measurement Update step 1
        Predicted_pitch = Predicted_pitch + (Accel_pitch - Predicted_pitch) * Kk0; % Measurement Update step 2
        Predicted_roll = Predicted_roll + (Accel_roll - Predicted_roll) * Kk0; % Measurement Update step 2
        P00 = P00*(1 - Kk0); % Measurement Update step 3
        P01 = P01*(1 - Kk1); % Measurement Update step 3
        P11 = P11 - Kk1 * P01; % Measurement Update step 3


    %     alpha = 0.5; %adjust for gryo wieghting in comp filter
    %     Comp_pitch = alpha*(Comp_pitch+Comp_pitch*dt) + (1.0 - alpha)*Accel_pitch; % Complimentary filter
    %     Comp_roll = alpha*(Comp_roll+Comp_roll*dt) + (1.0 - alpha)*Accel_roll; % Complimentary filter

        angle_z(i) = Gyrz(i);%Gyro_output(2);
        time(i)=time(i+1)-time(i);
        time(i) = (dt*1000)-time(i);
        t2(i) = time(i);

        gyro_pitch2(i) = Gyro_pitch;
        accel_pitch2(i) = Accel_pitch;
        comp_pitch2(i) = Comp_pitch;
        predicted_pitch2(i) = Predicted_pitch;

        gyro_roll2(i) = Gyro_roll;
        accel_roll2(i) = Accel_roll;
        comp_roll2(i) = Comp_roll;
        predicted_roll2(i) = Predicted_roll;
    end






    %%
    %init analysis

    Roll = predicted_roll2;
    Pitch = predicted_pitch2;
    Yaw = angle_z;
    min_peak_height = 0.004;
    win_r = length(Roll);
    win_p = length(Pitch);

    % Find fundamental frequency, threshold for roll,pitch and peaks and valleys
    X_valp = abs(fft((max(Pitch) - Pitch)).*hanning(win_p))/win_p;
    X_peakp = abs((fft(Pitch)).*hanning(win_p))/win_p;
    X_valr = abs(fft((max(Roll) - Roll).*hanning(win_r)))/win_r;
    X_peakr = abs((fft(Roll)).*hanning(win_r))/win_r;

    NUP_p = ceil((win_p+1)/2);    % calculate the number of unique points
    NUP = ceil((win_r+1)/2);    % calculate the number of unique points
    X_valp = X_valp(1:NUP_p);
    X_peakp = X_peakp(1:NUP_p);
    X_valr = X_valr(1:NUP);           % FFT is symmetric, throw away second half
    X_peakr = X_peakr(1:NUP);  
    if rem(win_r, 2)            % odd nfft excludes Nyquist point
        X_valr(2:end) = X_valr(2:end)*2;
    else                    % even nfft includes Nyquist point
        X_valr(2:end-1) = X_valr(2:end-1)*2;
    end

    if rem(win_r, 2)            % odd nfft excludes Nyquist point
        X_peakr(2:end) = X_peakr(2:end)*2;
    else                    % even nfft includes Nyquist point
        X_peakr(2:end-1) = X_peakr(2:end-1)*2;
    end

    if rem(win_p, 2)            % odd nfft excludes Nyquist point
        X_peakp(2:end) = X_peakp(2:end)*2;
    else                    % even nfft includes Nyquist point
        X_peakp(2:end-1) = X_peakp(2:end-1)*2;
    end

    if rem(win_p, 2)            % odd nfft excludes Nyquist point
        X_valp(2:end) = X_valp(2:end)*2;
    else                    % even nfft includes Nyquist point
        X_valp(2:end-1) = X_valp(2:end-1)*2;
    end

    f_r = (0:NUP-1)*fs/win_r;     % frequency vector
    X_valr = 20*log10(X_valr);        % spectrum magnitude
    [Y,I_r] = max(X_valr(5:end));

    X_peakr = 20*log10(X_peakr);  
    X_valr = 20*log10(X_valr);  

    fundamentalfreq_r = f_r(I_r);
    stepsize_r = fs/fundamentalfreq_r;

    f_p = (0:NUP_p-1)*fs/win_p;     % frequency vector
    X_valp = 20*log10(X_valp);        % spectrum magnitude
    [Y,I_p] = max(X_valp(5:end));

    X_peakp = 20*log10(X_peakp);  

    fundamentalfreq_p = f_p(I_p);
    stepsize_p = fs/fundamentalfreq_p;

    % Sigma and Mu estimation
    [u_val_r, s_val_r] = normfit(max(Roll)-Roll);

    [u_peak_r, s_peak_r] = normfit(Roll);

    thres_val_r = u_val_r;

    thres_peak_r = u_peak_r;

    [u_val_p, s_val_p] = normfit(max(Pitch)-Pitch);

    [u_peak_p, s_peak_p] = normfit(Pitch);

    thres_val_p = u_val_p;

    thres_peak_p = u_peak_p;
    if stepsize_p > 500
        stepsize_p = 50;
    end
    if stepsize_r > 500
        stepsize_r = 50;
    end
        
    [VALr, VALindex_r] = findpeaks(max(Roll) - Roll,'MinPeakHeight',sqrt(thres_val_r),'MinPeakDistance',ceil(stepsize_r/2));
    [VALp, VALindex_p] = findpeaks(max(Pitch) - Pitch,'MinPeakHeight',sqrt(thres_val_p),'MinPeakDistance',ceil(stepsize_p/2));

    VALp = Pitch(VALindex_p);
    VALr = Roll(VALindex_r);
     

    [PKSr, PKSindex_r] = findpeaks(Roll,'MinPeakHeight',sqrt(abs(thres_peak_r)),'MinPeakDistance',ceil(stepsize_r/2));
    [PKSp, PKSindex_p] = findpeaks(Pitch,'MinPeakHeight',sqrt(abs(thres_peak_p)),'MinPeakDistance',ceil(stepsize_p/2));
    clear i;
%     plot(Roll,'LineWidth',2);
%     hold on;
%     plot(VALindex_r, VALr,'x','MarkerSize',15,'LineWidth',2);
%     hold on;
%     plot(PKSindex_r,PKSr,'o','MarkerSize',15,'LineWidth',2);
% 
%     figure;
%     plot(Pitch,'LineWidth',2);
%     hold on;
%     plot(VALindex_p, VALp,'x','MarkerSize',15,'LineWidth',2);
%     hold on;
%     plot(PKSindex_p,PKSp,'o','MarkerSize',15,'LineWidth',2);

    %Feature extraction

    meanPitch = mean(abs(Pitch));
    meanRoll = mean(abs(Roll));
    dPitch = Pitch(2:end) - Pitch(1:end-1);
    dRoll = Roll(2:end)-Roll(1:end-1);
    dRollMean = mean(abs(dRoll));
    dPitchMean = mean(abs(dPitch));
    numPeaksRoll = length(PKSr);
    numPeaksPitch = length(PKSp);
    peakFreqRoll = length(PKSr)*fs;
    peakFreqPitch = length(PKSp)*fs;
    peakvarPitch = var(PKSp);
    peakvarRoll = var(PKSr);
    rmsPitch = sqrt(sum(Pitch.^2));
    rmsRoll = sqrt(sum(Roll.^2));
    rangeroll = max(Roll) - min(Roll);
    rangepitch = max(Pitch)-min(Pitch);

    meanAmpl_pitch = meanPitch;
    meanAmpl_roll = meanRoll;
    meandPitch = dPitchMean;
    meandRoll = dRollMean;
    numPeaks_roll = numPeaksRoll;
    numPeaks_pitch = numPeaksPitch;
    PeakFreqPitch = peakFreqPitch;
    PeakFreqRoll = peakFreqRoll;
    PeakVarPitch = peakvarPitch;
    PeakVarRoll = peakvarRoll;
    RMSpitch = rmsPitch;
    RMSroll = rmsRoll;
    rangePitch = rangepitch;
    rangeRoll = rangeroll;


    
    features_added = [meanAmpl_pitch,meanAmpl_roll meandPitch meandRoll...
                    numPeaks_pitch numPeaks_roll PeakFreqPitch PeakFreqRoll ...
                    RMSpitch RMSroll PeakVarPitch...
                    PeakVarRoll rangePitch rangeRoll label.Label(file_no) file_no];
%     feature_map = readtable(right_fname);
    features = [features; features_added];

end
vNames = {'MeanAmplPitch','MeanAmplRoll','MeanFirstDiffPitch',...
               'MeanFirstDiffRoll','NumofPeaksPitch','NumofPeaksRoll',...
               'PeakFreqPitch','PeakFreqRoll','RMSpitch',...
               'RMSroll','PeakVarPitch','PeakVarRoll',...
               'RangePitch','RangeRoll', 'Label', 'subject_no'};
features2 = features;
% save("features_handflip_labelled_test",'features2');

features =   array2table(features); 
features.Properties.VariableNames = vNames;
if analysis_side==1
    save("features_handflip_left_labelled_10subs",'features');

else
save("features_handflip_right_labelled_10subs",'features');
end
% % if isfile('features_handflip_labelled.mat')
% %     save("features_handflip_labelled",'feature_map','-append');
% % else
% %     save("features_handflip_labelled",'feature_map');
% % end

% FeaturesPD = array2table(features_added);
        
% FeaturesPD.Properties.VariableNames = vNames;