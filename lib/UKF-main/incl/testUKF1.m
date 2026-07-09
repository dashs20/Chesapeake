% Test new UKF objects on the project 2 datasets to make sure everything
% still works.

% Dataset
addpath ESE650' P2'/imu
addpath ESE650' P2'/vicon/

load('imuRaw2.mat');
vic = load('viconRot2.mat');

w_off = [373.6,375.4,369.5]'; % Gyro offset [ADC counts]
w_sca = [0.016,0.016,0.017]'; % Gyro scaling [rad/s per ADC count]

a_off = [512,501,502]'; % Accelerometer offset [ADC counts]
a_sca = [-0.0099,-0.0098,0.0097]'; % Accelerometer scaling [gs per ADC count]

% for test 9
%vals(4:6,713:990) = mean(w_off);

% Initialize orientation tracking UKF
clear ori
ori = UKF_SC(ts(1));

% Initialize plotting
figure(1)
h1 = hplot.rot(eye(3),[]);
hold on
h2 = hplot.rot(eye(3),[],2);
hold off

% Run UKF
for ii = 2:length(ts)
    % Measurements
    am = a_sca.*(vals(1:3,ii)-a_off);
    wm = w_sca.*(vals([5;6;4],ii)-w_off);
    
    % UKF update
    ori.update(ts(ii),am,wm);
    
    % Update plotting
    if ~mod(ii,3)
        h1 = hplot.rot(qutil.q2r(ori.x),h1);
        [terr,tind] = min(abs(ts(ii)-vic.ts));
        h2 = hplot.rot(vic.rots(:,:,tind),h2);
        title(ts(ii)-ts(1))
        drawnow
    end
end