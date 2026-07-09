classdef UKF < handle
    
    properties
        % Parameters ----------------------------------------------
        P = (1e-2)*eye(6); % State uncertainty
        Q = (2)*eye(6); % Process noise

        R = 10*eye(6);% zeros(6); % Measurement error
        %R(1:3,1:3) = (20)*eye(3); % Accelerometer noise
        %R(4:6,4:6) = (10)*eye(3); % Gyro noise

        % Nonlinear model functions and scalings ------------------
        w_off = [373.6,375.4,369.5]'; % Gyro offset [ADC counts]
        w_sca = [0.016,0.016,0.017]'; % Gyro scaling [rad/s per ADC count]

        a_off = [512,501,502]'; % Accelerometer offset [ADC counts]
        a_sca = [-0.0099,-0.0098,0.0097]'; % Accelerometer scaling [gs per ADC count]

        % Map from state vector to gyro measurements
        H1 = @(x) x(5:7,:);
        % Map from state vector to accelerometer measurements
        H2 = @(x) quatrotate(x(1:4,:)',[0,0,1])';
        
        % Scale accelerometer and gyroscope measurements
        scale = @(x,off,sca) (x-off).*sca;

        % Perturb the state vector by a rotation vector and angular velocity
        perturb = @(x,v) [qutil.rotq(x(1:4,:),v(1:3,:)); bsxfun(@plus, x(5:7,:), v(4:6,:))];
        % Update the state vector according to the dynamics
        A = @(x,dt) [qutil.rotq(x(1:4,:), x(5:7,:).*dt) ; x(5:7,:)];
        
        x = [1,0,0,0,0,0,0]'; % state vector
        t = 0; % current time
    end
    
    methods
        function o = UKF(t,x,P)
            if nargin < 3
                t = 0;
                x = [1,0,0,0,0,0,0]';
                P = (1e-2)*eye(6);
            end 
            o.initialize(t,x,P)
        end
        
        function initialize(o,t,x,P)
            if nargin < 4
                P = (1e-2)*eye(6);
            end
            if nargin < 3
                x = [1,0,0,0,0,0,0]';
            end
            if nargin < 2
                t = 0;
            end
            o.x = x;
            o.t = t;
            o.P = P;
        end
        
        function update(o, t, accel, gyro)
            dt = t - o.t;
            o.t = t;
            
            % Sigma Points and Prediction -------------------------
            % W: A set of points with the same covariance as P + Q.
            % X: Perturb the state vector x by the sigma points W.
            % Y: Predict the locations the X points at the next time step.
            % xapri: Mean of the Y points in the next time step.
            % Wp: Distance of the Y point from the new mean (xapri).
            % Papri: Covariance of the prediction (from Wp).
            S = chol(o.P + o.Q); % S
            W = [sqrt(6)*S,zeros(6,1),-sqrt(6)*S]; % script(W)_i
            X = o.perturb(o.x,W); % script(X)_i

            Y = o.A(X,dt); % script(Y)_i
            [~,~,Y_q] = svd(1/size(Y(1:4,:),2)*(Y(1:4,:)*Y(1:4,:)'));
            xapri = [Y_q(:,1)./norm(Y_q(:,1)); mean(X(5:7,:),2)]; %(x\hat)_k^-
            e_q = quatmultiply(Y(1:4,:)',xapri(1:4)'.*[1,-1,-1,-1])';% e_i
            e_q = bsxfun(@rdivide, e_q, vnorm(e_q));
            Wp = [qutil.q2v(e_q); bsxfun(@minus, Y(5:7,:), xapri(5:7))]; % script(W)_i'

            Papri = cov(Wp'); % P_k^-

            % Measurement and Innovation --------------------------
            % Z: Project the state vector into the measurements space (acc & w).
            % zapri: Mean of the predicted estimates (Z).
            % Pzz: Covariance of the predicted estimates (Z).
            % v: Innovation: difference of the measurements from the estimates.
            % Pvv: Total covariance of the measurement.
            Z = [o.H2(Y);o.H1(Y)]; % script(Z)_i
            zapri = mean(Z,2); % z_k^-
            Pzz = cov(Z'); % P_(zz)

            am = o.scale(accel,o.a_off,o.a_sca);
            wm = o.scale(gyro,o.w_off,o.w_sca);
            v = [am; wm] - zapri; % \nu_k
            Pvv = Pzz + o.R; % P_(\nu \nu)

            % Kalman update ---------------------------------------
            % Pxz: Cross-correlation of the prediction and measurement.
            % K: Kalman gain.
            % x: New state vector updated by K.
            % P: New state covariance.
            Pxz = 1/(2*6)*(Wp*bsxfun(@minus,Z,zapri)'); % P_xz

            K = Pxz/Pvv; % K_k
            update = (K*v).*[-1;-1;-1;1;1;1];
            o.x = o.perturb(xapri,update); % (x\hat)_k
            o.P = Papri - K*Pvv*K'; % P_k
        end
    end
    
end
    