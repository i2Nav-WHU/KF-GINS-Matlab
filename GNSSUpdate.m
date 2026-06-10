% -------------------------------------------------------------------------
% KF-GINS-Matlab: An EKF-based GNSS/INS Integrated Navigation System in Matlab
%
% Copyright (C) 2024, i2Nav Group, Wuhan University
%
%  Author : Liqiang Wang
% Contact : wlq@whu.edu.cn
%    Date : 2023.3.3
% -------------------------------------------------------------------------

% GNSS位置更新和速度更新
function kf = GNSSUpdate(navstate, gnssdata, kf, antlever, usegnssvel, thisimu, dt)

    param = Param();

    %% GNSS position update
    % abandon gnss vel outlier 
    gnssposstd = gnssdata(5:7, 1);
    if gnssposstd(1, 1) > 5 || gnssposstd(2, 1) > 5 || gnssposstd(3, 1) > 5
        disp(['WARNING: Abandon gnss position measurement at: ', num2str(gnssdata(1, 1))]);
    else
        % measurement innovation
        DR = diag([navstate.Rm + navstate.pos(3), (navstate.Rn + navstate.pos(3))*cos(navstate.pos(1)), -1]);
        Z = DR*(navstate.pos - gnssdata(2:4, 1))+navstate.cbn*antlever;%N系下的NED
        
        % measurement matrix and noise matrix
        R = diag(power(gnssdata(5:7, 1), 2));%m m m
        H = zeros(3, kf.RANK);
        H(1:3, 1:3) = eye(3);
        H(1:3, 7:9) = skew(navstate.cbn * antlever);
        
        % update covariance and state vector
        K = kf.P * H' / (H * kf.P * H' + R);
        kf.x = kf.x + K*(Z - H*kf.x);
        kf.P=(eye(kf.RANK) - K*H) * kf.P * (eye(kf.RANK) - K*H)' + K * R * K';
    end

    %% GNSS velocity update
    if usegnssvel

        % abandon gnss vel outlier 
        gnssvelstd = gnssdata(11:13, 1);
        if gnssvelstd(1, 1) > 0.5 || gnssvelstd(2, 1) > 0.5 || gnssvelstd(3, 1) > 0.5
            disp(['WARNING: Abandon gnss velocity measurement at: ', num2str(gnssdata(1, 1))]);
        else
            wie_n = [param.WGS84_WIE * cos(navstate.pos(1)); 0; -param.WGS84_WIE * sin(navstate.pos(1))];
            wib_b = thisimu(2:4) / dt;
            vel_ant = navstate.vel - skew(wie_n) * navstate.cbn * antlever - navstate.cbn * (skew(antlever) * wib_b);
            
            % measurement innovation, noise, matrix
            Z = vel_ant - gnssdata(8:10, 1);
            R = diag(power(gnssvelstd, 2));
            H = zeros(3, kf.RANK);
            H(1:3, 4:6) = eye(3);
            H(1:3, 7:9) = -skew(wie_n) * skew(navstate.cbn * antlever) - skew(navstate.cbn * skew(antlever) * wib_b);
            H(1:3, 10:12) = -navstate.cbn * skew(antlever);
            H(1:3, 16:18) = -navstate.cbn * skew(antlever) * diag(wib_b);
    
            % update covariance and state vector
            K = kf.P * H' / (H * kf.P * H' + R);
            kf.x = kf.x + K*(Z - H*kf.x);
            kf.P=(eye(kf.RANK) - K*H) * kf.P * (eye(kf.RANK) - K*H)' + K * R * K';
        end
    end
end