% -------------------------------------------------------------------------
% KF-GINS-Matlab: An EKF-based GNSS/INS Integrated Navigation System in Matlab
%
% Copyright (C) 2024, i2Nav Group, Wuhan University
%
%  Author : Liqiang Wang
% Contact : wlq@whu.edu.cn
%    Date : 2023.3.3
% -------------------------------------------------------------------------

function kf = InsPropagate(navstate, thisimu, dt, kf, corrtime)
    
    param = Param();
    
    %% copy navstate parameters
    pos = navstate.pos;
    vel = navstate.vel;
    cbn = navstate.cbn;
    rm = navstate.Rm;
    rn = navstate.Rn;
    gravity = navstate.gravity;

    % imu data, angular velocity and acceleration
    omega = thisimu(2:4, 1) / dt;
    accel = thisimu(5:7, 1) / dt;

    % geometric parameters
    rmh = rm + pos(3);
    rnh = rn + pos(3);
    wie_n = [param.WGS84_WIE * cos(pos(1)); 0; -param.WGS84_WIE * sin(pos(1))];
    wen_n = [vel(2) / (rn + pos(3)); -vel(1) / (rm + pos(3)); -vel(2) * tan(pos(1)) / (rn + pos(3))];


    %% state transition matrix
    F = zeros(kf.RANK, kf.RANK);
    PHI = eye(kf.RANK, kf.RANK);

    % position error
    Frr = zeros(3, 3);
    Frr(1, 1) = -vel(3) / rmh;
    Frr(1, 3) = vel(1) / rmh;
    Frr(2, 1) = vel(2) * tan(pos(1)) / rnh;
    Frr(2, 2) = -(vel(3) + vel(1) * tan(pos(1))) / rnh;
    Frr(2, 3) = vel(2) / rnh;
    F(1:3, 1:3) = Frr;
    F(1:3, 4:6) = eye(3);

    % velocity error
    Fvr = zeros(3, 3);
    Fvr(1, 1) = -2 * vel(2) * param.WGS84_WIE * cos(pos(1)) / rmh - power(vel(2), 2) / rmh / rnh / power(cos(pos(1)), 2);
    Fvr(1, 3) = vel(1) * vel(3) / rmh / rmh - power(vel(2), 2) * tan(pos(1)) / rnh / rnh;
    Fvr(2, 1) = 2 * param.WGS84_WIE * (vel(1) * cos(pos(1)) - vel(3) * sin(pos(1))) / rmh + vel(1) * vel(2) / rmh / rnh / power(cos(pos(1)), 2);
    Fvr(2, 3) = (vel(2) * vel(3) + vel(1) * vel(2) * tan(pos(1))) / rnh / rnh;
    Fvr(3, 1) = 2 * param.WGS84_WIE * vel(2) * sin(pos(1)) / rmh;
    Fvr(3, 3) = -power(vel(2), 2) / rnh / rnh - power(vel(1), 2) / rmh / rmh + 2 * gravity / (sqrt(rm * rn) + pos(3));
    F(4:6, 1:3) = Fvr;
    Fvv = zeros(3, 3);
    Fvv(1, 1) = vel(3) / rmh;
    Fvv(1, 2) = -2 * (param.WGS84_WIE * sin(pos(1)) + vel(2) * tan(pos(1)) / rnh);
    Fvv(1, 3) = vel(1) / rmh;
    Fvv(2, 1) = 2 * param.WGS84_WIE * sin(pos(1)) + vel(2) * tan(pos(1)) / rnh;
    Fvv(2, 2) = (vel(3) + vel(1) * tan(pos(1))) / rnh;
    Fvv(2, 3) = 2 * param.WGS84_WIE * cos(pos(1)) + vel(2) / rnh;
    Fvv(3, 1) = -2 * vel(1) / rmh;
    Fvv(3, 2) = -2 * (param.WGS84_WIE * cos(pos(1)) + vel(2) / rnh);
    F(4:6, 4:6) = Fvv;
    F(4:6, 7:9) = skew(cbn * accel);
    F(4:6, 13:15) = cbn;
    F(4:6, 19:21) = cbn * diag(accel);


    %% attitude error
    Fphir = zeros(3, 3);
    Fphir(1, 1) = -param.WGS84_WIE * sin(pos(1)) / rmh;
    Fphir(1, 3) = vel(2) / rnh / rnh;
    Fphir(2, 3) = -vel(1) / rmh / rmh;
    Fphir(3, 1) = -param.WGS84_WIE * cos(pos(1)) / rmh - vel(2) / rmh / rnh / power(cos(pos(1)), 2);
    Fphir(3, 3) = -vel(2) * tan(pos(1)) / rnh / rnh;
    F(7:9, 1:3) = Fphir;
    Fphiv = zeros(3, 3);
    Fphiv(1, 2) = 1 / rnh;
    Fphiv(2, 1) = -1 / rmh;
    Fphiv(3, 2) = -tan(pos(1)) / rnh;
    F(7:9, 4:6) = Fphiv;
    F(7:9, 7:9) = -skew(wie_n + wen_n);
    F(7:9, 10:12) = -cbn;
    F(7:9, 16:18) = -cbn * diag(omega);

    % IMU bias error and scale error, first-order Gauss-Markov process
    F(10:12, 10:12) = -1 / corrtime * eye(3); 
    F(13:15, 13:15) = -1 / corrtime * eye(3); 
    F(16:18, 16:18) = -1 / corrtime * eye(3); 
    F(19:21, 19:21) = -1 / corrtime * eye(3);

    % discrete state transition matrix
    PHI = PHI + F * dt;

    
    %% noise drive matrix
    G = zeros(kf.RANK, kf.NOISE_RANK);
    G(4:6, 1:3) = cbn;
    G(7:9, 4:6) = cbn;
    G(10:12, 7:9) = eye(3);
    G(13:15, 10:12) = eye(3);
    G(16:18, 13:15) = eye(3);
    G(19:21, 16:18) = eye(3);
    % discrete state noise
    Qd = G * kf.Qc * G' * dt;
    Qd = (PHI * Qd  * PHI' + Qd) / 2;


    %% covariance prpagation
    kf.P = PHI * kf.P * PHI' + Qd;
end