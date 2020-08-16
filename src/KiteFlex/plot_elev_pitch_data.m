close all;
clear all;
clc;

load('elev_pitch_15ms.mat')
plot(KitePitch,KiteElevation);
title('Kite Pitch vs Elevation Angle');
xlabel('Kite Pitch (deg)');
ylabel('Kite Elevation (deg)');

hold on;

load('elev_pitch_10ms.mat')
plot(KitePitch,KiteElevation);

load('elev_pitch_8ms.mat')
plot(KitePitch,KiteElevation);

load('elev_pitch_7ms.mat')
plot(KitePitch(1:end-3),KiteElevation(1:end-3));

legend('v_w = 15 m/s','v_w = 10 m/s','v_w = 8 m/s','v_w = 7 m/s');