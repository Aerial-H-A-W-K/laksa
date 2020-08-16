clear all
close all
clc

global PND

% Add the path of the common folder
addpath('../common/')

Flag_Dim = 1;
% Construct the dimensioned parameters
PD = Fun_PD_my_test_KF;
% Modify the dimensioned parameters
PD.Env.Vw      = 15;               % Wind Velocity                     (m/s)
PD.Bridle.delta = 80*pi/180;       % delta                             (rad)
% % Control parameters
PD.Control.Type      = 0;          % 0 -> No control
                                   % 1 -> Periodic laws
                                   % 2 -> Target variables
                                   % 3 -> Target deflection of the aileron
                                   % 4 -> Reel-In
                                   % 5 -> Close-Loop
% Construct the dimensionless parameters
PND = Fun_PND_KF(PD);

% Compute Equilibrium
[u0, Error, Flag, PND]=Equilibrium_GroGen_KF(0,PND);
fprintf("Equilibrium Error: %f\n", Error);

% disp(u0./pi.*180);

[T, X] = ode45('Fun_ODE_Lag_KF',[0:0.0025:1],u0);

[Tout, rR_Edge, rR, vR, R, omegaR, gR, ...
 FA_R, Tension, rQ, R_KE, rK, vK, aK, ...
 euler, omegaK, gK, FA_K, FR_K, FG_K,  MA_K, ...
 MR_K, MG_K, MMC_K, alfa_K, beta_K, rG, vG, ...
 aG, omegaG, gG, FA_G,  FK_G, MA_G, ...
 MK_G, MMC_G, xc_out, xs_target_out, Error_C] = Fun_Post_KF(PD,PND,T(41),X(41,:)',Flag_Dim);

kite_elevation = atan(rK(3,end)./rK(1,end))/pi*180; % Final kite elevation in degrees
kite_pitch = euler(2,end);                          % Final kite pitch in degrees

fprintf("Equilibrium Elevation: %.3f deg, Equilibrium Pitch: %.3f deg\n",kite_elevation,kite_pitch);
            
% Plot_GroGen_KF(X(41,:)',Tout,rQ,rR,rK,rG,R_KE,rR_Edge,PND,Flag_Dim,PD);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%    Close Loop Configuration    %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('Close Loop Results');
disp('------------------');

PND.Control.Type      = 5;  % Closed loop control
% Add the deflection of the control surface to the state vector
u0                    = [u0;PND.Bridle.lb;PND.Bridle.delta+10/180*pi;PND.Bridle.eta];  %[u0; Bridle Length; Bridle Angle Delta; Bridle Angle Eta]

[k_elev,k_pitch]     = calc_elev_pitch(30/180*pi,u0,PD,PND,Flag_Dim);
fprintf("Bridle Delta: %f deg\tElevation: %.3f deg, Pitch: %.3f deg\n",30,k_elev,k_pitch);
% KiteElevation = zeros(size(8:-5:20));
% KitePitch     = zeros(size(8:-5:20));
% index = 1;
% for i=80:-5:20
%     [k_elev,k_pitch]     = calc_elev_pitch(i/180*pi,u0,PD,PND,Flag_Dim);
%     KiteElevation(index) = k_elev;
%     KitePitch(index)     = k_pitch;
%     fprintf("Bridle Delta: %f deg\tElevation: %.3f deg, Pitch: %.3f deg\n",i,k_elev,k_pitch);
%     index = index + 1;
% end


%           [Kite Position, Velocity, Euler, alfa&beta, Tension, Control, Rotor angular velocity, Error_C]
% Flag_Plot = [1            ,     1   ,   1  ,     1    ,    1   ,    1   ,       0               ,   1    ];

% Plot_Results_KF(Tout,rR_Edge, rR, vR, aR, omegaR, gR, FA_R, Tension, rQ, R_KE, ...
%     rK, vK, aK, euler, omegaK, gK, FA_K, FR_K, FG_K,  MA_K, MR_K, MG_K, MMC_K, alfa_K, beta_K,... 
%     rG, vG, aG, omegaG, gG, FA_G,  FK_G, MA_G, MK_G,  MMC_G, xc_out,xs_target_out,Error_C, Flag_Dim, Flag_Plot,PD)



% Unit_Time         = '(s)$';
% Unit_deg          = '(^\circ)$';
% plot(T,euler(2,:),'linewidth',1)
% if PD.Control.Type==5 % Close-Loop
%    plot(T,xs_target_out(2*NR+1,:),'r--','linewidth',LWidth)
% end
% xlabel(['$Time\ ' Unit_Time],'fontsize',12,'interpreter','latex' )
% ylabel(['$\theta\ ' Unit_deg],'fontsize',12,'interpreter','latex')
% grid on
% set(gca,'box','on','fontsize',12)

function [k_elev,k_pitch] = calc_elev_pitch(bridle_delta,u0,PD,PND,Flag_Dim)
    u0(end-2:end) = [PND.Bridle.lb,bridle_delta,PND.Bridle.eta];
    % Check that the right-hand-side vanishes (u0 is equilibrium state)
    xs_amp_p              = Fun_ODE_Lag_KF(0,u0);
    Error                 = max(abs(xs_amp_p));
%     fprintf("Error: %f\n", Error);

    % Compute Jacobian
    Jnum                  = Jacobian('Fun_ODE_Lag_KF',0,u0,PND);

    % Compute eigenvalues
    Val                   = eig(Jnum);
%     fprintf("Eigenvalues:\n");
%     fprintf("%13.8f \t %13.8fi\n",real(Val),imag(Val));

    % Integrate the equations of motion
    [T X] = ode45('Fun_ODE_Lag_KF',[0:0.005:15.00],u0);

    for i=1:1:length(T)
        [Tout(i) rR_Edge(:,:,i) rR(:,:,i) vR(:,:,i) aR(:,:,i) omegaR(:,:,i) gR(:,:,i) FA_R(:,:,i) Tension(:,:,i) rQ(:,i) R_KE(:,:,i) ...
         rK(:,i) vK(:,i) aK(:,i) euler(:,i) omegaK(:,i) gK(:,i) FA_K(:,i) FR_K(:,i) FG_K(:,i)  MA_K(:,i) MR_K(:,i) MG_K(:,i) MMC_K(:,i) alfa_K(i) beta_K(i)... 
         rG(:,:,i) vG(:,:,i) aG(:,:,i) omegaG(:,:,i) gG(:,:,i) FA_G(:,:,i)  FK_G(:,:,i) MA_G(:,:,i) MK_G(:,:,i)  MMC_G(:,:,i) xc_out(:,i) xs_target_out(:,i) Error_C(i)] = Fun_Post_KF(PD,PND,T(i),X(i,:)',Flag_Dim);

        if (i==1 || i==length(T))
            title('');
            pause(0.01);
            Plot_GroGen_KF(X(i,:)',Tout(i),rQ(:,i),rR(:,:,i),rK(:,i),rG(:,:,i),R_KE(:,:,i),rR_Edge(:,:,i),PND,Flag_Dim,PD);
        end
    end

    k_elev = atan(rK(3,end)./rK(1,end))/pi*180; % Final kite elevation in degrees
    k_pitch = euler(2,end);                     % Final kite pitch in degrees
end