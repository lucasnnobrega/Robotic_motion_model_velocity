function [x1, y1, teta1] = sample_motion_model_odom(ps, ps_ant, ps1)
    %inputs
    % ut = [xbar(t-1), xbart]
    % onde, xbar(t-1) -> xbar, ybar, tetabar
    %     , xbart -> xbar1, ybar1, tetabar1 -> xbar', ybar', tetabar'

    %x(t-1) -> x, y, teta

    % Output
    % xt = [x1,y1,teta1] que representa o movimento de x(t-1) para xt
    
    % Definindo variáveis
    a1 = 0.02;
    a2 = 0.002;
    a3 = 0.02;
    a4 = 0.2;
    
    xbar1 = ps(1);
    ybar1 = ps(2);
    tetabar1 = ps(3);
    
    xbar = ps_ant(1);
    ybar = ps_ant(2);
    tetabar = ps_ant(3);
    
    x = ps1(1);
    y = ps1(2);
    teta = ps1(3);
    %inicio
    
    rot1 = atan2(ybar1 - ybar, xbar1 - xbar) - tetabar;
    trans = sqrt((xbar - xbar1)^2 + (ybar - ybar1)^2);
    rot2 = tetabar1 - tetabar - rot1;
    rot1_1 = rot1 + amostra(a1*abs(rot1) + a2*abs(trans));
    trans_1 = trans + amostra(a3*abs(trans) + a4*abs(rot1 + rot2));
    rot2_1 = rot2 + amostra(a1*abs(rot1) + a2*abs(trans));
    
    x1 = x + trans_1 * cos(teta + rot1_1);
    y1 = y + trans_1 * sin(teta + rot1_1);
    teta1 = teta + rot1_1 + rot2_1;  
    
    plot([x1], [y1], '.r');
    hold on
end
