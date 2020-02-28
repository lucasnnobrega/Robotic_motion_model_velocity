function [x1, y1, teta1] = motion_model_odom(ps, ps_ant,ps1, a)
    %inputs
    % ut = [xbar(t-1), xbart]
    % onde, xbar(t-1) -> xbar, ybar, tetabar
    %     , xbart -> xbar1, ybar1, tetabar1 -> xbar', ybar', tetabar'

    %x(t-1) -> x, y, teta

    % Output
    % xt = [x1,y1,teta1] que representa o movimento de x(t-1) para xt
    
    % motion_model_odom(ps, u, ps_ant, a, t)
    % ps  -> xbart
    % ps_ant -> posição anterior -> xbar(t-1)
    
    %Definindo as variáveis
    xbar1 = ps(1);
    ybar1 = ps(2);
    tetabar1 = ps(3);
    
    xbar = ps_ant(1);
    ybar = ps_ant(2);
    tetabar = ps_ant(3);
    
    x = ps1(1);
    y = ps1(2);
    teta = ps1(3);
    
    a1 = a(1);
    a2 = a(2);
    a3 = a(3);
    a4 = a(4);

    %Inicio

    rot1 = atan2(ybar1 - ybar, xbar1 - xbar) - tetabar;
    trans = sqrt((xbar - xbar1)^2 + (ybar - ybar1)^2);
    rot2 = tetabar1 - tetabar - rot1;
    
    rot1_1 = rot1 + prob(rot1, a1*rot1 + a2*trans);
    trans_1 = trans + prob(trans, a3*trans + a4*(rot1 + rot2));
    rot2_1 = rot2 + prob(rot2, a1*rot2 + a2*trans);
    
    x1 = x + trans_1 * cos(teta + rot1_1);
    y1 = y + trans_1 * sin(teta + rot1_1);
    teta1 = teta + rot1_1 + rot2_1;  
    
    figure(1)
    plot([x], [y], 'og');
    hold on;
    plot([x1], [y1], 'ok');
    plot([x x1], [y y1]);
    %axis([0 7 0.5 9]);


    for j = 1 : 200
        sample_motion_model_odom(ps, ps_ant, ps1);
    end
    end

