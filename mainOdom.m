clear;
clc;

ps_ant = [2; 2; -1];
u = [0.6 1];
ps = [3; 2; -1];
t = 2;
a = [0.02; 0.05; 0.12; 0.2; 0.2; 4.4];
ps1 = ps;

ps1 = motion_model_odom(ps, ps_ant, ps1, a);

ps = ps1;

ps1 = ps - ps_ant;
ps1 = motion_model_odom([4, 2, -1], ps_ant, ps1 ,a);

ps_ant = ps;
ps1 = ps - ps_ant;
ps = motion_model_odom([5, 3, -1], ps_ant, ps1, a);

ps_ant = ps1;
ps1 = ps - ps_ant;
ps = motion_model_odom([5, 5, 0.2], ps_ant, ps1, a);

ps_ant = ps;
ps1 = ps - ps_ant;

ps = motion_model_odom([4, 6, 2],  ps_ant, ps1, a);

ps_ant = ps;
ps1 = ps - ps_ant;

ps = motion_model_odom([2, 6, pi], ps_ant, ps1, a);
