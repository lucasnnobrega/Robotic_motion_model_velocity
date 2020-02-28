clear;
clc;

ps_ant = [2; 2; -1];
u = [0.6 1];
ps = [3; 2; -1];
t = 2;
a = [0.02; 0.05; 0.12; 0.2; 0.2; 4.4];

[prob_p_ant] = motion_model_velocity(ps, u, ps_ant, a, t);

u = [0.6 1];
[prob_p_atual] = motion_model_velocity([4, 2, -1], u, ps_ant, a, t);

ps_ant = [4; 2; -0.2];
u = [0.84 0.99];
[prob_p_ant] = motion_model_velocity([5, 3, -1], u, ps_ant, a, t);

ps_ant = [5, 3, 0.6];
u = [1.2 1];
[prob_p_ant] = motion_model_velocity([5, 5, 0.2], u, ps_ant, a,t);

ps_ant = [5, 5, 1];
u = [1 1.5];
[prob_p_ant] = motion_model_velocity([4, 6, 2], u, ps_ant, a,t);

ps_ant = [4, 6, 2];
u = [1.25 1.1];
[prob_p_ant] = motion_model_velocity([2, 6, pi], u, ps_ant, a,t);
