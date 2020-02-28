function [x1, y1, teta1] = sample_motion_model_velocity(x, y, teta, v, w, dt, a)

% Estima o valor do estado posteriori com Amostragem
% input: ação de controle u_t e estado anterior x_(t-1)
% u -> v, w
% x_(t-1) -> x, y, teta

% output: estado estimado
% xt -> x1, y1, teta1


%{
a1 = a(1);
a2 = a(2);
a3 = a(3);
a4 = a(4);
a5 = a(5);
a6 = a(6);
%}
a1 = 0.02;
a2 = 0.002;
a3 = 0.02;
a4 = 0.2;
a5 = 0.2;
a6 = 0.2;


v1 = v + amostra(a1*abs(v) + a2*abs(w));
w1 = w + amostra(a3*abs(v) + a4*abs(w));
gama1 = amostra(a5*abs(v) + a6*abs(w));


x1 = x - (v1/w1)*sin(teta) + (v1/w1)*sin(teta + w1*dt);
y1 = y + (v1/w1)*cos(teta) - (v1/w1)*cos(teta + w1*dt);
teta1 = teta + w1*dt + gama1*dt;


plot([x1], [y1], '.r');
hold on

return

end