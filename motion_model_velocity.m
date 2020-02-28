% Estima o valor do estado posteriori com Amostragem
% input: ação de controle u_t e estado anterior x_(t-1)
% output: estado estimado
function probabilidade = motion_model_velocity(ps, u, ps_ant, a, t)

a1 = a(1);
a2 = a(2);
a3 = a(3);
a4 = a(4);
a5 = a(5);
a6 = a(6);

dt = 2;
%atribuindo variaveis
% Ação de controle

v = u(1);
w = u(2);

%posição atual

x1 = ps(1);  %x'
y1 = ps(2);  %y'
teta1 = ps(3); %teta'

%posição anterior

x = ps_ant(1);
y = ps_ant(2);
teta = ps_ant(3);


media = 1/2*(((x - x1)*cos(teta) + (y -y1)*sin(teta))/ ((y - y1)*cos(teta) - (x - x1)*sin(teta)));

xc = ((x + x1)/2) + media*(y - y1);
yc = ((y + y1)/2) + media*(x - x1);

rc = sqrt((x - xc)^2+(y - yc)^2);

dteta = atan2(y1-yc, x1 - xc) - atan2(y-yc, x-xc);

v1 = (dteta/dt)*rc;
w1 = (dteta/dt);
gama1 = (teta1 - teta)/dt -w1;

probabilidade = prob(v-v1, a1*abs(v) + a2*abs(w))* prob(w - w1, a3*abs(v) + a4*abs(w)) * prob(gama1, a5*abs(v)+a6*abs(w));

figure(1)
plot([x], [y], 'og');
hold on;
plot([x1], [y1], 'ok');
plot([x x1], [y y1]);
axis([0 7 0.5 9]);


for j = 1 : 200
    sample_motion_model_velocity(x, y, teta, v, w, dt, a);
end

end

