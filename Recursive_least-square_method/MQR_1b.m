%%
% MQR
% João Luiz de Castro Pereira - 415049

clear all
close
clc

k = 1;
load('robot_arm.dat');

%%
sgn=robot_arm;

yr = sgn(:,2);
ur = sgn(:,1);
pontos = size(yr');

yr=yr;
cont = length(yr);

%%
for i=1:cont                     %Este 'for' cria um sinal aleatório de média nula e variância 1
    if rand>0.5 eta(i)=1;
    else eta(i)=-1;
    end
    u(i)=eta(i);
end

theta=[0;0;0];                     %gera as estimativas iniciais
e=eta*0.1;                     %gera o erro estimado como uma variável aleatória de média nula
p=1000*eye(length(theta));                %gera a matriz de covariância inicial com valores grandes (1000)

%%
for t=1:2
    y(t)=0;
    erro(t)=0;
    a1(t)=theta(1);
    a2(t)=theta(2);
    b0(t)=theta(3);
   % b1(t)=theta(4);
end

lambda1=0.99;
lambda2=1;

lambda = lambda1/lambda2;

for t=3:cont
    y(t)=yr(t)+e(t);
    fi=[-y(t-1);-y(t-2);ur(t-1)];
    erro(t)=y(t)-fi'*theta;
    k=p*fi/(lambda+fi'*p*fi);
    theta=theta+k*erro(t);
    p=(p-k*fi'*p)/lambda1;
    a1(t)=theta(1);
    a2(t)=theta(2);
    b0(t)=theta(3);
  %  b1(t)=theta(4);
 yx(t)=[b0(t) ]*[ur(t-1) ]'+[-a1(t) -a2(t)]*[y(t-1) y(t-2)]';
end


t=1:cont;
subplot(121),plot(t,a1(t)),title('a1'),xlabel('amostragem');
subplot(122),plot(t,b0(t)),title('b0'),xlabel('amostragem');
%subplot(221),plot(t,a2(t)),title('a2'),xlabel('amostragem');

figure(2);
plot(t,yr,'b');
hold on
plot(t,yx,'r');
title('Comparativo real x estimado (MQR)');
xlabel('Tempo(s)');
ylabel('Amplitude');
legend('Resposta real do processo','Resposta Estimada do processo','Location','SouthEast');