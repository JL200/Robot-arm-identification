%%
%Trabalho 1 - Controle Adaptativo
%João Luiz de Castro Pereira - 415049

clear all
close all
clc
load('robot_arm.dat')

%%
k1 = 1;
k2 = 1;

Ts = 2;
sgn = robot_arm;
cont = size(sgn);

ra1 = robot_arm(:,1);
ra2 = robot_arm(:,2);
for i=1:cont(1)
    if (mod(i,Ts)==0)
        rp(k1) = ra1(i);
        k1 = k1+1;
    end
end
for i=1:cont(1)
    if (mod(i,Ts)==0)
        ent(k2) = ra2(i);
        k2 = k2 + 1;
    end
end

%%
Ta = Ts;

yr = rp';
cont = size(rp');
ur = ent;   %sinal de controle

%%
Y=[];
fi=[];
for i=1:cont(1)
    if i<3
        y1 = [0];
        y2 = [0];
        u1 = [0];
        u2 = [0];
    else
        y1 = yr(i-1);
        y2 = yr(i-2);
        u1 = ur(i-1);
        u2 = ur(i-2);
    end
    yr2(i) = yr(i) - u1;
    Y = [Y; yr2(i)];
    fi = [fi; -y1 -y2 u2];
end
teta=inv(fi'*fi)*fi'*Y;              %Matriz de parâmetros estimados

 a1 = teta(1);
 a2 = teta(2);
 %b0 = teta(3);
 b1 = teta(3);

 %%
for t=1:cont(1)
    if t<=3
        yest(t)=0;
    else
        yest(t)=-a1*yest(t-1)-a2*yest(t-2)+b1*ur(t-2)+ur(t-1);
    end
end


Lz=tf([1 b1],[1 a1 a2],Ta);        %Calcula a função de transferência discreta (Z)
Ls=d2c(Lz);                         %Calcula a função de transferência contínua (s)


%%
tp=0:1:(cont(1)-1);
plot(tp,yr);
hold on
plot(tp,yest,'r');
title('Método dos Mínimos Quadrados');
xlabel('Tempo(s)');
ylabel('Amplitude');
legend('Planta Real','Planta Estimada MQ','Location','SouthEast');

er=yr-yest';
erro=abs((er)./yr)*100;       %Cálculo do erro de estimação

%%
figure(2)
plot(tp,erro)
title('Erro de Estimaçao')
xlabel('tempo')
ylabel('Erro (%)')
hold on
grid