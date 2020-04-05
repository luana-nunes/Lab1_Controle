# -*- coding: utf-8 -*-
"""
Created on Tue Mar 31 14:59:12 2020

@author: Luana Nunes
"""

import numpy as np
import control as co_general
import matplotlib.pyplot as plt
import control.matlab as co


plt.close("all")
#%
#% Controle de posicao angular do motor eletrico CC
#% Testes com controlador PID
#%
#% Definicao dos valores dos parametros do sistema
#%
#% Constante do tacometro
Kpot = 0.318;
#% Parametros da funcao de transferencia Gw(s)
K1 = 100;
Km = 2.083;
Kg = 0.1;
a  = 100;
am = 1.71;
#%
#% Funcao de transferencia da posicao angular do sistema
s = co.tf('s');
Gp = (K1*Km*Kg)/(s*(s+a)*(s+am))
#%
#% Controlador PID H(s)
#%                     1        Td*s
#%   H(s) = Kp*( 1 + ----- + ------------ )    
#%                    Ti*s     (Td/N)*s+1
#% 
#%   A parte derivativa possui um Filtro de 1a. ordem
#%   com polo em s = (-N/Td)
#%
#% Para controlador P:    Ti=Inf, Td=0, N=Inf, Kp > 0
#% Para controlador PI:   Td=0, N=Inf, Kp, Ti > 0
#% Para controlador PD:   Ti=Inf, Kp, Ti, N > 0
#% Para controlador PID:  Kp, Ti, Td, N >0
#%
#% Qto Maior Ti menor o seu efeito 
#% Qto maior N menor o efeito do Filtro de 1a. Ordem
#%
#% Definicao dos controladores
#% Escolha dos parametros
#%
#% Definicao dos controladores
#% Escolha dos parametros
#%
#% Controlador 1
Kp1 = 5
Ti1 = 0.1
Td1 = 0
N1  = float('inf')
#Controlador P
if Ti1 == float('inf') and Td1 == 0 and N1 == float('inf'):
    H1  = Kp1
#Controlador PI
elif Td1 == 0 and N1 == float('inf'):
    H1 = Kp1*(1+(1/(Ti1*s)))
#Controlador PD
elif Ti1 == float('inf'):
    H1 = Kp1*(1+(Td1*s/((Td1/N1)*(s+1))))
#Controlador PID
else:
    H1 = Kp1*(1+(1/(Ti1*s))+(Td1*s/((Td1/N1)*(s+1))))
Hp1 = Kpot*H1

#% Controlador 2
Kp2 = 5
Ti2 = 0.2
Td2 = 0
N2  = float('inf')
#Controlador P
if Ti2 == float('inf') and Td2 == 0 and N2 == float('inf'):
    H2  = Kp2
#Controlador PI
elif Td2 == 0 and N2 == float('inf'):
    H2 = Kp2*(1+(1/(Ti2*s)))
#Controlador PD
elif Ti2 == float('inf'):
    H2 = Kp2*(1+(Td2*s/((Td2/N2)*(s+1))))
#Controlador PID
else:
    H2 = Kp2*(1+(1/(Ti2*s))+(Td2*s/((Td2/N2)*(s+1))))
Hp2 = Kpot*H2

#% Controlador 3
Kp3 = 5
Ti3 = 0.5
Td3 = 0
N3  = float('inf')
#Controlador P
if Ti3 == float('inf') and Td3 == 0 and N3 == float('inf'):
    H3 = Kp3
#Controlador PI
elif Td3 == 0 and N3 == float('inf'):
    H3 = Kp3*(1+(1/(Ti3*s)))
#Controlador PD
elif Ti3 == float('inf'):
    H3 = Kp3*(1+(Td3*s/((Td3/N3)*(s+1))))
#Controlador PID
else:
    H3 = Kp3*(1+(1/(Ti3*s))+(Td3*s/((Td3/N3)*(s+1))))
Hp3 = Kpot*H3

#%
#% Definicao da malha aberta
#% Sao definidos 3 sistemas distintos
#%
GHp1 = Hp1*Gp
GHp2 = Hp2*Gp
GHp3 = Hp3*Gp
#%
#% Polos e zeros de maplha aberta
#%
print('Polos e zeros - GHp1')
print(co.pole(GHp1))
print(co.zero(GHp1))
print('\nPolos e zeros - GHp2')
print(co.pole(GHp2))
print(co.zero(GHp2))
print('\nPolos e zeros - GHp3')
print(co.pole(GHp3))
print(co.zero(GHp3))
#%
#% Plot com os polos e zeros de malha aberta
#%

co.pzmap(GHp1, title = "GHp1")
co.pzmap(GHp2,title = "GHp2")
co.pzmap(GHp3, title = "GHp3")

#figure(1);
#subplot(1,3,1);
#pzplot(GHp1);
#title('m. aberta GHp1');
#axis equal;
#subplot(1,3,2);
#pzplot(GHp2);
#title('m. aberta GHp2');
#axis equal;
#subplot(1,3,3);
#pzplot(GHp3);
#title('m. aberta GHp3');
#axis equal;

#%
#% definicao da malha fechada
#% realimentacao unitaria
#%
cloop1 = co.feedback(GHp1,1)
cloop2 = co.feedback(GHp2,1)
cloop3 = co.feedback(GHp3,1)
#%
#% Polos e zeros de malha fechada
#%

print('\nPolos e zeros cloop1')
print(co.pole(cloop1))
print(co.zero(cloop1))
print('\nPolos e zeros cloop2')
print(co.pole(cloop2))
print(co.zero(cloop2))
print('\nPolos e zeros cloop3')
print(co.pole(cloop3))
print(co.zero(cloop3))
#%
#%  Plot com os polos e zeros de malha fechada
#%

co.pzmap(cloop1, title = 'm. fechada CL1')
co.pzmap(cloop2, title = 'm. fechada CL2')
co.pzmap(cloop3, title = 'm. fechada CL3')

#figure(2);
#subplot(1,3,1)
#pzplot(cloop1);     
#title('m. fechada CL1');
#axis equal;
#subplot(1,3,2)
#pzplot(cloop2);
#title('m. fechada CL2');
#axis equal;
#subplot(1,3,3)
#pzplot(cloop3);
#title('m. fechada CL3');
#axis equal;

#%
#% Grafico da resposta a degrau
#%
tfinal = 20;
tspan = np.linspace(0,tfinal,int(tfinal//0.02))
tspan = tspan.reshape(-1,1)

y,t = co.step(cloop1,tspan)
plt.figure(7)
plt.plot(t,y)
plt.title("Step c.loop1")
plt.xlabel("Tempo[s]")
plt.ylabel("Amplitude")
plt.grid()

y1,t1 = co.step(cloop2,tspan)
plt.figure(8)
plt.plot(t1,y1)
plt.title("Step c.loop2")
plt.xlabel("Tempo[s]")
plt.ylabel("Amplitude")
plt.grid()

y2,t2 = co.step(cloop3,tspan)
plt.figure(9)
plt.plot(t,y,t1,y1,t2,y2)
plt.title("Step")
plt.legend(['Sistema 1', 'Sistema 2', 'Sistema 3'])
plt.xlabel("Tempo[s]")
plt.ylabel("Amplitude")
plt.grid()

#%
#% Caracteristicas da resposta a degrau
#% Tr - Tempo de subida, Ts - Tempo de acomodacao, Mp - Maximo sobresinal
#%
print('\ncaracteristicas da resposta a degrau cloop1')
#print(co.stepinfo(cloop1))
print('\ncaracteristicas da resposta a degrau cloop2')
print(co.stepinfo(cloop2))
print('\ncaracteristicas da resposta a degrau cloop3')
print(co.stepinfo(cloop3))

#%
#% Funcao de transferencia para calculo do esforco de controle u(t)
#%

esforco1 = co.feedback(Hp1,Gp)
esforco2 = co.feedback(Hp2,Gp)
esforco3 = co.feedback(Hp3,Gp)

y,t = co.step(esforco1,tspan)
plt.figure(10)
plt.plot(t,y)
plt.title("Step esforco1")
plt.xlabel("Tempo[s]")
plt.ylabel("Amplitude")
plt.grid()

y1,t1= co.step(esforco2,tspan)
plt.figure(11)
plt.plot(t1,y1)
plt.title("Step esforco2")
plt.xlabel("Tempo[s]")
plt.ylabel("Amplitude")
plt.grid()

y2,t2 = co.step(esforco3,tspan)
plt.figure(12)
plt.plot(t2,y2)
plt.title("Step esforco3")
plt.xlabel("Tempo[s]")
plt.ylabel("Amplitude")
plt.grid()

#%
#% Caracteristicas do esforco de controle
#%
print('\ncaracteristicas do esforco de controle de cloop1')
print(co.stepinfo(esforco1))
print('\ncaracteristicas do esforco de controle de cloop2')
print(co.stepinfo(esforco2))
print('\ncaracteristicas do esforco de controle de cloop3')
print(co.stepinfo(esforco3))
