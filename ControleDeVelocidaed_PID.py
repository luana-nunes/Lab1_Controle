# -*- coding: utf-8 -*-
"""
Created on Tue Mar 31 11:52:12 2020

@author: Luana Nunes
"""
import numpy as np
import control as co_general
import matplotlib.pyplot as plt
import control.matlab as co
import sympy

plt.close("all")
#%
#% Controle de velocidade angular do motor eletrico CC
#% Testes com controle PID
#%
#% Definicao dos valores dos parametros do sistema
#%
#% Constante do tacometro
Ktac = 0.48
#% Parametros da funcao de transferencia Gw(s)
K1 = 100
Km = 2.083
Kg = 0.1
a  = 100
am = 1.71
#%
#% Funcao de transferencia da velocidade angular do sistema
s = co.tf('s')
Gw = (K1*Km*Kg)/((s+a)*(s+am))
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
#% Controlador 1
Kp1 = 1
Ti1 = float('inf')
Td1 = 0.1
N1  = 1
#Controlador P
if Ti1 == float('inf') and Td1 == 0 and N1 == float('inf'):
    H1  = Kp1
#Controlador PI
elif Td1 == 0 and N1 == float('inf'):
    H1 = Kp1*(1+(1/(Ti1*s)))
#Controlador PD
elif Ti1 == float('inf'):
    H1 = Kp1*(1+(Td1*(s/((Td1/N1)*(s)+1))))
#Controlador PID
else:
    H1 = Kp1*(1+(1/(Ti1*s))+(Td1*s/((Td1/N1)*(s)+1)))
Hw1 = Ktac*H1
#% Controlador 2
Kp2 = 1
Ti2 = float('inf')
Td2 = 0.5
N2  = 1
#Controlador P
if Ti2 == float('inf') and Td2 == 0 and N2 == float('inf'):
    H2  = Kp2
#Controlador PI
elif Td2 == 0 and N2 == float('inf'):
    H2 = Kp2*(1+(1/(Ti2*s)))
#Controlador PD
elif Ti2 == float('inf'):
    H2 = Kp2*(1+(Td2*s/((Td2/N2)*(s)+1)))
#Controlador PID
else:
    H2 = Kp2*(1+(1/(Ti2*s))+(Td2*s/((Td2/N2)*(s)+1)))
Hw2 = Ktac*H2
#% Controlador 3
Kp3 = 1
Ti3 = float('inf')
Td3 = 1.5
N3  = 1
#Controlador P
if Ti3 == float('inf') and Td3 == 0 and N3 == float('inf'):
    H3 = Kp3
#Controlador PI
elif Td3 == 0 and N3 == float('inf'):
    H3 = Kp3*(1+(1/(Ti3*s)))
#Controlador PD
elif Ti3 == float('inf'):
    H3 = Kp3*(1+(Td3*s/((Td3/N3)*(s)+1)))
#Controlador PID
else:
    H3 = Kp3*(1+(1/(Ti3*s))+(Td3*s/((Td3/N3)*(s)+1)))
Hw3 = Ktac*H3
#%
#% Definicao da malha aberta
#% Sao definidos 3 sistemas distintos
#%
GHw1 = Hw1*Gw
GHw2 = Hw2*Gw
GHw3 = Hw3*Gw
#%
#% Polos e zeros de malha aberta
#%
print('Polos e zeros - GHw1')
print(co.pole(GHw1))
print(co.zero(GHw1))
print('\nPolos e zeros - GHw2')
print(co.pole(GHw2))
print(co.zero(GHw2))
print('\nPolos e zeros - GHw3')
print(co.pole(GHw3))
print(co.zero(GHw3))
#%
#% Plot com os polos e zeros de malha aberta
#%


co.pzmap(GHw1, title = "GHw1")
co.pzmap(GHw2,title = "GHw2")
co.pzmap(GHw3, title = "GHw3")

#plt.subplots_adjust(left=None, bottom=None, right=None, top=None, wspace=None, hspace=0.45)
#
#
#plt.subplot(3,1,1)
#co.pzmap(GHw1)
#plt.title('m. aberta GHw1')
##axis equal
#
#plt.subplot(3,1,2)
#co.pzmap(GHw2)
#plt.title('m. aberta GHw2')
##axis equal
#
#plt.subplot(3,1,3)
#co.pzmap(GHw3)
#plt.title('m. aberta GHw3')
##axis equal
#%
#% definicao da malha fechada
#% realimentacao unitaria
#%
#% Funcao de transferencia em malha fechada
#% pode ser definida em Matlab utilizando-se
#% O comando feedback(S1,S2) 
#% Onde o sistema S1 se encontra na malha direta
#% e S2 se encontra na malha de realimentacao
#% No caso desse sistema a malha direta
#% e' G(s)H(s) e malha de realimentacao e' 
#% unitaria
#%
#% R(s)  E(s)|------|  |------|  U(s)
#%---->(+)---| H(s) |--| G(s) |------->
#%    _ ^    |------|  |------|    |
#%      |---------------------------                       
#%      
cloop1 = co.feedback(GHw1,1)
cloop2 = co.feedback(GHw2,1)
cloop3 = co.feedback(GHw3,1)
#%
#% Polos e zeros de malha fechada
#% comando pole() obtem os polos do sistema
#% e zero() obtem os zeros do sistema
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

#% comando pzplot()
#% plot os polos e zeros
co.pzmap(cloop1, title = 'm. fechada CL1')
#axis equal
co.pzmap(cloop2, title ='m. fechada CL2')
#axis equal
#subplot(1,3,3)
co.pzmap(cloop3, title ='m. fechada CL3')
#axis equal
#%
#% Grafico da resposta a degrau
#%
tfinal = 12
tspan = np.linspace(0,tfinal,int(tfinal//0.02))
tspan = tspan.reshape(-1,1)

#%
#% comando stepplot() simula e plota
#% a resposta a degrau unitaria
#%

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
plt.title("Step Sistemas")
plt.legend(["Sistema 1","Sistema 2","Sistema 3"])
plt.xlabel("Tempo[s]")
plt.ylabel("Amplitude")
plt.grid()

#stepplot(cloop1,cloop2,cloop3,tspan)
#grid on
#%
#% Caracteristicas da resposta a degrau
#% Tr - Tempo de subida, Ts - Tempo de acomodacao, Mp - Maximo sobresinal
#%
#% Comando stepinfo()
#% calcula as caracteristicas da resposta a degrau
print('\ncaracteristicas da resposta a degrau cloop1')
print(co.stepinfo(cloop1))
print('\ncaracteristicas da resposta a degrau cloop2')
print(co.stepinfo(cloop2))
print('\ncaracteristicas da resposta a degrau cloop3')
print(co.stepinfo(cloop3))
#%
#% Funcao de transferencia para calculo do esforco de controle u(t)
#% o sinal u(t) pode ser calculado definindo-se
#% um sistema de controle em malha fechada onde H(s)
#% esta na malha direta e G(s) na malha de realimentacao
#%
#% R(s)  E(s)|------|        U(s)
#%---->(+)---| H(s) |------------>
#%    _ ^    |------|    |
#%      |                |
#%      |    |------|    |
#%      |----| G(s) |<----
#%           |------|
#%      
esforco1 = co.feedback(Hw1,Gw)
esforco2 = co.feedback(Hw2,Gw)
esforco3 = co.feedback(Hw3,Gw)

y,t = co.step(esforco1,tspan)
plt.figure(10)
plt.plot(t,y)
plt.title("Step esforco1")
plt.xlabel("Tempo[s]")
plt.ylabel("Amplitude")
plt.grid()

y,t = co.step(esforco2,tspan)
plt.figure(11)
plt.plot(t,y)
plt.title("Step esforco2")
plt.xlabel("Tempo[s]")
plt.ylabel("Amplitude")
plt.grid()

y,t = co.step(esforco3,tspan)
plt.figure(12)
plt.plot(t,y)
plt.title("Step esforco3")
plt.xlabel("Tempo[s]")
plt.ylabel("Amplitude")
plt.grid()

#figure(4)
#stepplot(esforco1,esforco2,esforco3,tspan)
#grid on
#%
#% Caracteristicas do esforco de controle
#%
print('\ncaracteristicas do esforco de controle de cloop1')
print(co.stepinfo(esforco1))
print('\ncaracteristicas do esforco de controle de cloop2')
print(co.stepinfo(esforco2))
print('\ncaracteristicas do esforco de controle de cloop3')
print(co.stepinfo(esforco3))
