# -*- coding: utf-8 -*-
"""
Created on Tue Mar 31 10:53:26 2020

@author: Luana Nunes
"""

import numpy as np
import control as co_general
import matplotlib.pyplot as plt
import control.matlab as co
import sympy

plt.close("all")

#% Parametros da funcao de transferencia
K1 = 100
Km = 2.083
Kg = 0.1
a  = 100
am = 1.71
#%
#% Funcao de transferencia da posicao angular do sistema
s = co.tf('s');
Gp = (K1*Km*Kg)/(s*(s+a)*(s+am))
#%
#% Calculo dos polos da funcao de transferencia Gp
print(co.pole(Gp))
#%
#% Expansao em fracoes parciais
#% caso nao haja multiplos polos:
#%       B(s)       R(1)       R(2)             R(n)
#%       ----  =  -------- + -------- + ... + -------- + K(s)
#%       A(s)     s - P(1)   s - P(2)         s - P(n)
#%
B=K1*Km*Kg;
A=[1,a+am,a*am,0]
print(sympy.apart(Gp))
#%
#% Plot dos polos e zeros de Gm
#%

co.pzmap(Gp)
#% Resposta a entrada degrau do sistema
tspan = np.linspace(0,10,int(10//0.02))
print(tspan)
tspan = tspan.reshape(-1,1)
y,t = co.step(Gp,tspan)
plt.figure(2)
plt.plot(t,y)
plt.title("Step response")
plt.xlabel("Tempo[s]")
plt.ylabel("Amplitude")
plt.grid()
#
##% Caracteristicas da resposta a degrau
#print(co.stepinfo(Gp))