# -*- coding: utf-8 -*-
"""
Created on Tue Mar 31 10:27:59 2020

@author: Luana Nunes
"""
import numpy as np
import control as co_general
import matplotlib.pyplot as plt
import control.matlab as co
import sympy


# Parametros da funcao de transferencia
K1 = 100
Km = 2.083
Kg = 0.1
a  = 100
am = 1.71
#%
#% Funcao de transferencia da velocidade angular do sistema
s = co.tf('s')
Gomega = (K1*Km*Kg)/((s+a)*(s+am))
#%
#% Calculo dos polos da funcao de transferencia Gm
print(co.pole(Gomega))
#%
#% Expansao em fracoes parciais
#% caso nao haja multiplos polos:
#%       B(s)       R(1)       R(2)             R(n)
#%       ----  =  -------- + -------- + ... + -------- + K(s)
#%       A(s)     s - P(1)   s - P(2)         s - P(n)
#%
B=K1*Km*Kg
A=[1, a+am, a*am]
print(sympy.apart(Gomega))

#%
#% Plot dos polos e zeros de Gm
#%
co.pzmap(Gomega)
#% Resposta a entrada degrau do sistema
tspan = np.linspace(0,10,int(10//0.02))
print(tspan)
tspan = tspan.reshape(-1,1)
y,t = co.step(Gomega,tspan)
plt.figure(2)
plt.plot(t,y)
plt.xlabel("Tempo[s]")
plt.ylabel("Amplitude")
plt.grid()
##% Caracteristicas da resposta a degrau
print(co.stepinfo(Gomega))
