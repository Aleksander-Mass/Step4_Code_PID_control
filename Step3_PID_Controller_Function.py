# -*- coding: utf-8 -*-
"""
Created on Wed Nov  8 18:13:43 2023

@author: alexa
"""

# самый простой случай ПИД-регулятора без графика
'''
первая задача сгенерировать показания датчика
'''

import numpy
# или другой вариант 
'''
from numpy import random
'''



import matplotlib
matplotlib.use("TkAgg")

P = 0.0
I = 0.0
D = 0.0

kP = 1.0
kI = 0.0
kD = 0.0

e = 0.0

# функция ПИД - регулятора

def pid(err):
    global I
    global e
    P = kP * err
    I = I + kI * err
    D = kD * (err - e)
    e = err

    return P + I + D

x0 = 5.0
x = [0] * 1000

for i in range(1000):
    temp = numpy.random.normal(x0, 2.0, 7) # такая функция сгенерирует нормальное матожидание
    temp.sort()
    x[i] = temp[3]


for i in range(1000):
    print(pid(x[i] - x0))

