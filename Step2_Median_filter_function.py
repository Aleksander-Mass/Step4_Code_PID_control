# -*- coding: utf-8 -*-
"""
Created on Wed Nov  8 18:02:30 2023

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
# Шаг 2. Напишите функцию медианного фильтра:

x0 = 5.0
x = [0] * 1000

for i in range(1000):
    temp = numpy.random.normal(x0, 2.0, 7) # такая функция сгенерирует нормальное матожидание
    temp.sort()
    x[i] = temp[3]


print(x)