import numpy

from numpy import random
'''
import matplotlib

matplotlib.use("TkAgg")

import matplotlib.pyplot as plt
'''

from matplotlib import pyplot as plt


P = 0.0
I = 0.0
D = 0.0

kP = 1.3
kI = 0.2
kD = 0.06

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
x = [0] * 100

for i in range(100):
    temp = numpy.random.normal(x0, 0.2, 7) # такая функция сгенерирует нормальное матожидание
    temp.sort()
    x[i] = temp[3]

correct = []

for i in range(100):
    correct.append(x0 + pid(x[i] - x0))

x_a = [i for i in range(100)]

plt.plot(x_a, x, 'g.-.', x_a, correct, 'b--')
plt.show()


