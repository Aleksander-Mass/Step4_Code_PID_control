from matplotlib import pyplot as plt
import numpy as np

class VehicleSimpleDynamic:
    def __init__(self, I_y, k_b, T_cmd, l, accInit, velInit, angleInit):
        """
        класс, который отвечает за динамику БЛА

        :param I_y: момент инерции
        :type I_y: float
        :param k_b: коэффициент тяги двигателя
        :type k_b: float
        :param T_cmd: Требуемая тяга
        :type T_cmd: int
        :param l: длина аппарата
        :type l: int
        :param accInit: начальное значение углового ускорения ЛА
        :type accInit: float
        :param velInit: начальное значение угловой скорости ЛА
        :type velInit: float
        :param angleInit: начальное значение углового положения
        :type angleInit: float

        """
        self.I_y = I_y
        self.k_b = k_b
        self.T_cmd = T_cmd
        self.l = l
        self.angle_acceleration = accInit
        self.angle_velocity = velInit
        self.angle = angleInit

    def build_dynamics_model(self, angle_acceleration_cmd):
        """
        построение модели динамики
        :param angle_acceleration_cmd: угловое ускорение
        :type angle_acceleration_cmd: float

        """
        w1 = angle_acceleration_cmd + self.T_cmd
        w2 = - angle_acceleration_cmd + self.T_cmd
        M_y = self.k_b * self.l * (w1 ** 2 - w2 ** 2)
        self.angle_acceleration = M_y / self.I_y

    def integrate(self, dt):
        """

        :param dt: шаг моделирования
        :type dt: float

        """
        # интегрируем ускорение методом эйлера
        self.angle_velocity += self.angle_acceleration * dt
        # Полученную скорость интегрируем для определения углового положения
        self.angle += self.angle_velocity * dt

    def calculateAngle(self, u, dt):
        """

        :param u: управляющие воздействие
        :type u: float
        :param dt: шаг моделирования
        :type dt: float

        """
        self.build_dynamics_model(u)
        self.integrate(dt)

    def getAngle(self):
        """

        :return: угол ЛА
        :rtype: float

        """
        return self.angle

    def getAcceleration(self):
        """

        :return: скорость ЛА
        :rtype: float

        """
        return self.angle_acceleration

    def getVelocity(self):
        """

         :return: ускорение
         :rtype: float

         """
        return self.angle_velocity


class AnglePosControlSystem():
    def __init__(self, k_p, k_i, k_d, controlLimit):
        """
        Система управления угловым положением ЛА

        :param k_p: коэффициент П регулятора
        :type k_p: float
        :param k_i: коэффициент И регулятора
        :type k_i: float
        :param k_d: коэффициент Д регулятора
        :type k_d: float
        :param controlLimit: ограничение по управляющему воздействию
        :type controlLimit: float

        """
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d
        self.desiredAngle = 0
        self.error = 0
        self.errorPast = 0
        self.integral = 0
        self.controlLimit = controlLimit

    def setDesiredAngle(self, desiredAngle):
        """

        :param desiredAngle: целевой угол ЛА
        :type desiredAngle: float

        """
        # данный метод устанавливает целевой угол ЛА,
        # к которому система с течением времени будет стремиться
        self.desiredAngle = desiredAngle

    def PID(self, currentAngle, dt):
        """

        :param currentAngle: текущий угол ЛА
        :type currentAngle: float
        :param dt: шаг моделирования
        :type dt: float

        """
        # Вычислим функцию ошибки
        self.error = self.desiredAngle - currentAngle
        # Вычисляем интеграл ошибки
        self.integral += self.error * dt
        # Рассчитаем целевую угловую скорость аппарата на основе ошибки
        angle_velocity = self.k_p * self.error + self.k_i * self.integral + \
            self.k_d * ((self.error - self.errorPast) / dt)
        # Установим предыдущую ошибку для использования в дальнейших итерациях
        self.errorPast = self.error
        # Вызовем звено насыщения для ограничения максимального управляющего воздействия
        angle_velocity = self.saturation(angle_velocity)
        return angle_velocity

    def saturation(self, inputVal):
        """

        :param inputVal: входное значение
        :type inputVal: float
        :return: выходное значение после прохождения проверки на ограничение
        :rtype: float

        """
        # Звено насыщения ограничивает размер входного параметра
        # На выходе метода абсолютное значение не может быть больше
        # заданного предела controlLimit
        if inputVal > self.controlLimit:
            inputVal = self.controlLimit
        elif inputVal < -self.controlLimit:
            inputVal = - self.controlLimit

        return inputVal


class AngleVelocityControlSystem():
    def __init__(self, k_p, k_i, k_d, angular_velocity):
        """
        Система управления угловой скоростью ЛА

        :param k_p: коэффициент П регулятора
        :type k_p: float
        :param k_i: коэффициент И регулятора
        :type k_i: float
        :param k_d: коэффициент Д регулятора
        :type k_d: float
        :param angular_velocity: угловая скорость (полученная из AnglePosControlSystem)
        :type angular_velocity: float

        """
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d
        self.angular_velocity = angular_velocity
        self.error = 0
        self.errorPast = 0
        self.integral = 0

    def PID(self, currentAngVelocity, angular_velocity, dt):
        """

        :param currentAngVelocity: текущая угловая скорость ЛА
        :type currentAngVelocity: float
        :param dt: шаг моделирования
        :type dt: float

        """
        # Вычислим функцию ошибки
        self.error = currentAngVelocity - angular_velocity
        # Вычисляем интеграл ошибки
        self.integral += self.error * dt
        # Рассчитаем целевое ускорение аппарата на основе ошибки
        angle_acceleration = self.k_p * self.error + self.k_i * self.integral + \
            self.k_d * ((self.error - self.errorPast) / dt)
        # Установим предыдущую ошибку для использования в дальнейших итерациях
        self.errorPast = self.error
        return angle_acceleration


class Simulator():

    def __init__(self, Tend, dt, angle_pos_controller, angle_velocity_controller, dynamicModel):
        """
        :param Tend: конечное время моделирования
        :type Tend: float
        :param dt: шаг моделирования
        :type dt: float
        :param angle_pos_controller: объект системы управления угловым положением ЛА
        :type angle_pos_controller: AnglePosControlSystem
        :param angle_pos_controller: объект системы управления угловой скоростью ЛА
        :type angle_pos_controller: AngleVelocityControlSystem
        :param dynamicModel: объект модели ЛА
        :type dynamicModel: VehicleSimpleDynamic
        """
        self.dt = dt
        self.Tend = Tend
        self.angle_pos_controller = angle_pos_controller
        self.angle_velocity_controller = angle_velocity_controller
        self.dynamicModel = dynamicModel
        self.accList = []
        self.velList = []
        self.angleList = []
        self.timeList = []

    def runSimulation(self):
        """

        метод запускает моделирование системы от 0 до конечного времени Tend
        с шагом dt

        """
        # Задаем 0 время и начинаем расчет до тех пор, пока
        # время не достигнет конечного значения Tend
        time = 0
        while time <= self.Tend:
            # получаем угол ЛА
            angle = self.dynamicModel.getAngle()
            # Получаем угловую скорость ЛА
            vel = self.dynamicModel.getVelocity()
            # Получаем угловое ускорение ЛА
            acc = self.dynamicModel.getAcceleration()
            # Записываем полученные значения в списки
            # для дальнейшего построения графиков
            self.angleList.append(angle)
            self.velList.append(vel)
            self.accList.append(acc)

            # рассчитываем новое управляющие воздействие
            # на основе текущего угла (angle) ЛА
            angle_velocity = self.angle_pos_controller.PID(angle, self.dt)
            angle_acceleration = self.angle_velocity_controller.PID(angle_velocity, vel, self.dt)
            # Рассчитываем угол ЛА с учетом полученного
            # управляющего воздействия
            self.dynamicModel.calculateAngle(angle_acceleration, self.dt)
            # увеличиваем время на dt, то есть на шаг моделирования
            time += self.dt

    def showPlots(self):
        """
        метод строит графики на основе измерений полученных в
        ходе моделирования системы

        """
        # создадим список с отсечками времени для построения оси Ox
        time = [i for i in np.arange(0, self.Tend + self.dt, self.dt)]

        f = plt.figure(constrained_layout=True)
        gs = f.add_gridspec(3, 5)
        ax1 = f.add_subplot(gs[0, :-1])
        ax1.plot(time, self.angleList)
        ax1.grid()
        ax1.set_yticks([0, 10, 20, 30])
        ax1.set_title('angle')
        ax1.set_xlabel('time')

        ax2 = f.add_subplot(gs[1, :-1])
        ax2.plot(time, self.velList, "g")
        ax2.grid()
        ax2.set_title('angular velocity')
        ax2.set_xlabel('time')

        ax3 = f.add_subplot(gs[2, :-1])
        ax3.plot(time, self.accList, "r")
        ax3.grid()
        ax3.set_title('angular acceleration')
        ax3.set_xlabel('time')

        plt.show()


'''
 Объявим параметры для моделирования
'''
# Коэффициенты ПИД регулятора контура управления угловым положением
k_p = 150  # коэффициент Пропорционального регулирования
k_i = 50  # коэффициент Интегрального регулирования
k_d = 20  # коэффициент Дифференциального регулирования

# Коэффициенты ПИД регулятора контура управления угловой скоростью
k_p_tetta = 150  # коэффициент Пропорционального регулирования
k_i_tetta = 50  # коэффициент Интегрального регулирования
k_d_tetta = 10  # коэффициент Дифференциального регулирования

dt = 0.01  # шаг моделирования системы (одна сотая секунды)
Tend = 10  # конечное время моделирования (10 сек по заданию)

# угол, к которому система должно стремится
theta_cmd = 30
# начальное угловое положение
tetta = 0
# начальная угловая скорость
angular_velocity = 0
# момент инерции
I_y = 7.16914 * 10 ** -5
# длина
l = 0.17
# Коэффициент тяги двигателя ЛА
k_b = 3.9865 * 10 ** -8
# Требуемая тяга (любое положительное число согласно заданию)
T_cmd = 10

# Ограничение на угловую скорость двигателей рад/сек
motorSpeedLimit = 1000

'''
Создадим объект контроллера и объект для нашей математической модели
'''
angle_pos_controller = AnglePosControlSystem(k_p, k_i, k_d, motorSpeedLimit)
# Установим целевой угол для нашей системы
angle_pos_controller.setDesiredAngle(theta_cmd)

angle_velocity_controller = AngleVelocityControlSystem(k_p_tetta, k_i_tetta, k_d_tetta, angular_velocity)

uavSimpleDynamic = VehicleSimpleDynamic(I_y, k_b, T_cmd, l, 0, 0, 0)

"""
Создадим объект симулятора и передадим в него контроллер
и математическую модель
"""
sim = Simulator(Tend, dt, angle_pos_controller, angle_velocity_controller, uavSimpleDynamic)
sim.runSimulation()  # запуск симулятора
sim.showPlots()  # построение графиков
