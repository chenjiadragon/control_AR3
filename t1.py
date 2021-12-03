import math
import time

import sympy

import myEI
import serial
# from sympy import*




def move_arm(a, b, c):
   # (a, b, c)为击打点
   # 计算J2, J3的移动角度
   temp1 = (a-64.2) * (a-64.2)+(169.77-b)*(169.77-b)-440*440-305*305
   temp2 = 2*440*305
   t2 = math.acos(temp1/temp2)
   x = 440*math.cos(t2) + 305
   y = 440*math.sin(t2)
   z = a-64.2
   t = math.acos(x/(math.sqrt(x**2+y**2)))
   t1 = math.acos(z/(math.sqrt(x**2+y**2))) - t
   t1 = math.degrees(t1)
   t2 = math.degrees(t2)
   # print("t1:", math.degrees(t1))
   # print("t2:", math.degrees(t2))
   return [t1, t2]

# was 0.01 -90 90 0.01 0.01 0.01
# was 0 -65 65 0 0 0
# wcs 80 -340 420 -50 120 -75
# wcs 420 20 785 -50 100 -100
if __name__ == '__main__':
   # 串口通信
   port = "COM5"
   baud = 115200
   ser = serial.Serial(port, baud)

   # # 发送命令
   # while True:
   #    command = input("> ")
   #    if (command[0] == 'q'):
   #       break
   #    myEI.EI(command, ser)

   # 初始化
   time.sleep(1)
   command = "calRobot"
   myEI.EI(command, ser)
   time.sleep(1)

   command = "was 0 -132 141 165 -10.4 -75.27"
   myEI.EI(command, ser)

   time.sleep(1)
   l = move_arm(400, 0, 300)
   print(l)

   # time.sleep(1)
   # command = "wa 2 "+ str(l[0])
   # myEI.EI(command, ser)
   # command = "wa 3 "+str(l[1])
   # myEI.EI(command, ser)

   command = "wa 2 -52"
   myEI.EI(command, ser)
   command = "wa 3 122"
   myEI.EI(command, ser)

   time.sleep(1)


   # # 退出循环
   ser.close()
   myEI.f.close()



