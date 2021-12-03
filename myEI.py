import serial
import time
import math
import numpy as np

# 一些全局参数
global speed
global ACCdur
global ACCspeed
global DECdur
global DECspeed
DHr = [0.0] * 7
DHa = [0.0] * 7
DHd = [0.0] * 7
DHt = [0.0] * 7
JNegAngLim = [0] * 7
JPosAngLim = [0] * 7
JStepLim = [0] * 7
JDegPerStep = [0] * 7   # 每步移动步数
JStepCur = [0] * 7
JAngCur = [0] * 7
global XcurPos
global YcurPos
global ZcurPos
global RxcurPos
global RycurPos
global RzcurPos

# 读取文件
filepath = "./parameter.txt"

# 读取文件
f = open(filepath, 'r')
lines = f.readlines()
# 赋初值
speed = int(lines[1].strip())
ACCdur = int(lines[2].strip())
ACCspeed = int(lines[3].strip())
DECdur = int(lines[4].strip())
DECspeed = int(lines[5].strip())
DHr[1] = float(lines[7].strip())
DHr[2] = float(lines[8].strip())
DHr[3] = float(lines[9].strip())
DHr[4] = float(lines[10].strip())
DHr[5] = float(lines[11].strip())
DHr[6] = float(lines[12].strip())
DHa[1] = float(lines[13].strip())
DHa[2] = float(lines[14].strip())
DHa[3] = float(lines[15].strip())
DHa[4] = float(lines[16].strip())
DHa[5] = float(lines[17].strip())
DHa[6] = float(lines[18].strip())
DHd[1] = float(lines[19].strip())
DHd[2] = float(lines[20].strip())
DHd[3] = float(lines[21].strip())
DHd[4] = float(lines[22].strip())
DHd[5] = float(lines[23].strip())
DHd[6] = float(lines[24].strip())
DHt[1] = float(lines[25].strip())
DHt[2] = float(lines[26].strip())
DHt[3] = float(lines[27].strip())
DHt[4] = float(lines[28].strip())
DHt[5] = float(lines[29].strip())
DHt[6] = float(lines[30].strip())
JNegAngLim[1] = float(lines[32].strip())
JPosAngLim[1] = float(lines[33].strip())
JStepLim[1] = float(lines[34].strip())
# JDegPerStep[1] = float((J1PosAngLim - J1NegAngLim) / float(J1StepLim))
JStepCur[1] = 10
JNegAngLim[2] = float(lines[35].strip())
JPosAngLim[2] = float(lines[36].strip())
JStepLim[2] = float(lines[37].strip())
# JDegPerStep[2] = float((J2PosAngLim - J2NegAngLim) / float(J2StepLim))
JStepCur[2] = 10
JNegAngLim[3] = float(lines[38].strip())
JPosAngLim[3] = float(lines[39].strip())
JStepLim[3] = float(lines[40].strip())
# JDegPerStep[3] = float((J3PosAngLim - J3NegAngLim) / float(J3StepLim))
JStepCur[3] = 10
JNegAngLim[4] = float(lines[41].strip())
JPosAngLim[4] = float(lines[42].strip())
JStepLim[4] = float(lines[43].strip())
# JDegPerStep[4] = float((J4PosAngLim - J4NegAngLim) / float(J4StepLim))
JStepCur[4] = 10
JNegAngLim[5] = float(lines[44].strip())
JPosAngLim[5] = float(lines[45].strip())
JStepLim[5] = float(lines[46].strip())
# JDegPerStep[5] = float((J5PosAngLim - J5NegAngLim) / float(J5StepLim))
JStepCur[5] = 10
JNegAngLim[6] = float(lines[47].strip())
JPosAngLim[6] = float(lines[48].strip())
JStepLim[6] = float(lines[49].strip())
# JDegPerStep[6] = float((J6PosAngLim - J6NegAngLim) / float(J6StepLim))
JStepCur[6] = 10
for i in range(6):
    JDegPerStep[i+1] = float((JPosAngLim[i+1] - JNegAngLim[i+1]) / float(JStepLim[i+1]))



# 正运动学
def myCalcFwdKin():
    global XcurPos
    global YcurPos
    global ZcurPos
    global RxcurPos
    global RycurPos
    global RzcurPos
    global WC
    for i in range(6):
        if (JAngCur[i + 1] == 0):
            JAngCur[i + 1] = .0001
    ## Set Wrist Config （暂时没看懂）
    if (JAngCur[5] > 0):
        WC = "F"
    else:
        WC = "N"
    ## CONVERT TO RADIANS
    ## DH TABLE
    calDHt = [0.0]*7
    calDHr = [0.0]*7
    for i in range(6):
        calDHt[i+1] = math.radians(float(JAngCur[i+1])+DHt[i+1])
        calDHr[i+1] = math.radians(float(DHr[i+1]))
    calDHa = DHa
    calDHd = DHd
    ## DH矩阵计算
    DH65 = np.zeros((4, 4))
    DH54 = np.zeros((4, 4))
    DH43 = np.zeros((4, 4))
    DH32 = np.zeros((4, 4))
    DH21 = np.zeros((4, 4))
    DH10 = np.zeros((4, 4))
    DH = [np.zeros((4, 4)), DH10, DH21, DH32, DH43, DH54, DH65]
    for i in range(1, 7):
        DH[i][0][0] = math.cos(calDHt[i])
        DH[i][0][1] = -math.sin(calDHt[i])*math.cos(calDHr[i])
        DH[i][0][2] = math.sin((calDHt[i]))*math.sin(calDHr[i])
        DH[i][0][3] = calDHa[i]*math.cos(calDHt[i])

        DH[i][1][0] = math.sin(calDHt[i])
        DH[i][1][1] = math.cos(calDHt[i])*math.cos(calDHr[i])
        DH[i][1][2] = -math.cos(calDHt[i])*math.sin(calDHr[i])
        DH[i][1][3] = calDHa[i]*math.sin(calDHt[i])

        DH[i][2][1] = math.sin(calDHr[i])
        DH[i][2][2] = math.cos(calDHr[i])
        DH[i][2][3] = calDHd[i]

        DH[i][3][3] = 1
        # print(DH[i])
    DH60 = np.identity(4, float)
    for i in range(6):
        # DH60 = DH60 * DH[1] * DH[2] * DH[3] * DH[4] * DH[5] * DH[6]
        DH60 = DH60.dot(DH[i+1])
        # print(DH60)

    # was 0.01 -90 90 0.01 0.01 0.01
    ## GET YPR
    XcurPos = DH60[0][3]
    YcurPos = DH60[1][3]
    ZcurPos = DH60[2][3]
    I8 = math.atan2(math.sqrt((DH60[0][2] ** 2) + (DH60[1][2] ** 2)), -DH60[2][2])
    I7 = math.atan2((DH60[2][0] / I8), (DH60[2][1] / I8))
    I9 = math.atan2((DH60[0][2] / I8), (DH60[1][2] / I8))
    RxcurPos = math.degrees(I9)
    RycurPos = math.degrees(I8)
    RzcurPos = math.degrees(I7)
    # 滚、仰、偏变换解
    # myRx = math.atan2(DH60[1][0], DH60[0][0]) + math.pi
    # myRy = math.atan2(-DH60[2][0], math.cos(myRx)*DH60[0][0]+math.sin(myRx)*DH60[1][0])
    # myRz = math.atan2(math.sin(myRx)*DH60[0][2]-math.cos(myRx)*DH60[1][2], -math.sin(myRx)*DH60[0][1]+math.cos(myRx)*DH60[1][1])
    # myRx = math.degrees(myRx)
    # myRy = math.degrees(myRy)
    # myRz = math.degrees(myRz)
    # 欧拉变换解
    myRx = math.atan2(DH60[1][2], DH60[0][2]) + math.pi
    myRy = math.atan2(math.cos(myRx)*DH60[0][2]+math.sin(myRx)*DH60[1][2], DH60[2][2])
    myRz = math.atan2(-math.sin(myRx)*DH60[0][0]+math.cos(myRx)*DH60[0][1], -math.sin(myRx)*DH60[0][1]+math.cos(myRx)*DH60[1][1])
    myRx = math.degrees(myRx)
    myRy = math.degrees(myRy)
    myRz = math.degrees(myRz)


    # print("Rx:",RxcurPos,"\nRy:", RycurPos, "\nRz:", RzcurPos)
    # print("myRx:", myRx, "\nmyRy:", myRy, "\nmyRz:", myRz)

# 逆运动学计算，由位姿计算关节角度
def CalcRevKin(CX, CY, CZ, CRx, CRy, CRz):
    angle = [0.0] * 7

    for i in range(6):
        if(JAngCur[i] == 0):
            JAngCur[i] = .0001
    # input
    O4 = CX
    O5 = CY
    O6 = CZ
    O9 = CRx
    O8 = CRy
    O7 = CRz
    V8 = 'N'
    if (O4 == 0):
        O4 = .0001
    if (O5 == 0):
        O5 = .0001
    if (O6 == 0):
        O6 = .0001
    if (O7 == 0):
        O7 = .0001
    if (O8 == 0):
        O8 = .0001
    if (O9 == 0):
        O9 = .0001
    # quadrant
    if (O4 > 0 and O5 > 0):
        V9 = 1
    elif (O4 > 0 and O5 < 0):
        V9 = 2
    elif (O4 < 0 and O5 < 0):
        V9 = 3
    elif (O4 < 0 and O5 > 0):
        V9 = 4
    ## DH TABLE
    D13 = math.radians(DHr[1])
    D14 = math.radians(DHr[2])
    D15 = math.radians(DHr[3])
    D16 = math.radians(DHr[4])
    D17 = math.radians(DHr[5])
    D18 = math.radians(DHr[6])
    E13 = DHd[1]
    E14 = DHd[2]
    E15 = DHd[3]
    E16 = DHd[4]
    E17 = DHd[5]
    E18 = DHd[6]
    F13 = DHa[1]
    F14 = DHa[2]
    F15 = DHa[3]
    F16 = DHa[4]
    F17 = DHa[5]
    F18 = DHa[6]
    ## WORK FRAME INPUT
    H13 = -0.0
    H14 = -0.0
    H15 = -0.0
    H16 = -0.0
    H17 = -0.0
    H18 = -0.0
    ## TOOL FRAME INPUT
    J13 = -0.0
    J14 = -0.0
    J15 = -0.0
    J16 = -0.0
    J17 = -0.0
    J18 = -0.0
    ## WORK FRAME TABLE
    N30 = math.cos(math.radians(H18)) * math.cos(math.radians(H17))
    O30 = -math.sin(math.radians(H18)) * math.cos(math.radians(H16)) + math.cos(math.radians(H18)) * math.sin(
        math.radians(H17)) * math.sin(math.radians(H16))
    P30 = math.sin(math.radians(H18)) * math.sin(math.radians(H16)) + math.cos(math.radians(H18)) * math.sin(
        math.radians(H17)) * math.cos(math.radians(H16))
    Q30 = H13
    N31 = math.sin(math.radians(H18)) * math.cos(math.radians(H17))
    O31 = math.cos(math.radians(H18)) * math.cos(math.radians(H16)) + math.sin(math.radians(H18)) * math.sin(
        math.radians(H17)) * math.sin(math.radians(H16))
    P31 = -math.cos(math.radians(H18)) * math.sin(math.radians(H16)) + math.sin(math.radians(H18)) * math.sin(
        math.radians(H17)) * math.cos(math.radians(H16))
    Q31 = H14
    N32 = -math.sin(math.radians(H18))
    O32 = math.cos(math.radians(H17)) * math.sin(math.radians(H16))
    P32 = math.cos(math.radians(H17)) * math.cos(math.radians(H16))
    Q32 = H15
    N33 = 0
    O33 = 0
    P33 = 0
    Q33 = 1
    ## R 0-T
    X30 = math.cos(math.radians(O7)) * math.cos(math.radians(O9)) - math.cos(math.radians(O8)) * math.sin(
        math.radians(O7)) * math.sin(math.radians(O9))
    Y30 = math.cos(math.radians(O9)) * math.sin(math.radians(O7)) + math.cos(math.radians(O7)) * math.cos(
        math.radians(O8)) * math.sin(math.radians(O9))
    Z30 = math.sin(math.radians(O8)) * math.sin(math.radians(O9))
    AA30 = O4
    X31 = math.cos(math.radians(O8)) * math.cos(math.radians(O9)) * math.sin(math.radians(O7)) + math.cos(
        math.radians(O7)) * math.sin(math.radians(O9))
    Y31 = math.cos(math.radians(O7)) * math.cos(math.radians(O8)) * math.cos(math.radians(O9)) - math.sin(
        math.radians(O7)) * math.sin(math.radians(O9))
    Z31 = math.cos(math.radians(O9)) * math.sin(math.radians(O8))
    AA31 = O5
    X32 = math.sin(math.radians(O7)) * math.sin(math.radians(O8))
    Y32 = math.cos(math.radians(O7)) * math.sin(math.radians(O8))
    Z32 = -math.cos(math.radians(O8))
    AA32 = O6
    X33 = 0
    Y33 = 0
    Z33 = 0
    AA33 = 1
    ## R 0-T   offset by work frame
    X36 = ((N30 * X30) + (O30 * X31) + (P30 * X32) + (Q30 * X33)) * -1
    Y36 = (N30 * Y30) + (O30 * Y31) + (P30 * Y32) + (Q30 * Y33)
    Z36 = (N30 * Z30) + (O30 * Z31) + (P30 * Z32) + (Q30 * Z33)
    AA36 = (N30 * AA30) + (O30 * AA31) + (P30 * AA32) + (Q30 * AA33)
    X37 = (N31 * X30) + (O31 * X31) + (P31 * X32) + (Q31 * X33)
    Y37 = (N31 * Y30) + (O31 * Y31) + (P31 * Y32) + (Q31 * Y33)
    Z37 = (N31 * Z30) + (O31 * Z31) + (P31 * Z32) + (Q31 * Z33)
    AA37 = (N31 * AA30) + (O31 * AA31) + (P31 * AA32) + (Q31 * AA33)
    X38 = (N32 * X30) + (O32 * X31) + (P32 * X32) + (Q32 * X33)
    Y38 = (N32 * Y30) + (O32 * Y31) + (P32 * Y32) + (Q32 * Y33)
    Z38 = (N32 * Z30) + (O32 * Z31) + (P32 * Z32) + (Q32 * Z33)
    AA38 = (N32 * AA30) + (O32 * AA31) + (P32 * AA32) + (Q32 * AA33)
    X39 = (N33 * X30) + (O33 * X31) + (P33 * X32) + (Q33 * X33)
    Y39 = (N33 * Y30) + (O33 * Y31) + (P33 * Y32) + (Q33 * Y33)
    Z39 = (N33 * Z30) + (O33 * Z31) + (P33 * Z32) + (Q33 * Z33)
    AA39 = (N33 * AA30) + (O33 * AA31) + (P33 * AA32) + (Q33 * AA33)
    ## TOOL FRAME
    X42 = math.cos(math.radians(J18)) * math.cos(math.radians(J17))
    Y42 = -math.sin(math.radians(J18)) * math.cos(math.radians(J16)) + math.cos(math.radians(J18)) * math.sin(
        math.radians(J17)) * math.sin(math.radians(J16))
    Z42 = math.sin(math.radians(J18)) * math.sin(math.radians(J16)) + math.cos(math.radians(J18)) * math.sin(
        math.radians(J17)) * math.cos(math.radians(J16))
    AA42 = (J13)
    X43 = math.sin(math.radians(J18)) * math.cos(math.radians(J17))
    Y43 = math.cos(math.radians(J18)) * math.cos(math.radians(J16)) + math.sin(math.radians(J18)) * math.sin(
        math.radians(J17)) * math.sin(math.radians(J16))
    Z43 = -math.cos(math.radians(J18)) * math.sin(math.radians(J16)) + math.sin(math.radians(J18)) * math.sin(
        math.radians(J17)) * math.cos(math.radians(J16))
    AA43 = (J14)
    X44 = -math.sin(math.radians(J18))
    Y44 = math.cos(math.radians(J17)) * math.sin(math.radians(J16))
    Z44 = math.cos(math.radians(J17)) * math.cos(math.radians(J16))
    AA44 = (J15)
    X45 = 0
    Y45 = 0
    Z45 = 0
    AA45 = 1
    ## INVERT TOOL FRAME
    X48 = X42
    Y48 = X43
    Z48 = X44
    AA48 = (X48 * AA42) + (Y48 * AA43) + (Z48 * AA44)
    X49 = Y42
    Y49 = Y43
    Z49 = Y44
    AA49 = (X49 * AA42) + (Y49 * AA43) + (Z49 * AA44)
    X50 = Z42
    Y50 = Z43
    Z50 = Z44
    AA50 = (X50 * AA42) + (Y50 * AA43) + (Z50 * AA44)
    X51 = 0
    Y51 = 0
    Z51 = 0
    AA51 = 1
    ## R 0-6
    X54 = (X36 * X48) + (Y36 * X49) + (Z36 * X50) + (AA36 * X51)
    Y54 = (X36 * Y48) + (Y36 * Y49) + (Z36 * Y50) + (AA36 * Y51)
    Z54 = (X36 * Z48) + (Y36 * Z49) + (Z36 * Z50) + (AA36 * Z51)
    AA54 = (X36 * AA48) + (Y36 * AA49) + (Z36 * AA50) + (AA36 * AA51)
    X55 = (X37 * X48) + (Y37 * X49) + (Z37 * X50) + (AA37 * X51)
    Y55 = (X37 * Y48) + (Y37 * Y49) + (Z37 * Y50) + (AA37 * Y51)
    Z55 = (X37 * Z48) + (Y37 * Z49) + (Z37 * Z50) + (AA37 * Z51)
    AA55 = (X37 * AA48) + (Y37 * AA49) + (Z37 * AA50) + (AA37 * AA51)
    X56 = (X38 * X48) + (Y38 * X49) + (Z38 * X50) + (AA38 * X51)
    Y56 = (X38 * Y48) + (Y38 * Y49) + (Z38 * Y50) + (AA38 * Y51)
    Z56 = (X38 * Z48) + (Y38 * Z49) + (Z38 * Z50) + (AA38 * Z51)
    AA56 = (X38 * AA48) + (Y38 * AA49) + (Z38 * AA50) + (AA38 * AA51)
    X57 = (X39 * X48) + (Y39 * X49) + (Z39 * X50) + (AA39 * X51)
    Y57 = (X39 * Y48) + (Y39 * Y49) + (Z39 * Y50) + (AA39 * Y51)
    Z57 = (X39 * Z48) + (Y39 * Z49) + (Z39 * Z50) + (AA39 * Z51)
    AA57 = (X39 * AA48) + (Y39 * AA49) + (Z39 * AA50) + (AA39 * AA51)
    ## REMOVE R 0-6
    X60 = math.cos(math.radians(180))
    Y60 = math.sin(math.radians(180))
    Z60 = 0
    AA60 = 0
    X61 = -math.sin(math.radians(180)) * math.cos(D18)
    Y61 = math.cos(math.radians(180)) * math.cos(D18)
    Z61 = math.sin(D18)
    AA61 = 0
    X62 = math.sin(math.radians(180)) * math.sin(D18)
    Y62 = -math.cos(math.radians(180)) * math.sin(D18)
    Z62 = math.cos(D18)
    AA62 = -E18
    X63 = 0
    Y63 = 0
    Z63 = 0
    AA63 = 1
    ## R 0-5 (center spherica wrist)
    X66 = (X54 * X60) + (Y54 * X61) + (Z54 * X62) + (AA54 * X63)
    Y66 = (X54 * Y60) + (Y54 * Y61) + (Z54 * Y62) + (AA54 * Y63)
    Z66 = (X54 * Z60) + (Y54 * Z61) + (Z54 * Z62) + (AA54 * Z63)
    AA66 = (X54 * AA60) + (Y54 * AA61) + (Z54 * AA62) + (AA54 * AA63)
    X67 = (X55 * X60) + (Y55 * X61) + (Z55 * X62) + (AA55 * X63)
    Y67 = (X55 * Y60) + (Y55 * Y61) + (Z55 * Y62) + (AA55 * Y63)
    Z67 = (X55 * Z60) + (Y55 * Z61) + (Z55 * Z62) + (AA55 * Z63)
    AA67 = (X55 * AA60) + (Y55 * AA61) + (Z55 * AA62) + (AA55 * AA63)
    X68 = (X56 * X60) + (Y56 * X61) + (Z56 * X62) + (AA56 * X63)
    Y68 = (X56 * Y60) + (Y56 * Y61) + (Z56 * Y62) + (AA56 * Y63)
    Z68 = (X56 * Z60) + (Y56 * Z61) + (Z56 * Z62) + (AA56 * Z63)
    AA68 = (X56 * AA60) + (Y56 * AA61) + (Z56 * AA62) + (AA56 * AA63)
    X69 = (X57 * X60) + (Y57 * X61) + (Z57 * X62) + (AA57 * X63)
    Y69 = (X57 * Y60) + (Y57 * Y61) + (Z57 * Y62) + (AA57 * Y63)
    Z69 = (X57 * Z60) + (Y57 * Z61) + (Z57 * Z62) + (AA57 * Z63)
    AA69 = (X57 * AA60) + (Y57 * AA61) + (Z57 * AA62) + (AA57 * AA63)
    ## CALCULATE J1 ANGLE
    O13 = math.atan((AA67) / (AA66))
    if (V9 == 1):
        P13 = math.degrees(O13)
    if (V9 == 2):
        P13 = math.degrees(O13)
    if (V9 == 3):
        P13 = -180 + math.degrees(O13)
    if (V9 == 4):
        P13 = 180 + math.degrees(O13)
    ## CALCULATE J2 ANGLE	FWD
    O16 = math.sqrt(((abs(AA67)) ** 2) + ((abs(AA66)) ** 2))
    O17 = AA68 - E13
    O18 = O16 - F13
    O19 = math.sqrt((O17 ** 2) + (O18 ** 2))
    O20 = math.sqrt((E16 ** 2) + (F15 ** 2))
    O21 = math.degrees(math.atan(O17 / O18))
    O22 = math.degrees(math.acos(((F14 ** 2) + (O19 ** 2) - (abs(O20) ** 2)) / (2 * F14 * O19)))
    try:
        O25 = math.degrees(math.atan(abs(E16) / F15))
    except:
        O25 = 90
    O23 = 180 - math.degrees(math.acos(((abs(O20) ** 2) + (F14 ** 2) - (O19 ** 2)) / (2 * abs(O20) * F14))) + (90 - O25)
    O26 = -(O21 + O22)
    O27 = O23
    ## CALCULATE J2 ANGLE	MID
    P18 = -O18
    P19 = math.sqrt((O17 ** 2) + (P18 ** 2))
    P21 = math.degrees(math.acos(((F14 ** 2) + (P19 ** 2) - (abs(O20) ** 2)) / (2 * F14 * P19)))
    P22 = math.degrees(math.atan(P18 / O17))
    P23 = 180 - math.degrees(math.acos(((abs(O20) ** 2) + (F14 ** 2) - (P19 ** 2)) / (2 * abs(O20) * F14))) + (90 - O25)
    P24 = 90 - (P21 + P22)
    P26 = -180 + P24
    P27 = P23
    ## J1,J2,J3
    Q4 = P13
    if (O18 < 0):
        Q5 = P26
        Q6 = P27
    else:
        Q5 = O26
        Q6 = O27
    ## J1
    N36 = math.cos(math.radians(Q4))
    O36 = -math.sin(math.radians(Q4)) * math.cos(D13)
    P36 = math.sin(math.radians(Q4)) * math.sin(D13)
    Q36 = F13 * math.cos(math.radians(Q4))
    N37 = math.sin(math.radians(Q4))
    O37 = math.cos(math.radians(Q4)) * math.cos(D13)
    P37 = -math.cos(math.radians(Q4)) * math.sin(D13)
    Q37 = F13 * math.sin(math.radians(Q4))
    N38 = 0
    O38 = math.sin(D13)
    P38 = math.cos(D13)
    Q38 = E13
    N39 = 0
    O39 = 0
    P39 = 0
    Q39 = 1
    ## J2
    N42 = math.cos(math.radians(Q5))
    O42 = -math.sin(math.radians(Q5)) * math.cos(D14)
    P42 = math.sin(math.radians(Q5)) * math.sin(D14)
    Q42 = F14 * math.cos(math.radians(Q5))
    N43 = math.sin(math.radians(Q5))
    O43 = math.cos(math.radians(Q5)) * math.cos(D14)
    P43 = -math.cos(math.radians(Q5)) * math.sin(D14)
    Q43 = F14 * math.sin(math.radians(Q5))
    N44 = 0
    O44 = math.sin(D14)
    P44 = math.cos(D14)
    Q44 = E14
    N45 = 0
    O45 = 0
    P45 = 0
    Q45 = 1
    ## J3
    N48 = math.cos(math.radians((Q6) - 90))
    O48 = -math.sin(math.radians((Q6) - 90)) * math.cos(D15)
    P48 = math.sin(math.radians((Q6) - 90)) * math.sin(D15)
    Q48 = F15 * math.cos(math.radians((Q6) - 90))
    N49 = math.sin(math.radians((Q6) - 90))
    O49 = math.cos(math.radians((Q6) - 90)) * math.cos(D15)
    P49 = -math.cos(math.radians((Q6) - 90)) * math.sin(D15)
    Q49 = F15 * math.sin(math.radians((Q6) - 90))
    N50 = 0
    O50 = math.sin(D15)
    P50 = math.cos(D15)
    Q50 = E15
    N51 = 0
    O51 = 0
    P51 = 0
    Q51 = 0
    ## R 0-1
    S33 = (N30 * N36) + (O30 * N37) + (P30 * N38) + (Q30 * N39)
    T33 = (N30 * O36) + (O30 * O37) + (P30 * O38) + (Q30 * O39)
    U33 = (N30 * P36) + (O30 * P37) + (P30 * P38) + (Q30 * P39)
    V33 = (N30 * Q36) + (O30 * Q37) + (P30 * Q38) + (Q30 * Q39)
    S34 = (N31 * N36) + (O31 * N37) + (P31 * N38) + (Q31 * N39)
    T34 = (N31 * O36) + (O31 * O37) + (P31 * O38) + (Q31 * O39)
    U34 = (N31 * P36) + (O31 * P37) + (P31 * P38) + (Q31 * P39)
    V34 = (N31 * Q36) + (O31 * Q37) + (P31 * Q38) + (Q31 * Q39)
    S35 = (N32 * N36) + (O32 * N37) + (P32 * N38) + (Q32 * N39)
    T35 = (N32 * O36) + (O32 * O37) + (P32 * O38) + (Q32 * O39)
    U35 = (N32 * P36) + (O32 * P37) + (P32 * P38) + (Q32 * P39)
    V35 = (N32 * Q36) + (O32 * Q37) + (P32 * Q38) + (Q32 * Q39)
    S36 = (N33 * N36) + (O33 * N37) + (P33 * N38) + (Q33 * N39)
    T36 = (N33 * O36) + (O33 * O37) + (P33 * O38) + (Q33 * O39)
    U36 = (N33 * P36) + (O33 * P37) + (P33 * P38) + (Q33 * P39)
    V36 = (N33 * Q36) + (O33 * Q37) + (P33 * Q38) + (Q33 * Q39)
    ## R 0-2
    S39 = (S33 * N42) + (T33 * N43) + (U33 * N44) + (V33 * N45)
    T39 = (S33 * O42) + (T33 * O43) + (U33 * O44) + (V33 * O45)
    U39 = (S33 * P42) + (T33 * P43) + (U33 * P44) + (V33 * P45)
    V39 = (S33 * Q42) + (T33 * Q43) + (U33 * Q44) + (V33 * Q45)
    S40 = (S34 * N42) + (T34 * N43) + (U34 * N44) + (V34 * N45)
    T40 = (S34 * O42) + (T34 * O43) + (U34 * O44) + (V34 * O45)
    U40 = (S34 * P42) + (T34 * P43) + (U34 * P44) + (V34 * P45)
    V40 = (S34 * Q42) + (T34 * Q43) + (U34 * Q44) + (V34 * Q45)
    S41 = (S35 * N42) + (T35 * N43) + (U35 * N44) + (V35 * N45)
    T41 = (S35 * O42) + (T35 * O43) + (U35 * O44) + (V35 * O45)
    U41 = (S35 * P42) + (T35 * P43) + (U35 * P44) + (V35 * P45)
    V41 = (S35 * Q42) + (T35 * Q43) + (U35 * Q44) + (V35 * Q45)
    S42 = (S36 * N42) + (T36 * N43) + (U36 * N44) + (V36 * N45)
    T42 = (S36 * O42) + (T36 * O43) + (U36 * O44) + (V36 * O45)
    U42 = (S36 * P42) + (T36 * P43) + (U36 * P44) + (V36 * P45)
    V42 = (S36 * Q42) + (T36 * Q43) + (U36 * Q44) + (V36 * Q45)
    ## R 0-3
    S45 = (S39 * N48) + (T39 * N49) + (U39 * N50) + (V39 * N51)
    T45 = (S39 * O48) + (T39 * O49) + (U39 * O50) + (V39 * O51)
    U45 = (S39 * P48) + (T39 * P49) + (U39 * P50) + (V39 * P51)
    V45 = (S39 * Q48) + (T39 * Q49) + (U39 * Q50) + (V39 * Q51)
    S46 = (S40 * N48) + (T40 * N49) + (U40 * N50) + (V40 * N51)
    T46 = (S40 * O48) + (T40 * O49) + (U40 * O50) + (V40 * O51)
    U46 = (S40 * P48) + (T40 * P49) + (U40 * P50) + (V40 * P51)
    V46 = (S40 * Q48) + (T40 * Q49) + (U40 * Q50) + (V40 * Q51)
    S47 = (S41 * N48) + (T41 * N49) + (U41 * N50) + (V41 * N51)
    T47 = (S41 * O48) + (T41 * O49) + (U41 * O50) + (V41 * O51)
    U47 = (S41 * P48) + (T41 * P49) + (U41 * P50) + (V41 * P51)
    V47 = (S41 * Q48) + (T41 * Q49) + (U41 * Q50) + (V41 * Q51)
    S48 = (S42 * N48) + (T42 * N49) + (U42 * N50) + (V42 * N51)
    T48 = (S42 * O48) + (T42 * O49) + (U42 * O50) + (V42 * O51)
    U48 = (S42 * P48) + (T42 * P49) + (U42 * P50) + (V42 * P51)
    V48 = (S42 * Q48) + (T42 * Q49) + (U42 * Q50) + (V42 * Q51)
    ## R 0-3 transposed
    S51 = S45
    T51 = S46
    U51 = S47
    S52 = T45
    T52 = T46
    U52 = T47
    S53 = U45
    T53 = U46
    U53 = U47
    ## R 3-6 (spherical wrist  orietation)
    X72 = (S51 * X66) + (T51 * X67) + (U51 * X68)
    Y72 = (S51 * Y66) + (T51 * Y67) + (U51 * Y68)
    Z72 = (S51 * Z66) + (T51 * Z67) + (U51 * Z68)
    X73 = (S52 * X66) + (T52 * X67) + (U52 * X68)
    Y73 = (S52 * Y66) + (T52 * Y67) + (U52 * Y68)
    Z73 = (S52 * Z66) + (T52 * Z67) + (U52 * Z68)
    X74 = (S53 * X66) + (T53 * X67) + (U53 * X68)
    Y74 = (S53 * Y66) + (T53 * Y67) + (U53 * Y68)
    Z74 = (S53 * Z66) + (T53 * Z67) + (U53 * Z68)
    ## WRIST ORENTATION
    R7 = math.degrees(math.atan2(Z73, Z72))
    R8 = math.degrees(math.atan2(+math.sqrt(1 - Z74 ** 2), Z74))
    if (Y74 < 0):
        R9 = math.degrees(math.atan2(-Y74, X74)) - 180
    else:
        R9 = math.degrees(math.atan2(-Y74, X74)) + 180
    S7 = math.degrees(math.atan2(-Z73, -Z72))
    S8 = math.degrees(math.atan2(-math.sqrt(1 - Z74 ** 2), Z74))
    if (Y74 < 0):
        S9 = math.degrees(math.atan2(Y74, -X74)) + 180
    else:
        S9 = math.degrees(math.atan2(Y74, -X74)) - 180
    if (V8 == "F"):
        Q8 = R8
    else:
        Q8 = S8
    if (Q8 > 0):
        Q7 = R7
    else:
        Q7 = S7
    if (Q8 < 0):
        Q9 = S9
    else:
        Q9 = R9
    ## FINAL OUTPUT
    angle[1] = Q4
    angle[2] = Q5
    angle[3] = Q6
    angle[4] = Q7
    angle[5] = Q8
    angle[6] = Q9
    return angle


# 命令解释器
def EI(command, ser):
    time_start = time.time()
    # 全局变量声明
    global speed
    global ACCdur
    global ACCspeed
    global DECdur
    global DECspeed

    global XcurPos
    global YcurPos
    global ZcurPos
    global RxcurPos
    global RycurPos
    global RzcurPos
    # 命令处理
    x = command.split()
    if (x[0] == "calRobot"):
        speed = 50
        J1caldrive = "0"
        J2caldrive = "0"
        J3caldrive = "1"
        J4caldrive = "0"
        J5caldrive = "0"
        J6caldrive = "1"
        J1step = str(JStepLim[1])
        J2step = str(JStepLim[2])
        J3step = str(JStepLim[3])
        J4step = str(JStepLim[4])
        J5step = str(JStepLim[5])
        J6step = str(JStepLim[6])
        # time_start = time.time()
        command = "LL" + "A" + J1caldrive + J1step + "B" + J2caldrive + J2step + "C" + J3caldrive + J3step + "D" + J4caldrive + J4step + "E" + J5caldrive + J5step + "F" + J6caldrive + J6step + "S" + str(
            speed) + "\n"
        ser.write(command.encode())
        ser.flushInput()
        # time_end = time.time()
        # print('time cost', time_end - time_start, 's')
        calvalue = ser.read()

        if (calvalue == b'P'):
            print(calvalue)
            pass
        else:
            if (calvalue == b'F'):
                print("CALIBRATION FAILED")
            else:
                print("NO CAL FEEDBACK FROM ARDUINO")
        J1caldrive1 = "1"
        J2caldrive1 = "1"
        J3caldrive1 = "0"
        J4caldrive1 = "1"
        J5caldrive1 = "1"
        J6caldrive1 = "0"
        command = "MJA" + J1caldrive1 + "500" + "B" + J2caldrive1 + "500" + "C" + J3caldrive1 + "500" + "D" + J4caldrive1 + "500" + "E" + J5caldrive1 + "500" + "F" + J6caldrive1 + "500" + "S15G10H10I10K10" + "\n"
        ser.write(command.encode())
        ser.flushInput()
        speed = 8
        time.sleep(2.5)
        # 第二次
        command = "LL" + "A" + J1caldrive + J1step + "B" + J2caldrive + J2step + "C" + J3caldrive + J3step + "D" + J4caldrive + J4step + "E" + J5caldrive + J5step + "F" + J6caldrive + J6step + "S" + str(
            speed) + "\n"
        ser.write(command.encode())
        ser.flushInput()
        calvalue = ser.read()
        if (calvalue == b'P'):
            for i in range(6):
                if (i + 1 == 3 or i + 1 == 4):
                    JAngCur[i + 1] = JPosAngLim[i + 1]
                else:
                    JAngCur[i + 1] = JNegAngLim[i + 1]
            # CalcFwdKin()
            myCalcFwdKin()
        else:
            if (calvalue == b'F'):
                print("CALIBRATION FAILED")
            else:
                print("NO CAL FEEDBACK FROM ARDUINO")
        speed = 50
    elif (x[0] == "mj"):
        Jnum = int(x[1])
        Jflag = chr(ord('A') + Jnum - 1)
        Jdir = int(x[2])
        angle = float(x[3])
        if (len(x) == 9):
            speed = int(x[4])
            ACCdur = int(x[5])
            ACCspeed = int(x[6])
            DECdur = int(x[7])
            DECspeed = int(x[8])
        # 先判断移动后的角度
        if (Jnum == 3 or Jnum == 6):
            if (Jdir == 1):
                JNewAng = JAngCur[Jnum] - angle
            else:
                JNewAng = JAngCur[Jnum] + angle
        else:
            if (Jdir == 1):
                JNewAng = JAngCur[Jnum] + angle
            else:
                JNewAng = JAngCur[Jnum] - angle

        if (JNewAng < JNegAngLim[Jnum] or JNewAng > JPosAngLim[Jnum]):
            print("超出限制！")
        else:
            JAngCur[Jnum] = JNewAng
            # CalcFwdKin()
            myCalcFwdKin()
            Jstep = angle / JDegPerStep[Jnum]
            command = "MJ" + Jflag + str(Jdir) + str(Jstep) + "S" + str(speed) + "G" + str(ACCdur) + "H" + str(
                ACCspeed) + "I" + str(DECdur) + "K" + str(DECspeed) + "\n"
            ser.write(command.encode())
            ser.flushInput()
            time.sleep(.01)
            ser.read()
    elif (x[0] == "ssp"):
        if (x[1] == "S"):
            speed = int(x[2])
        elif (x[1] == "G"):
            ACCdur = int(x[2])
        elif (x[1] == "H"):
            ACCspeed = int(x[2])
        elif (x[1] == "I"):
            DECdur = int(x[2])
        elif (x[1] == "K"):
            DECspeed = int(x[2])
    elif (x[0] == "gsp"):
        print("speed:", speed, "\nACCdur:", ACCdur, "\nACCspeed:", ACCspeed, "\nDECdur:", DECdur, "\nDECspeed:",
              DECspeed)
    elif (x[0] == "gas"):
        print("J1:", JAngCur[1], "\nJ2:", JAngCur[2], "\nJ3:", JAngCur[3], "\nJ4:", JAngCur[4], "\nJ5:", JAngCur[5],
              "\nJ6:", JAngCur[6])
    elif (x[0] == "wa"):
        Jnum = int(x[1])
        angle = int(x[2])
        Jflag = chr(ord('A') + Jnum - 1)
        if (angle < JNegAngLim[Jnum] or angle > JPosAngLim[Jnum]):
            print("超出限制！")
            return
        if (len(x) == 8):
            speed = int(x[3])
            ACCdur = int(x[4])
            ACCspeed = int(x[5])
            DECdur = int(x[6])
            DECspeed = int(x[7])
        if (angle < JAngCur[Jnum]):
            Jstep = (JAngCur[Jnum] - angle) / JDegPerStep[Jnum]
            if (Jnum == 4 or Jnum == 6):
                Jdir = 1
            else:
                Jdir = 0
            JAngCur[Jnum] = angle
            # CalcFwdKin()
            myCalcFwdKin()

            command = "MJ" + Jflag + str(Jdir) + str(Jstep) + "S" + str(speed) + "G" + str(ACCdur) + "H" + str(
                ACCspeed) + "I" + str(DECdur) + "K" + str(DECspeed) + "\n"
            time_end = time.time()
            print('time cost', time_end - time_start, 's')
            ser.write(command.encode())

            ser.flushInput()
            time_end = time.time()
            print('time cost', time_end - time_start, 's')
            time.sleep(.01)
            ser.read()
            time_end = time.time()
            print('time cost', time_end - time_start, 's')
        elif (angle > JAngCur[Jnum]):
            Jstep = (angle - JAngCur[Jnum]) / JDegPerStep[Jnum]
            if (Jnum == 4 or Jnum == 6):
                Jdir = 0
            else:
                Jdir = 1
            JAngCur[Jnum] = angle
            # CalcFwdKin()
            myCalcFwdKin()
            command = "MJ" + Jflag + str(Jdir) + str(Jstep) + "S" + str(speed) + "G" + str(ACCdur) + "H" + str(
                ACCspeed) + "I" + str(DECdur) + "K" + str(DECspeed) + "\n"
            ser.write(command.encode())
            ser.flushInput()
            time.sleep(.01)
            ser.read()
        else:
            pass
    elif (x[0] == "was"):
        angle = [0.0] * 7
        flag = 0
        for i in range(6):
            angle[i + 1] = float(x[i + 1])
            if (angle[i + 1] < JNegAngLim[i + 1] or angle[i + 1] > JPosAngLim[i + 1]):
                print("超出限制！")
                flag = 1
                break
        if (flag):
            return
        Jflag = [None, 'A', 'B', 'C', 'D', 'E', 'F']
        Jstep = [0] * 7
        Jdir = [0] * 7

        if (len(x) == 12):
            speed = int(x[7])
            ACCdur = int(x[8])
            ACCspeed = int(x[9])
            DECdur = int(x[10])
            DECspeed = int(x[11])
        for i in range(6):
            if (angle[i + 1] < float(JAngCur[i + 1])):
                Jstep[i + 1] = (JAngCur[i + 1] - angle[i + 1]) / JDegPerStep[i + 1]
                if (i + 1 == 4 or i + 1 == 6):
                    Jdir[i + 1] = 1
                else:
                    Jdir[i + 1] = 0
                JAngCur[i + 1] = angle[i + 1]
            elif (angle[i + 1] >= float(JAngCur[i + 1])):
                Jstep[i + 1] = (angle[i + 1] - JAngCur[i + 1]) / JDegPerStep[i + 1]
                if (i + 1 == 4 or i + 1 == 6):
                    Jdir[i + 1] = 0
                else:
                    Jdir[i + 1] = 1
                JAngCur[i + 1] = angle[i + 1]
        # CalcFwdKin()
        myCalcFwdKin()
        command = "MJ" + Jflag[1] + str(Jdir[1]) + str(Jstep[1]) \
                  + Jflag[2] + str(Jdir[2]) + str(Jstep[2]) \
                  + Jflag[3] + str(Jdir[3]) + str(Jstep[3]) \
                  + Jflag[4] + str(Jdir[4]) + str(Jstep[4]) \
                  + Jflag[5] + str(Jdir[5]) + str(Jstep[5]) \
                  + Jflag[6] + str(Jdir[6]) + str(Jstep[6]) \
                  + "S" + str(speed) + "G" + str(ACCdur) + "H" + str(
            ACCspeed) + "I" + str(DECdur) + "K" + str(DECspeed) + "\n"
        ser.write(command.encode())
        ser.flushInput()
        time.sleep(.01)
        ser.read()
    elif (x[0] == "gcs"):
        print("X:", XcurPos, "\nY:", YcurPos, "\nZ:", ZcurPos, "\nRX:", RxcurPos, "\nRY:", RycurPos, "\nRZ:", RzcurPos)
    elif (x[0] == "mc"):
        zifu = ['x', 'y', 'z', 'w', 'p', 'r']
        VectorStep = float(x[2])
        if (x[1] == 'x'):
            CX = XcurPos + VectorStep
            # CX = XcurPos
            CY = YcurPos
            CZ = ZcurPos
            CRx = RxcurPos
            CRy = RycurPos
            CRz = RzcurPos
        elif (x[1] == 'y'):
            CY = YcurPos + VectorStep
            CX = XcurPos
            # CY = YcurPos
            CZ = ZcurPos
            CRx = RxcurPos
            CRy = RycurPos
            CRz = RzcurPos
        elif (x[1] == 'z'):
            CZ = ZcurPos + VectorStep
            CX = XcurPos
            CY = YcurPos
            # CZ = ZcurPos
            CRx = RxcurPos
            CRy = RycurPos
            CRz = RzcurPos
        elif (x[1] == 'w'):
            CRx = RxcurPos + VectorStep
            CX = XcurPos
            CY = YcurPos
            CZ = ZcurPos
            # CRx = RxcurPos
            CRy = RycurPos
            CRz = RzcurPos
        elif (x[1] == 'p'):
            CRy = RycurPos + VectorStep
            CX = XcurPos
            CY = YcurPos
            CZ = ZcurPos
            CRx = RxcurPos
            # CRy = RycurPos
            CRz = RzcurPos
        elif (x[1] == 'r'):
            CRz = RzcurPos + VectorStep
            CX = XcurPos
            CY = YcurPos
            CZ = ZcurPos
            CRx = RxcurPos
            CRy = RycurPos
            # CRz = RzcurPos
        angle = CalcRevKin(CX, CY, CZ, CRx, CRy, CRz)
        # print(angle)
        # 判断是否可达
        flag = 0
        for i in range(6):
            if (angle[i + 1] < JNegAngLim[i + 1] or angle[i + 1] > JPosAngLim[i + 1]):
                flag = 1
                break
        if (flag):
            print("超出限制！")
            return
        XcurPos = CX
        YcurPos = CY
        ZcurPos = CZ
        RxcurPos = CRx
        RycurPos = CRy
        RzcurPos = CRz
        # 移动
        Jflag = [None, 'A', 'B', 'C', 'D', 'E', 'F']
        Jstep = [0] * 7
        Jdir = [0] * 7
        for i in range(6):
            if (angle[i + 1] < float(JAngCur[i + 1])):
                Jstep[i + 1] = (JAngCur[i + 1] - angle[i + 1]) / JDegPerStep[i + 1]
                if (i + 1 == 4 or i + 1 == 6):
                    Jdir[i + 1] = 1
                else:
                    Jdir[i + 1] = 0
                JAngCur[i + 1] = angle[i + 1]
            elif (angle[i + 1] >= float(JAngCur[i + 1])):
                Jstep[i + 1] = (angle[i + 1] - JAngCur[i + 1]) / JDegPerStep[i + 1]
                if (i + 1 == 4 or i + 1 == 6):
                    Jdir[i + 1] = 0
                else:
                    Jdir[i + 1] = 1
                JAngCur[i + 1] = angle[i + 1]
        command = "MJ" + Jflag[1] + str(Jdir[1]) + str(Jstep[1]) \
                  + Jflag[2] + str(Jdir[2]) + str(Jstep[2]) \
                  + Jflag[3] + str(Jdir[3]) + str(Jstep[3]) \
                  + Jflag[4] + str(Jdir[4]) + str(Jstep[4]) \
                  + Jflag[5] + str(Jdir[5]) + str(Jstep[5]) \
                  + Jflag[6] + str(Jdir[6]) + str(Jstep[6]) \
                  + "S" + str(speed) + "G" + str(ACCdur) + "H" + str(
            ACCspeed) + "I" + str(DECdur) + "K" + str(DECspeed) + "\n"
        ser.write(command.encode())
        ser.flushInput()
        time.sleep(.01)
        ser.read()

    elif (x[0] == "wc"):
        zifu = ['x', 'y', 'z', 'w', 'p', 'r']
        VectorStep = float(x[2])
        if (x[1] == 'x'):
            CX = VectorStep
            # CX = XcurPos
            CY = YcurPos
            CZ = ZcurPos
            CRx = RxcurPos
            CRy = RycurPos
            CRz = RzcurPos
        elif (x[1] == 'y'):
            CY = VectorStep
            CX = XcurPos
            # CY = YcurPos
            CZ = ZcurPos
            CRx = RxcurPos
            CRy = RycurPos
            CRz = RzcurPos
        elif (x[1] == 'z'):
            CZ = VectorStep
            CX = XcurPos
            CY = YcurPos
            # CZ = ZcurPos
            CRx = RxcurPos
            CRy = RycurPos
            CRz = RzcurPos
        elif (x[1] == 'w'):
            CRx = VectorStep
            CX = XcurPos
            CY = YcurPos
            CZ = ZcurPos
            # CRx = RxcurPos
            CRy = RycurPos
            CRz = RzcurPos
        elif (x[1] == 'p'):
            CRy = VectorStep
            CX = XcurPos
            CY = YcurPos
            CZ = ZcurPos
            CRx = RxcurPos
            # CRy = RycurPos
            CRz = RzcurPos
        elif (x[1] == 'r'):
            CRz = VectorStep
            CX = XcurPos
            CY = YcurPos
            CZ = ZcurPos
            CRx = RxcurPos
            CRy = RycurPos
            # CRz = RzcurPos
        angle = CalcRevKin(CX, CY, CZ, CRx, CRy, CRz)
        # print(angle)
        # 判断是否可达
        flag = 0
        for i in range(6):
            if (angle[i + 1] < JNegAngLim[i + 1] or angle[i + 1] > JPosAngLim[i + 1]):
                flag = 1
                break
        if (flag):
            print("超出限制！")
            return
        XcurPos = CX
        YcurPos = CY
        ZcurPos = CZ
        RxcurPos = CRx
        RycurPos = CRy
        RzcurPos = CRz
        # 移动
        Jflag = [None, 'A', 'B', 'C', 'D', 'E', 'F']
        Jstep = [0] * 7
        Jdir = [0] * 7
        for i in range(6):
            if (angle[i + 1] < float(JAngCur[i + 1])):
                Jstep[i + 1] = (JAngCur[i + 1] - angle[i + 1]) / JDegPerStep[i + 1]
                if (i + 1 == 4 or i + 1 == 6):
                    Jdir[i + 1] = 1
                else:
                    Jdir[i + 1] = 0
                JAngCur[i + 1] = angle[i + 1]
            elif (angle[i + 1] >= float(JAngCur[i + 1])):
                Jstep[i + 1] = (angle[i + 1] - JAngCur[i + 1]) / JDegPerStep[i + 1]
                if (i + 1 == 4 or i + 1 == 6):
                    Jdir[i + 1] = 0
                else:
                    Jdir[i + 1] = 1
                JAngCur[i + 1] = angle[i + 1]
        command = "MJ" + Jflag[1] + str(Jdir[1]) + str(Jstep[1]) \
                  + Jflag[2] + str(Jdir[2]) + str(Jstep[2]) \
                  + Jflag[3] + str(Jdir[3]) + str(Jstep[3]) \
                  + Jflag[4] + str(Jdir[4]) + str(Jstep[4]) \
                  + Jflag[5] + str(Jdir[5]) + str(Jstep[5]) \
                  + Jflag[6] + str(Jdir[6]) + str(Jstep[6]) \
                  + "S" + str(speed) + "G" + str(ACCdur) + "H" + str(
            ACCspeed) + "I" + str(DECdur) + "K" + str(DECspeed) + "\n"
        ser.write(command.encode())
        ser.flushInput()
        time.sleep(.01)
        ser.read()
    elif (x[0] == "wcs"):

        CX = float(x[1])
        CY = float(x[2])
        CZ = float(x[3])
        CRx = float(x[4])
        CRy = float(x[5])
        CRz = float(x[6])
        # CRz = RzcurPos
        angle = CalcRevKin(CX, CY, CZ, CRx, CRy, CRz)
        # print(angle)
        # 判断是否可达
        flag = 0
        for i in range(6):
            if (angle[i + 1] < JNegAngLim[i + 1] or angle[i + 1] > JPosAngLim[i + 1]):
                flag = 1
                break
        if (flag):
            print("超出限制！")
            return
        XcurPos = CX
        YcurPos = CY
        ZcurPos = CZ
        RxcurPos = CRx
        RycurPos = CRy
        RzcurPos = CRz
        # 移动
        Jflag = [None, 'A', 'B', 'C', 'D', 'E', 'F']
        Jstep = [0] * 7
        Jdir = [0] * 7
        for i in range(6):
            if (angle[i + 1] < float(JAngCur[i + 1])):
                Jstep[i + 1] = (JAngCur[i + 1] - angle[i + 1]) / JDegPerStep[i + 1]
                if (i + 1 == 4 or i + 1 == 6):
                    Jdir[i + 1] = 1
                else:
                    Jdir[i + 1] = 0
                JAngCur[i + 1] = angle[i + 1]
            elif (angle[i + 1] >= float(JAngCur[i + 1])):
                Jstep[i + 1] = (angle[i + 1] - JAngCur[i + 1]) / JDegPerStep[i + 1]
                if (i + 1 == 4 or i + 1 == 6):
                    Jdir[i + 1] = 0
                else:
                    Jdir[i + 1] = 1
                JAngCur[i + 1] = angle[i + 1]
        command = "MJ" + Jflag[1] + str(Jdir[1]) + str(Jstep[1]) \
                  + Jflag[2] + str(Jdir[2]) + str(Jstep[2]) \
                  + Jflag[3] + str(Jdir[3]) + str(Jstep[3]) \
                  + Jflag[4] + str(Jdir[4]) + str(Jstep[4]) \
                  + Jflag[5] + str(Jdir[5]) + str(Jstep[5]) \
                  + Jflag[6] + str(Jdir[6]) + str(Jstep[6]) \
                  + "S" + str(speed) + "G" + str(ACCdur) + "H" + str(
            ACCspeed) + "I" + str(DECdur) + "K" + str(DECspeed) + "\n"
        ser.write(command.encode())
        ser.flushInput()
        time.sleep(.01)
        ser.read()

    elif (x[0] == "TM"):

        command = "TM"+x[1]+"\n"
        ser.write(command.encode())
        ser.flushInput()
        time.sleep(0)
        echo = ser.readline()
        print(echo)


    # elif (x[0] == "q"):
    #     print("按q停止！")
    #     return
    # time_end = time.time()
    # print('time cost', time_end - time_start, 's')


if __name__ == '__main__':
    # 串口通信
    port = "COM4"
    baud = 115200
    ser = serial.Serial(port, baud)


    # 发送命令
    while True:
        command = input("> ")
        if(command[0] == 'q'):
            break
        EI(command, ser)

    # 退出循环
    ser.close()
    f.close()
