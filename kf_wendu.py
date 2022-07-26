from matplotlib import pyplot
import math
import random
 
lastTimePredVal = 0  # 上次估计值
lastTimePredCovVal = 0.1  # 上次估计协方差
lastTimeRealCovVal = 0.1  # 上次实际协方差
kg = 0.0 #卡尔曼增益
 
# val: 本次测量值
def kalman(val):
    #python中如果若想在函数内部对函数外的变量进行操作，就需要在函数内部声明其为global。
    global lastTimePredVal  # 上次估计值
    global lastTimePredCovVal  # 上次估计协方差
    global lastTimeRealCovVal  # 上次实际协方差
    global kg
 
    currRealVal = val  # 本次实际值
    currPredCovVal = lastTimePredCovVal  # 本次估计协方差值
    currRealCovVal = lastTimeRealCovVal  # 本次实际协方差值
 
    # 计算本次估计值，并更新保留上次预测值的变量
    currPredVal = lastTimePredVal + kg * (currRealVal - lastTimePredVal)
    lastTimePredVal = currPredVal
 
    #计算卡尔曼增益
    kg = math.sqrt(math.pow(lastTimePredCovVal, 2) / (math.pow(lastTimePredCovVal, 2) + math.pow(lastTimeRealCovVal, 2)))
 
    # 计算下次估计和实际协方差
    lastTimePredCovVal = math.sqrt(1.0 - kg) * currPredCovVal
    lastTimeRealCovVal = math.sqrt(1.0 - kg) * currRealCovVal
 
    # 返回本次的估计值,也就是滤波输出值
    return currPredVal
 
 
if __name__ == "__main__":
    realTemp = []                   # 真实温度
    predTemp = []                   # 预测温度
 
    # 生成50个真实温度，20度到23度之间
    for i in range(50):
        realTemp.append(random.uniform(20, 23))
 
    # 卡尔曼滤波
    for t in realTemp:
        predVal = kalman(t)
        predTemp.append(predVal)
 
    # 绘制真实温度和预测温度折线图
    pyplot.figure()
    pyplot.plot(predTemp, label='kf_predict_temp',c='r')
    pyplot.plot(realTemp, label='real_temp',c='b')
    pyplot.tick_params(axis='x', which='major', labelsize=int(len(predTemp)/10))
    pyplot.xlabel('Count')
    pyplot.ylabel('Temperature')
    pyplot.legend(loc='lower right')
    pyplot.savefig('kf_wendu.png')
    pyplot.show()
