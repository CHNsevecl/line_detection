from machine import Pin,PWM
import time 

#PWM标注

Motor_PWMA_Flag = None
Motor_PWMB_Flag = None
Motor_STBY_Flag = None

#电机控制引脚（电机行为引脚）
#A为右
Motor_AIN_1 = Pin(18,Pin.OUT)
Motor_AIN_2 = Pin(17,Pin.OUT)
#B为左
Motor_BIN_1 = Pin(20,Pin.OUT)
Motor_BIN_2 = Pin(21,Pin.OUT)
#stby配置
Pin_stby = Pin(19,Pin.OUT,value=0)

class CarControl:
    def __init__(self):
        
        self.PWMB = PWM(22,freq=10000,duty_u16=0)
        self.PWMA = PWM(16,freq=10000,duty_u16=0)
        Pin_stby.value(1)
        pass
    
    def Motor_standby(self,situation =0):
        Pin_stby.value(situation)

    def Motor_stop(self):#滑行停止
        Motor_AIN_1.value(0)
        Motor_AIN_2.value(0)
        self.PWMA.duty_u16(10000)

        Motor_BIN_1.value(0)
        Motor_BIN_2.value(0)
        self.PWMB.duty_u16(10000)

    def Motor_forward(self,speedA = 15000,speedB = 15000):
        Motor_AIN_1.value(0)
        Motor_AIN_2.value(1)
        self.PWMA.duty_u16(speedA)

        Motor_BIN_1.value(1)
        Motor_BIN_2.value(0)
        self.PWMB.duty_u16(speedB)

    def Motor_backward(self):
        Motor_AIN_1.value(0)
        Motor_AIN_2.value(1)
        self.PWMA.duty_u16(10000)

        Motor_BIN_1.value(0)
        Motor_BIN_2.value(1)
        self.PWMB.duty_u16(10000)

    def Motor_brake(self,brake_time =0.1):#制动
        Motor_AIN_1.value(1)
        Motor_AIN_2.value(0)
        self.PWMA.duty_u16(0)

        Motor_BIN_1.value(1)
        Motor_BIN_2.value(0)
        self.PWMB.duty_u16(0)
        time.sleep(brake_time)
    
    def finish(self):
        self.PWMA.deinit()
        self.PWMB.deinit()
        self.Motor_standby(0)
