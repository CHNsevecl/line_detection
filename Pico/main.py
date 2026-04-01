from machine import Pin
from utime import sleep
from com_2_pi5 import UARTController
from car import CarControl

def main():
    try:
        led = Pin(25,Pin.OUT,value=1)
        car = CarControl()
        com = UARTController()  

        while 1:
            lst = com.get_command()  # 获取解析好的列

            if lst:
                
                if int(lst[1]) == 0:
                    car.Motor_forward()
                    print("前进")
                elif int(lst[0]) > 0:
                    car.Motor_forward(speedA=15000+int(lst[1]))
                    print("右转")
                elif int(lst[0]) < 0:
                    car.Motor_forward(speedB=15000+int(lst[1]))
                    print("左转")
                print(lst)
    except Exception as e:
        print(f"发生错误: {e}")
        led.value(0)  # 确保在发生错误时关闭LED
        car.finish()
if __name__ == "__main__":
    main()

