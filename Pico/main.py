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
                print(lst)
                
                if lst[0] =="TURN_RIGHT":
                    while lst[0] == "TURN_RIGHT":
                        lst = com.get_command()
                        while lst == None:
                            lst = com.get_command()
                        car.Motor_forward(speedA=3000+int(lst[1]),speedB = 3000)
                        print("right")
                    car.Motor_brake()    
                elif lst[0] == "TURN_LEFT":
                    while lst[0] == "TURN_LEFT":
                        lst = com.get_command()
                        while lst == None:
                            lst = com.get_command()
                        car.Motor_forward(speedB=3000+int(lst[1]),speedA = 3000)
                        print("left")
                    car.Motor_brake()
                    
                elif lst[0] == "BRAKE":
                    car.Motor_brake()
                    break
                elif int(lst[1]) == 0:
                    car.Motor_forward()
                    #print("前进")
                elif int(lst[0]) > 0:
                    car.Motor_forward(speedA=15000+int(lst[1]))
                    #print("右转")
                elif int(lst[0]) < 0:
                    car.Motor_forward(speedB=15000+int(lst[1]))
                    #print("左转")
                else:
                    car.Motor_brake()
    except Exception as e:
        print(f"发生错误: {e}")
        print(lst)
        led.value(0)  # 确保在发生错误时关闭LED
        car.finish()
if __name__ == "__main__":
    main()

