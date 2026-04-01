from machine import UART, Pin
import time

# 1. 初始化串口 (TX=4, RX=5)
uart = UART(1, baudrate=115200, tx=Pin(4), rx=Pin(5))
led = Pin(25, Pin.OUT) # 板载 LED

print("等待指令...")

while True:
    # 2. 检查是否有数据
    if uart.any():
        # 3. 读取并解码 (errors='ignore' 防止乱码报错)
        raw_data = uart.read()
        try:
            if raw_data is None:
                continue

            text = raw_data.decode("utf-8").strip().lower()
            
            # 4. 简单判断
            if text == "on":
                led.value(1)
                uart.write("LED 已打开\n")
            elif text == "off":
                led.value(0)
                uart.write("LED 已关闭\n")

        except Exception as e:
            print(f"错误: {e}")
            
    time.sleep(0.01)