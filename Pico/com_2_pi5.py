from machine import UART, Pin
import time

class UARTController:
    def __init__(self, id=1, baudrate=115200, tx_pin=4, rx_pin=5):
        self.uart = UART(id, baudrate, tx = Pin(tx_pin), rx = Pin(rx_pin))
        self.led = Pin(25, Pin.OUT)
        # 创建一个缓冲区，用来存放不完整的数据
        self.buffer = ""
        print("串口控制器已启动")

    def get_command(self):
        """
        核心函数：从缓冲区读取并解析完整的指令列表
        返回: list (例如 ['1', '100']) 或 默认值 ['0', '0']
        """
        # 1. 将串口新收到的数据添加到缓冲区
        if self.uart.any():
            raw_data = self.uart.read()
            if raw_data is not None:
                try:
                    self.buffer += raw_data.decode()
                except UnicodeError:
                    # 如果解码出错，清空缓冲区，防止错误累积
                    self.buffer = ""

        # 2. 检查缓冲区里是否有完整的指令（以换行符结尾）
        if '\n' in self.buffer:
            # 按换行符分割，得到所有完整的指令行
            lines = self.buffer.split('\n')
            
            # 最后一行可能是不完整的（还没收到换行符），把它放回缓冲区
            self.buffer = lines[-1]
            
            # 处理所有已经完整的指令行
            for line in lines[:-1]:
                line = line.strip() # 去除首尾空格
                if line: # 确保不是空行
                    # 将 "1 100" 这样的字符串分割成列表 ['1', '100']
                    return line.split()

        # 如果没有完整的指令，返回默认值
        return None

def main():
    com = UARTController()
    print("等待指令...")

    while True:
        # 获取解析好的列表
        cmd = com.get_command()
        if cmd:
            print(cmd)
            
        time.sleep(0.01)

if __name__ == "__main__":
    main()