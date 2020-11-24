import threading
import time
import struct
import serial
import binascii
from datetime import datetime
class Config:
    # 端口号
    serialPort = 'COM3' 
    # 波特率 
    baudRate = 9600  
    #在传感器数据中，最小的包长度是11个字节
    minPackageLen = 11
#传感器数据读取类，全部读取数据累加到receiveBuffer这个字节数组中
class SensorReader:
    def __init__(self):
        self.port = serial.Serial(Config.serialPort, Config.baudRate)
        self.port.close()
        if not self.port.isOpen():
            self.port.open()
        self.receiveBuffer = bytearray()
        self.working = False

    #打开
    def open(self):
        if not self.port.isOpen():
            self.port.open()

    #关闭
    def close(self):
        self.port.close()

    #发送数据
    def send(self,data):
        self.port.write(data)

    #接收数据
    def receive(self):        
        while self.working:
            #休眠一个微小的时间，可以避免无谓的CPU占用，在windows上实验时直接从12%下降到接近于0%
            time.sleep(0.001)
            count = self.port.inWaiting()
            if count > 0:
                s = self.port.read(count)
                self.receiveBuffer+= s                

    #开始工作
    def start(self):
        #开始数据读取线程
        t = threading.Thread(target=self.receive)
        #将当前线程设为子线程t的守护线程，这样一来，当前线程结束时会强制子线程结束
        t.setDaemon(True)
        self.working = True
        t.start()

    #停止工作
    def stop(self):
        self.working = False
#数据解析类，负责分析SensorReader
        
class DataParser:
    def __init__(self,sensorReader):
        self.r = sensorReader     
        self.working = False
        self.TimeStart = datetime.now()
        self.iniVariable()

    #流逝的毫秒数，返回浮点数
    def elapseMilliSeconds(self):
        now = datetime.now()
        seconds = time.mktime(now.timetuple()) - time.mktime(self.TimeStart.timetuple())
        #微秒数的差值 
        microSeconds = now.microsecond - self.TimeStart.microsecond
        return seconds* 1000+ microSeconds/1000.0

    #流逝的秒数，返回浮点数
    def elapseSeconds(self):
        return self.elapseMilliSeconds()/1000
   
    #在缓冲数据中找到第一个包的起始位置
    def findFirstPackage(self,buffer):
        i = 0
        while True:
            if buffer[i] == 0x55 and (buffer[i+1] & 0x50) == 0x50 :
                return i
            if i+2 >= len(buffer):
                return -1
            i+=1

    #处理数据
    def handle(self):        
        #处理接收到的数据
        while self.working:               
            
            #显示当前收到的数据
            dataLen = len(self.r.receiveBuffer)
            if dataLen >= Config.minPackageLen:
                #去掉第1个包头前的数据
                headerPos = self.findFirstPackage(self.r.receiveBuffer)           
                
                if headerPos >= 0 :
                    if headerPos > 0:
                        self.r.receiveBuffer[0:headerPos]=b''
                    #取 Config.minPackageLen 整数倍长度的数据
                    if dataLen-headerPos >= Config.minPackageLen : 
                        packageCount =   int((dataLen-headerPos)/Config.minPackageLen)
                        if packageCount > 0:
                            cutLen = packageCount*Config.minPackageLen                        
                            
                            temp = self.r.receiveBuffer[0:cutLen]
                            #按16进制字符串的形式显示收到的内容
                            hexStr = str(binascii.b2a_hex(temp))                     
                           
                            self.r.receiveBuffer[0:cutLen]=b''
                            #在窗口的文本框中显示数据
                            #self.u.showData(text) 
                            #解析数据,逐个数据包进行解析
                            for i in range(packageCount):                                                    
                                beginIdx =int(i*Config.minPackageLen)
                                endIdx   =int(i*Config.minPackageLen+Config.minPackageLen)                            
                                byteTemp = temp[beginIdx:endIdx]
                                #校验和通过了的数据包才进行解析
                                if self.sbSumCheck(byteTemp):
                                    self.decodeData(byteTemp) 
                                              
            time.sleep(0.005)

    #初始化解析丰关的变量
    def iniVariable(self):           
        self.ChipTime  = [0,0,0,0,0,0,0]
        self.a = [0,0,0,0]        
        self.w = [0,0,0,0]
        self.Angle = [0,0,0,0]
        self.h = [0,0,0,0]
        self.Port = [0,0,0,0]
        self.LastTime = [0,0,0,0,0,0,0,0,0,0]
        self.Temperature = 0
        self.Pressure = 0
        self.Altitude = 0
        self.GroundVelocity = 0
        self.GPSYaw = 0
        self.GPSHeight = 0
        self.Longitude = 0
        self.Latitude = 0

    #解码包中的数据
    def decodeData(self,byteTemp):        
        #记录当前的相对时间
        TimeElapse = self.elapseSeconds();
        #将8个字节的数据解析成4个短整型
        Data = list(struct.unpack("hhhh", byteTemp[2:10]) )

        if byteTemp[1] == 0x50:
            self.ChipTime[0] = (2000 + byteTemp[2])
            self.ChipTime[1] = byteTemp[3]
            self.ChipTime[2] = byteTemp[4]
            self.ChipTime[3] = byteTemp[5]
            self.ChipTime[4] = byteTemp[6]
            self.ChipTime[5] = byteTemp[7]
            self.ChipTime[6] = struct.unpack("h", byteTemp[8:10])[0] 

        if byteTemp[1] == 0x51:
            self.Temperature = Data[3] / 100.0
            Data[0] = Data[0] / 32768.0 * 16
            Data[1] = Data[1] / 32768.0 * 16
            Data[2] = Data[2] / 32768.0 * 16

            self.a[0] = Data[0]
            self.a[1] = Data[1]
            self.a[2] = Data[2]
            self.a[3] = Data[3]
            if ((TimeElapse - self.LastTime[1]) < 0.1):
                return
            self.LastTime[1] = TimeElapse

        if byteTemp[1] == 0x52:
            self.Temperature = Data[3] / 100.0
            Data[0] = Data[0] / 32768.0 * 2000
            Data[1] = Data[1] / 32768.0 * 2000
            Data[2] = Data[2] / 32768.0 * 2000
            self.w[0] = Data[0]
            self.w[1] = Data[1]
            self.w[2] = Data[2]
            self.w[3] = Data[3]
            if ((TimeElapse - self.LastTime[2]) < 0.1):
                return
            self.LastTime[2] = TimeElapse

        if byteTemp[1] == 0x53:
            self.Temperature = Data[3] / 100.0
            Data[0] = Data[0] / 32768.0 * 180
            Data[1] = Data[1] / 32768.0 * 180
            Data[2] = Data[2] / 32768.0 * 180
            self.Angle[0] = Data[0]
            self.Angle[1] = Data[1]
            self.Angle[2] = Data[2]
            self.Angle[3] = Data[3]
            if ((TimeElapse - self.LastTime[3]) < 0.1):
                return
            self.LastTime[3] = TimeElapse

        if byteTemp[1] == 0x54:
            self.Temperature = Data[3] / 100.0
            self.h[0] = Data[0]
            self.h[1] = Data[1]
            self.h[2] = Data[2]
            self.h[3] = Data[3]
            if ((TimeElapse - self.LastTime[4]) < 0.1):
                return
            self.LastTime[4] = TimeElapse

        if byteTemp[1] == 0x55:
            self.Port[0] = Data[0]
            self.Port[1] = Data[1]
            self.Port[2] = Data[2]
            self.Port[3] = Data[3]

        if byteTemp[1] == 0x56:
            self.Pressure = struct.unpack("i", byteTemp[2:6])[0]
            self.Altitude = struct.unpack("i", byteTemp[6:10])[0] / 100.0

        if byteTemp[1] == 0x57:            
            self.Longitude = struct.unpack("i", byteTemp[2:6])[0]
            self.Latitude  = struct.unpack("i", byteTemp[6:10])[0]

        if byteTemp[1] == 0x58:
            self.GPSHeight = struct.unpack("h", byteTemp[2:4])[0] / 10.0
            self.GPSYaw = struct.unpack("h", byteTemp[4:6])[0] / 10.0
            self.GroundVelocity = struct.unpack("h", byteTemp[6:8])[0] / 1e3

       
       

    #检查校验和
    def sbSumCheck(self,byteTemp):
        if (((byteTemp[0]+byteTemp[1]+byteTemp[2]+byteTemp[3]+byteTemp[4]+byteTemp[5]+byteTemp[6]+byteTemp[7]+byteTemp[8]+byteTemp[9])&0xff)==byteTemp[10]) :
            #print('sum check ok!')
            return True
        else:
            print('sum check false!')
            return False

    #开始工作
    def start(self):
        #开启数据解析线程
        t = threading.Thread(target=self.handle)
        #将当前线程设为子线程t的守护线程，这样一来，当前线程结束时会强制子线程结束
        t.setDaemon(True)
        self.working = True
        t.start()

    #停止工作
    def stop(self):
        self.working = False  
if __name__=="__main__":
    s=SensorReader()
    s.start()
    p=DataParser(s)
    p.start()
    while True:
        time.sleep(1)
        print(p.a)#加速度
        print(p.w)#角速度
        print(p.Angle)#角度
        print(p.Temperature)#温度
        
    