# coding: utf-8
import smbus
import math
import time
## AK8963 I2C slave address
AK8963_SLAVE_ADDRESS = 0x0C
# AK8963 Register Addresses
AK8963_ST1        =0x02
AK8963_MAGNET_OUT =0x03
AK8963_CNTL1      =0x0A
AK8963_CNTL2      =0x0B
AK8963_ASAX       =0x10

# CNTL1 Mode select
## Power down mode
AK8963_MODE_DOWN   =0x00
## One shot data output
AK8963_MODE_ONE    =0x01
## Continous data output 8Hz
AK8963_MODE_C8HZ   =0x02
## Continous data output 100Hz
AK8963_MODE_C100HZ =0x06
# Magneto Scale Select
## 14bit output
AK8963_BIT_14     =0x00
## 16bit output
AK8963_BIT_16     =0x01

XACCEL            =0x3b
YACCEL            =0x3d
ZACCEL            =0x3f  
XGYRO             =0x43
YGYRO             =0x45
ZGYRO             =0x47
RANGE_2G          =0b00
RANGE_4G          =0b01
RANGE_8G          =0b10
RANGE_16G         =0b11
REG_ACCEL_CONFIG  =0x1C
SCALE_2000DPS     =0b11
SCALE_1000DPS     =0b10
SCALE_500DPS      =0b01
SCALE_250DPS      =0b00
REG_GYRO_CONFIG   =0x1B 
TEMP_OUT          =0x41
POW_MGMT_1        =0x6b
POW_MGMT_1_2      =0x6c
INT_PIN_CFG       =0x37
REG_CONFIG        =0X1A
REG_SMP_DIV       =0X19    
class MPU:
  def __init__(self ,addr):
    self.address=addr
    self.bus = smbus.SMBus(1) # 0 or 1, bus = smbus.SMBus(1) for Revision 2 boards
    self.digits=6 #decimal digits of returned values
    #mpu configuration   
    self.accelrange=0;
    self.gyroscale=0;
    self.accelRangePerDigit=0
    self.gyroScalePerDigit=0
    
    #calibration parameters
    #AXIS OFFSET
    self.XACCEL_OFFSET=0
    self.YACCEL_OFFSET=0
    self.ZACCEL_OFFSET=0  
    self.XGYRO_OFFSET=0
    self.YGYRO_OFFSET=0
    self.ZGYRO_OFFSET=0
    #STANDART DEVIATION FOR EACH AXIS
    self.XACCEL_SIGMA=0
    self.YACCEL_SIGMA=0
    self.ZACCEL_SIGMA=0  
    self.XGYRO_SIGMA=0
    self.YGYRO_SIGMA=0
    self.ZGYRO_SIGMA=0
    #THRESHOLD FOR EACH AXIS
    
    self.XACCEL_TH=0
    self.YACCEL_TH=0
    self.ZACCEL_TH=0  
    self.XGYRO_TH=0
    self.YGYRO_TH=0
    self.ZGYRO_TH=0
    #output holding variables
    self.accel_xout=0
    self.accel_yout=0
    self.accel_zout=0
    self.gyro_xout=0
    self.gyro_yout=0
    self.gyro_zout=0
    self.mag_xout=0
    self.mag_yout=0
    self.mag_zout=0
    #previous reads
    self.p_accel_xout=0
    self.p_accel_yout=0
    self.p_accel_zout=0
    self.p_gyro_xout=0
    self.p_gyro_xout=0
    self.p_gyro_xout=0
    self.p_mag_xout=0
    self.p_mag_yout=0
    self.p_mag_zout=0   
    #magnetometer
    self.mres = 4912.0/32760.0
    self.magXcoef =1
    self.magYcoef =1
    self.magZcoef =1  
        
    self.verbose=False # ativa modo verboso para debug
 
  def read_byte(self,adr):
    return self.bus.read_byte_data(self.address, adr)

  def read_word(self,adr):
      self.high = self.bus.read_byte_data(self.address, adr)
      self.low = self.bus.read_byte_data(self.address, adr+1)
      self.val = (self.high << 8) + self.low
      return self.val
  
  def read_word_2c(self,adr):
      self.val = self.read_word(adr)
      if (self.val >= 0x8000):
          return -((65535 - self.val) + 1)
      else:
          return self.val



  def wake(self):
    self.bus.write_byte_data(self.address,POW_MGMT_1, 0)    

  def begin(self):
    self.getGyroScale()
    self.getAccelRange()
    self.wake()
    self.bus.write_byte_data(self.address,INT_PIN_CFG, 0x02)# BYPASS_EN # IMPORTANTÍSSIMO PARA PODER LER O MAGNETOMETRO    

  def setAccelRange(self,arange):
       
     if (arange==RANGE_2G)or(arange==RANGE_4G)or(arange==RANGE_8G)or(arange==RANGE_16G):   
        value=self.bus.read_byte_data(self.address,REG_ACCEL_CONFIG)
        value &= 0b11100111
        value |= (arange << 3)
        self.bus.write_byte_data(self.address, REG_ACCEL_CONFIG, value)
        self.accelrange= 2*(2**arange)
        self.accelRangePerDigit=(self.accelrange*9.80665/32767) 
        if self.verbose:
          print("Accelerometer scale setted to:",bin(arange))
     else:
        print("ERROR - UNAVAILABLE SCALE SET: TRY 2,4,8, or 16.")
  
  def setGyroScale(self,arange):
       if (arange==SCALE_250DPS)or(arange==SCALE_500DPS)or(arange==SCALE_1000DPS)or(arange==SCALE_2000DPS):
          value=self.bus.read_byte_data(self.address,REG_GYRO_CONFIG)
          value &= 0b11100111
          value |= (arange << 3)
          self.bus.write_byte_data(self.address,REG_GYRO_CONFIG, value)
          value=self.bus.read_byte_data(self.address,REG_GYRO_CONFIG)
          value &= 0b00011000;
          value >>= 3;
          self.gyroscale= 250*(2**value)
          self.gyroScalePerDigit=(self.gyroscale/32767) 
          print("Gyroscope scale setted to:",bin(arange))
       else:
          print("ERROR - UNAVAILABLE SCALE SET: TRY 250,500,1000, or 2000.")     
  def setRate(self,rate):# seleciona a taxa de amostragem com base no registrador 26 
    value=self.bus.read_byte_data(self.address,REG_CONFIG)
    #self.rate=value
    value &= 0b11000000
    value |= (000<< 3) #disable fsync
    value |= (rate<< 0)
    time.sleep(0.01)
    self.bus.write_byte_data(self.address, REG_CONFIG, value)
    time.sleep(0.01)
    gfchoice=self.bus.read_byte_data(self.address,REG_GYRO_CONFIG)
    #self.rate=value
    gfchoice &= 0b11111100
    gfchoice |= 0b10
    time.sleep(0.01)
    self.bus.write_byte_data(self.address, REG_GYRO_CONFIG, gfchoice)   
    #div sample rate
    time.sleep(0.01)
    self.bus.write_byte_data(self.address, REG_SMP_DIV, 0b00000000)    #DIV BY 1 
    #set accelerometer fchoice----------------------------------------------------
    time.sleep(0.01)
    achoice=self.bus.read_byte_data(self.address,REG_ACCEL_CONFIG)
    achoice &= 0b11110111
    achoice |= 0b1<<3
    self.bus.write_byte_data(self.address, REG_ACCEL_CONFIG, achoice)   
    #set A_DLPFCFG
    time.sleep(0.01)
    adlpfcfg=self.bus.read_byte_data(self.address,REG_ACCEL_CONFIG)
    adlpfcfg &= 0b11111000
    adlpfcfg |= 0b0<<0
    time.sleep(0.01)
    self.bus.write_byte_data(self.address, REG_ACCEL_CONFIG, adlpfcfg) 
    
  def getAccelRange(self):
    value=self.bus.read_byte_data(self.address,REG_ACCEL_CONFIG)
    value &= 0b00011000;
    value >>= 3;
    self.accelrange=2* (2**value)
    self.accelRangePerDigit=(self.accelrange*9.80665/32767) 
    return self.accelrange

  def getGyroScale(self):
    value=self.bus.read_byte_data(self.address,REG_GYRO_CONFIG)
    value &= 0b00011000;
    value >>= 3;
    self.gyroscale= 250*(2**value)
    self.gyroScalePerDigit=(self.gyroscale/32767) 
    return self.gyroscale       
 
  def read_raw_accel(self):
    self.accel_xout = (self.read_word_2c(XACCEL))#>>1)<<1
    self.accel_yout = (self.read_word_2c(YACCEL))#>>1)<<1
    self.accel_zout = (self.read_word_2c(ZACCEL))#>>1)<<1
    return [self.accel_xout,self.accel_yout,self.accel_zout]
    
  def read_raw_gyro(self):
    self.gyro_xout = self.read_word_2c(XGYRO)
    self.gyro_yout = self.read_word_2c(YGYRO)
    self.gyro_zout = self.read_word_2c(ZGYRO)
    return [self.gyro_xout,self.gyro_yout,self.gyro_zout]

  def readAccel(self):
    self.p_accel_xout=self.accel_xout
    self.p_accel_yout=self.accel_yout
    self.p_accel_zout=self.accel_zout
    self.read_raw_accel()
    self.accel_xout=self.accel_xout-self.XACCEL_OFFSET
    self.accel_yout=self.accel_yout-self.YACCEL_OFFSET
    self.accel_zout=self.accel_zout-self.ZACCEL_OFFSET
    #if acceleration change by a delta below than threshold, so assume than equals previous value 
    #--No integration error here, because it is already a position measure
    if(abs(self.accel_xout- self.p_accel_xout)<=self.XACCEL_TH):
      self.accel_xout=self.p_accel_xout
    if(abs(self.accel_yout- self.p_accel_yout)<=self.YACCEL_TH):
      self.accel_yout=self.p_accel_yout
    if(abs(self.accel_zout- self.p_accel_zout)<=self.ZACCEL_TH):
      self.accel_zout=self.p_accel_zout
    return [round(self.accel_xout,self.digits) ,round( self.accel_yout,self.digits) ,round(self.accel_zout,self.digits)]
    
  def readGyro(self):
    self.p_gyro_xout=self.gyro_xout
    self.p_gyro_yout=self.gyro_yout
    self.p_gyro_zout=self.gyro_zout
    self.read_raw_gyro()
    self.gyro_xout-=self.XGYRO_OFFSET
    self.gyro_yout-=self.YGYRO_OFFSET
    self.gyro_zout-=self.ZGYRO_OFFSET
    #if omega below gyro threshold, then it goes to zero, to prevent accumulate error when be integrated
    if(abs(self.gyro_xout)<self.XGYRO_TH):
      self.gyro_xout=0
    if(abs(self.gyro_yout)<self.YGYRO_TH):
      self.gyro_yout=0
    if(abs(self.gyro_zout)<self.ZGYRO_TH):
      self.gyro_zout=0
  
  def readScaledAccel(self):
    self.readAccel()
    scaled_accel_xout = 2*self.accelRangePerDigit*self.accel_xout
    scaled_accel_yout = 2*self.accelRangePerDigit*self.accel_yout
    scaled_accel_zout = 2*self.accelRangePerDigit*self.accel_zout
    return [round(scaled_accel_xout,self.digits),round(scaled_accel_yout,self.digits),round(scaled_accel_zout,self.digits)]
  
  def readScaledGyro(self):
    self.readGyro()
    scaled_gyro_xout = self.gyroScalePerDigit*self.gyro_xout
    scaled_gyro_yout = self.gyroScalePerDigit*self.gyro_yout
    scaled_gyro_zout = self.gyroScalePerDigit*self.gyro_zout
    return [round(scaled_gyro_xout,self.digits),round(scaled_gyro_yout,self.digits),round(scaled_gyro_zout,self.digits)]
    

 
  def calibrateAccel(self,samples):
    sumX=0
    sumY=0
    sumZ=0
    sigmaX=0
    sigmaY=0
    sigmaZ=0
    for i in range(samples):
      readAccel=self.read_raw_accel()
      sumX+=readAccel[0]
      sumY+=readAccel[1]
      sumZ+=readAccel[2]
      sigmaX+=readAccel[0]*readAccel[0]
      sigmaY+=readAccel[1]*readAccel[1]
      sigmaZ+=readAccel[2]*readAccel[2]
      time.sleep(0.001)#sleep 5 milisseconds until next measure
    self.XACCEL_OFFSET=sumX/samples
    self.YACCEL_OFFSET=sumY/samples
    self.ZACCEL_OFFSET=(sumZ/samples)
    self.XACCEL_SIGMA=math.sqrt((sigmaX/samples)-((sumX/samples)**2) )
    self.YACCEL_SIGMA=math.sqrt((sigmaY/samples)-((sumY/samples)**2) )
    self.ZACCEL_SIGMA=math.sqrt((sigmaZ/samples)-((sumZ/samples)**2) ) 
    self.ZACCEL_OFFSET=(sumZ/samples)#-2*(9.80665/self.accelRangePerDigit)
    if(self.verbose):
      print('Accelerometer calibration data:\r\n')
      print('Axis offsets:\r\n')
      print('X= ', self.XACCEL_OFFSET)
      print('Y= ', self.YACCEL_OFFSET)
      print('Z= ', self.ZACCEL_OFFSET)
      print('Axis standart deviations:\r\n')
      print('X= ', self.XACCEL_SIGMA)
      print('Y= ', self.YACCEL_SIGMA)
      print('Z= ', self.ZACCEL_SIGMA)

  def calibrateGyro(self,samples):
    sumX=0
    sumY=0
    sumZ=0
    sigmaX=0
    sigmaY=0
    sigmaZ=0
    for i in range(samples):
      readGyro=self.read_raw_gyro()
      sumX+=readGyro[0]
      sumY+=readGyro[1]
      sumZ+=readGyro[2]
      sigmaX+=readGyro[0]*readGyro[0]
      sigmaY+=readGyro[1]*readGyro[1]
      sigmaZ+=readGyro[2]*readGyro[2]
      time.sleep(0.001)#sleep 5 milisseconds until next measure
    self.XGYRO_OFFSET=sumX/samples
    self.YGYRO_OFFSET=sumY/samples
    self.ZGYRO_OFFSET=sumZ/samples
    self.XGYRO_SIGMA=math.sqrt((sigmaX/samples)-((sumX/samples)**2) )
    self.YGYRO_SIGMA=math.sqrt((sigmaY/samples)-((sumY/samples)**2) )
    self.ZGYRO_SIGMA=math.sqrt((sigmaZ/samples)-((sumZ/samples)**2) ) 
    if(self.verbose):
      print('Gyroscope calibration data:\n')
      print('Axis offsets:\r\n')
      print('X= ', self.XGYRO_OFFSET)
      print('Y= ', self.YGYRO_OFFSET)
      print('Z= ', self.ZGYRO_OFFSET)
      print('Axis standart deviations:\r\n')
      print('X= ', self.XGYRO_SIGMA)
      print('Y= ', self.YGYRO_SIGMA)
      print('Z= ', self.ZGYRO_SIGMA)
  
  def setAccelThreshold(self,multiplier):
    self.XACCEL_TH=self.XACCEL_SIGMA*multiplier
    self.YACCEL_TH=self.YACCEL_SIGMA*multiplier
    self.ZACCEL_TH=self.ZACCEL_SIGMA*multiplier   

  def setGyroThreshold(self,multiplier):
    self.XGYRO_TH=self.XGYRO_SIGMA*multiplier
    self.YGYRO_TH=self.YGYRO_SIGMA*multiplier
    self.ZGYRO_TH=self.ZGYRO_SIGMA*multiplier
      
    return [round(self.gyro_xout,self.digits)  ,round(self.gyro_yout,self.digits) ,round(self.gyro_zout,self.digits) ]    
 
  #----------------------------------------------------------código baseado na biblioteca - mpu9250 - FAB9axis
  ## Data Convert
  # @param [in] self The object pointer.
  # @param [in] data1 LSB
  # @param [in] data2 MSB
  # @retval Value MSB+LSB(int 16bit)
  def dataConv(self, data1, data2):
      value = data1 | (data2 << 8)
      if(value & (1 << 16 - 1)):
          value -= (1<<16)
      return value  
      
  def setupMag(self, mode, mfs):
        if mfs == AK8963_BIT_14:
            self.mres = 4912.0/8190.0
        else: #  mfs == AK8963_BIT_16:
            self.mres = 4912.0/32760.0
        self.bus.write_byte_data(AK8963_SLAVE_ADDRESS, AK8963_CNTL1, 0x00)
        time.sleep(0.01)
        # set read FuseROM mode
        self.bus.write_byte_data(AK8963_SLAVE_ADDRESS, AK8963_CNTL1, 0x0F)
        time.sleep(0.01)
        # read coef data
        data = self.bus.read_i2c_block_data(AK8963_SLAVE_ADDRESS, AK8963_ASAX, 3)
        self.magXcoef = (data[0] - 128) / 256.0 + 1.0
        self.magYcoef = (data[1] - 128) / 256.0 + 1.0
        self.magZcoef = (data[2] - 128) / 256.0 + 1.0
        # set power down mode
        self.bus.write_byte_data(AK8963_SLAVE_ADDRESS, AK8963_CNTL1, 0x00)
        time.sleep(0.01)
        # set scale&continous mode
        self.bus.write_byte_data(AK8963_SLAVE_ADDRESS, AK8963_CNTL1, (mfs<<4|mode))
        time.sleep(0.01)
        
  def readMagnet(self):
        # check data ready
        drdy = self.bus.read_byte_data(AK8963_SLAVE_ADDRESS, AK8963_ST1)
        if drdy & 0x01 :
            data = self.bus.read_i2c_block_data(AK8963_SLAVE_ADDRESS, AK8963_MAGNET_OUT, 7)
            # check overflow
            if (data[6] & 0x08)!=0x08:
                self.mag_xout = self.dataConv(data[0], data[1]) * self.mres * self.magXcoef  # x component * resolution * magnetic coeficient
                self.mag_yout = self.dataConv(data[2], data[3]) * self.mres * self.magYcoef
                self.mag_zout = self.dataConv(data[4], data[5]) * self.mres * self.magZcoef
        #return {"x":self.mag_xout, "y":self.mag_yout, "z":self.mag_zout}
        return [round(self.mag_xout,self.digits) , round(self.mag_yout,self.digits) , round(self.mag_zout,self.digits)]

  ## Read temperature
  #  @param [out] temperature temperature(degrees C)
  def readTemperature(self):
      data = self.bus.read_i2c_block_data(self.address, TEMP_OUT, 2)
      temp = self.dataConv(data[1], data[0])
      temp = round((temp / 333.87 + 21.0), 3)
      return temp
  
  
#-----------------------------------------------------------------------------------------------




#------------->>>>>>>>>>>TEM QUE COLOCAR FUNÇÃO SET_THRESHOLD, E PADRONIZAR NOMENCLATURA<<<<<<<<<<-------------------------------



# O que faz: lê aceleração em m/s², lê velocidade angular em °/s, lê campo magnético em uT, em XYZ, e temperatura
#Precisa fazer umas flags para controlar as coisas como os sigmas
#função para definir ou não o threshold, é opcional e depende do desvio padrão, o threshold é multiplo do sigma
#tem que criar uma biblioteca só para fazer leituras de IMU, que cria uma trhead com interrupção de tempo e amostra dados no intervalo 10ms, com filtragem
#questão de normalizar offset de acelerometro, normaliza para 9.8066m/s²
#
#
#Não usa mais:
  #def setupMag(self):
    ##Setup MPU9150 to receive data from compass and write it to internal registers.
    ##I2C_write(0x24, 0x40);      // Setup device to wait for external sensor (compass) data before firing interupt
    #self.bus.write_byte_data(self.address, 0x24,0x40 )
    ##Slave 0
    #self.bus.write_byte_data(self.address,0x25,0x8c)#Set I2C address for slave 0.
    #self.bus.write_byte_data(self.address,0x26,0x02)#Set internal register for compass data.
    #self.bus.write_byte_data(self.address,0x27,0x88)#Set I2C control for slave 0.

