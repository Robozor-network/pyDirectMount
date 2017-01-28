import time

class axis:
    def __init__(self, SPI, SPI_CS, Direction, StepsPerUnit, protocol = 'i2c'):
        ' One axis of robot '
        self.CS = SPI_CS
        self.Dir = Direction
        self.SPU = StepsPerUnit
        self.spi = SPI
        self.protocol = protocol
        self.Reset()


    def writeByte(self, CS, address):
        if self.protocol == 'i2c':
            return self.spi.SPI_write_byte(self.CS, address)
        elif self.protocol == 'spi':
            return "Err. SPI neni podporovano"
        elif self.protocol == 'arom':
            return eval(self.spi(device="telescope_spi", method="SPI_write_byte", parameters=str((self.CS,address))).value) # self.spi.I2CSPI_MSB_FIRST| self.spi.I2CSPI_MODE_CLK_IDLE_HIGH_DATA_EDGE_TRAILING| self.spi.I2CSPI_CLK_461kHz


    def readByte(self):
        if self.protocol == 'i2c':
            return self.spi.SPI_read_byte(self.CS, address)
        elif self.protocol == 'spi':
            return "Err. SPI neni podporovano"
        elif self.protocol == 'arom':
            return eval(self.spi(device="telescope_spi", method="SPI_read_byte").value)


    def Reset(self):
        ' Reset Axis and set default parameters for H-bridge '
        self.writeByte(self.CS, 0xC0)      # reset

        #self.writeByte(self.CS, 0x14)      # Stall Treshold setup
        #self.writeByte(self.CS, 0xFF)  
        #self.writeByte(self.CS, 0x13)      # Over Current Treshold setup 
        #self.writeByte(self.CS, 0xFF)

        self.writeByte(self.CS, 0x15)      # Full Step speed 
        self.writeByte(self.CS, 0xFF)
        self.writeByte(self.CS, 0xFF)
        self.writeByte(self.CS, 0xFF)

        self.writeByte(self.CS, 0x05)      # ACC (0x008a)
        self.writeByte(self.CS, 0x00)
        self.writeByte(self.CS, 0x8a)

        self.writeByte(self.CS, 0x06)      # DEC (0x008a)
        self.writeByte(self.CS, 0x00)
        self.writeByte(self.CS, 0x8a)

        self.writeByte(self.CS, 0x0A)      # KVAL_RUN -  Constant speed
        self.writeByte(self.CS, 0xF0)

        self.writeByte(self.CS, 0x0B)      # KVAL_ACC
        self.writeByte(self.CS, 0xF0)

        self.writeByte(self.CS, 0x0C)      # KVAL_DEC
        self.writeByte(self.CS, 0xF0)

        self.writeByte(self.CS, 0x18)      # CONFIG
        #self.writeByte(self.CS, 0x00)
        self.writeByte(self.CS, 0b00111000)
        self.writeByte(self.CS, 0b00000000)
        #self.writeByte(self.CS, 0b00111110)
        #self.writeByte(self.CS, 0b10000000)

        self.writeByte(self.CS, 0x16)      # Microstepping
        self.writeByte(self.CS, 0x04)      # 0x04 - 1/16
      
    def MaxSpeed(self, speed):
        ' Setup of maximum speed '
        self.writeByte(self.CS, 0x07)       # Max Speed setup 
        self.writeByte(self.CS, 0x00)
        self.writeByte(self.CS, (speed >> 16) & 0xFF)  
        self.writeByte(self.CS, (speed >> 8) & 0xFF)  
        self.writeByte(self.CS, (speed) & 0xFF)  

    def GoTo(self, abspos):
        self.writeByte(self.CS, 0x60)       # Max Speed setup 
        self.writeByte(self.CS, (abspos >> 16) & 0xFF)  
        self.writeByte(self.CS, (abspos >> 8) & 0xFF)  
        self.writeByte(self.CS, (abspos) & 0xFF)
        return abspos

    def ReleaseSW(self):
        ' Go away from Limit Switch '
        while self.ReadStatusBit(2) == 1:           # is Limit Switch ON ?
            self.writeByte(self.CS, 0x92 | (~self.Dir & 1))     # release SW 
            while self.IsBusy():
                time.sleep(0.25)
            self.MoveWait(10)           # move 10 units away
 
    def GoZero(self, speed):
        ' Go to Zero position '
        self.ReleaseSW()

        self.writeByte(self.CS, 0x82 | (self.Dir & 1))       # Go to Zero
        self.writeByte(self.CS, 0x00)
        self.writeByte(self.CS, speed)  
        while self.IsBusy():
            time.sleep(0.25)
        time.sleep(0.3)
        self.ReleaseSW()

    def GoHome(self, wait = True):
        ' Go to Zero position '
        self.ReleaseSW()

        self.writeByte(self.CS, 0x70)       # Go to Zero
        self.writeByte(self.CS, 0x00)
        while self.IsBusy() and wait:
            time.sleep(0.25)

    def ResetPos(self):
        self.writeByte(self.CS, 0xD8)       # Reset position
        self.writeByte(self.CS, 0x00)

    def Move(self, units):
        ' Move some distance units from current position '
        print 'move', units, 'units'
        steps = units * self.SPU  # translate units to steps 
        if steps > 0:                                          # look for direction
            self.writeByte(self.CS, 0x40 | (~self.Dir & 1))       
        else:
            self.writeByte(self.CS, 0x40 | (self.Dir & 1)) 
        steps = int(abs(steps))     
        self.writeByte(self.CS, (steps >> 16) & 0xFF)
        self.writeByte(self.CS, (steps >> 8) & 0xFF)
        self.writeByte(self.CS, steps & 0xFF)
        return steps

    def MoveWait(self, units):
        ' Move some distance units from current position and wait for execution '
        self.Move(units)
        while self.IsBusy():
            pass

    def Run(self, direction, speed):
        print "run", direction, speed
        speed_value = int(abs(speed) / 0.015)

        if speed < 0:
            direction = not bool(direction)

        self.writeByte(self.CS, 0b01010000 + int(direction))
        self.writeByte(self.CS, (speed_value >> 16) & 0xFF)
        self.writeByte(self.CS, (speed_value >> 8) & 0xFF)
        self.writeByte(self.CS, (speed_value >> 0) & 0xFF)

        data = [0b01010000 + int(direction)]
        data = data +[(speed_value >> i & 0xff) for i in (16,8,0)]
        print data
        for x in data:
            print format(speed_value, '08b'),
        print ""
        #self.writeByte(self.CS,data[0])       # Max Speed setup 
        #self.writeByte(self.CS,data[1])
        #self.writeByte(self.CS,data[2])  
        #self.writeByte(self.CS,data[3])
        #print speed_value, data
        return (speed_value * 0.015)
        #return (speed_value)

    def Float(self):
        ' switch H-bridge to High impedance state '
        print "switch H-bridge to High impedance state"
        self.writeByte(self.CS, 0xA0)

    def ReadStatusBit(self, bit):
        ' Report given status bit '
        self.writeByte(self.CS, 0x39)   # Read from address 0x19 (STATUS)
        self.writeByte(self.CS, 0x00)
        data0 = self.readByte()           # 1st byte
        self.writeByte(self.CS, 0x00)
        data1 = self.readByte()           # 2nd byte
        #print hex(data0), hex(data1)
        if bit > 7:                                   # extract requested bit
            OutputBit = (data0 >> (bit - 8)) & 1
        else:
            OutputBit = (data1 >> bit) & 1        
        return OutputBit

    def ReadStatusReg(self):
        self.writeByte(self.CS, 0x39)   # Read from address 0x19 (STATUS)
        self.writeByte(self.CS, 0x00)
        data0 = self.readByte()           # 1st byte
        self.writeByte(self.CS, 0x00)
        data1 = self.readByte() 
        print  "\t\t\t\t ", bin(data0)[2:].zfill(8), bin(data1)[2:].zfill(8), self.ReadStatusBit(4)   
        #return (data0, data1)

    def ABS_POS(self):
        return 0x01

    def EL_POS(self):
        return 0x02

    def MARK(self):
        return 0x03

    def SPEED(self):
        return 0x04

    def ReadParam(self, param):
        self.writeByte(self.CS, 0x20 | param)
        self.writeByte(self.CS, 0x00)
        data0 = self.readByte()           # 1st byte
        self.writeByte(self.CS, 0x00)
        data1 = self.readByte()           # 2nd byte
        self.writeByte(self.CS, 0x00)
        data2 = self.readByte()
        return data0 << 16 | data1 <<8 | data2

    def ReadPosition(self):
        self.writeByte(self.CS, 0x20 | 0x01)   # Read from address (STATUS)
        self.writeByte(self.CS, 0x00)
        data0 = self.readByte()           # 1st byte
        self.writeByte(self.CS, 0x00)
        data1 = self.readByte()           # 2nd byte
        self.writeByte(self.CS, 0x00)
        data2 = self.readByte()           # 3rd byte
        return data0 << 16 | data1 <<8 | data2
    
    def IsBusy(self):
        """ Return True if tehre are motion """
        if self.ReadStatusBit(1) == 1:
            return False
        else:
            return True

# End Class axis --------------------------------------------------
