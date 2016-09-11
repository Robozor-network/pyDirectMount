import time

class axis:
    def __init__(self, SPI, SPI_CS, Direction, StepsPerUnit):
        ' One axis of robot '
        self.CS = SPI_CS
        self.Dir = Direction
        self.SPU = StepsPerUnit
        self.spi = SPI
        self.Reset()


    def Reset(self):
        ' Reset Axis and set default parameters for H-bridge '
        self.spi.SPI_write_byte(self.CS, 0xC0)      # reset
        #self.spi.SPI_write_byte(self.CS, 0x14)      # Stall Treshold setup
        #self.spi.SPI_write_byte(self.CS, 0xFF)  
        #self.spi.SPI_write_byte(self.CS, 0x13)      # Over Current Treshold setup 
        #self.spi.SPI_write_byte(self.CS, 0xFF)  
        self.spi.SPI_write_byte(self.CS, 0x15)      # Full Step speed 
        self.spi.SPI_write_byte(self.CS, 0xFF)
        self.spi.SPI_write_byte(self.CS, 0xFF) 
        self.spi.SPI_write_byte(self.CS, 0xFF) 
        self.spi.SPI_write_byte(self.CS, 0x00)      # ACC 
        self.spi.SPI_write_byte(self.CS, 0x00)
        self.spi.SPI_write_byte(self.CS, 0x10) 
        self.spi.SPI_write_byte(self.CS, 0x00)      # DEC 
        self.spi.SPI_write_byte(self.CS, 0x00)
        self.spi.SPI_write_byte(self.CS, 0x10) 
        self.spi.SPI_write_byte(self.CS, 0x0A)      # KVAL_RUN
        self.spi.SPI_write_byte(self.CS, 0xF0)
        self.spi.SPI_write_byte(self.CS, 0x0B)      # KVAL_ACC
        self.spi.SPI_write_byte(self.CS, 0xF0)
        self.spi.SPI_write_byte(self.CS, 0x0C)      # KVAL_DEC
        self.spi.SPI_write_byte(self.CS, 0xF0)
        self.spi.SPI_write_byte(self.CS, 0x18)      # CONFIG
        self.spi.SPI_write_byte(self.CS, 0b00111000)
        self.spi.SPI_write_byte(self.CS, 0b00000000)
        self.spi.SPI_write_byte(self.CS, 0x16)      # Microstepping
        self.spi.SPI_write_byte(self.CS, 0b00000100)
      
    def MaxSpeed(self, speed):
        ' Setup of maximum speed '
        self.spi.SPI_write_byte(self.CS, 0x07)       # Max Speed setup 
        self.spi.SPI_write_byte(self.CS, 0x00)
        self.spi.SPI_write_byte(self.CS, (speed >> 16) & 0xFF)  
        self.spi.SPI_write_byte(self.CS, (speed >> 8) & 0xFF)  
        self.spi.SPI_write_byte(self.CS, (speed) & 0xFF)  

    def ReleaseSW(self):
        ' Go away from Limit Switch '
        while self.ReadStatusBit(2) == 1:           # is Limit Switch ON ?
            self.spi.SPI_write_byte(self.CS, 0x92 | (~self.Dir & 1))     # release SW 
            while self.IsBusy():
                pass
            self.MoveWait(10)           # move 10 units away
 
    def GoZero(self, speed):
        ' Go to Zero position '
        self.ReleaseSW()

        self.spi.SPI_write_byte(self.CS, 0x82 | (self.Dir & 1))       # Go to Zero
        self.spi.SPI_write_byte(self.CS, 0x00)
        self.spi.SPI_write_byte(self.CS, speed)  
        while self.IsBusy():
            pass
        time.sleep(0.3)
        self.ReleaseSW()

    def GoHome(self, wait = True):
        ' Go to Zero position '
        self.ReleaseSW()

        self.spi.SPI_write_byte(self.CS, 0x70)       # Go to Zero
        self.spi.SPI_write_byte(self.CS, 0x00)
        while self.IsBusy() and wait:
            pass


    def ResetPos(self):
        self.spi.SPI_write_byte(self.CS, 0xD8)       # Reset position
        self.spi.SPI_write_byte(self.CS, 0x00)

    def Move(self, units):
        ' Move some distance units from current position '
        print 'move', units, 'units'
        steps = units * self.SPU  # translate units to steps 
        if steps > 0:                                          # look for direction
            self.spi.SPI_write_byte(self.CS, 0x40 | (~self.Dir & 1))       
        else:
            self.spi.SPI_write_byte(self.CS, 0x40 | (self.Dir & 1)) 
        steps = int(abs(steps))     
        self.spi.SPI_write_byte(self.CS, (steps >> 16) & 0xFF)
        self.spi.SPI_write_byte(self.CS, (steps >> 8) & 0xFF)
        self.spi.SPI_write_byte(self.CS, steps & 0xFF)
        return steps

    def MoveWait(self, units):
        ' Move some distance units from current position and wait for execution '
        self.Move(units)
        while self.IsBusy():
            pass

    def Run(self, direction, speed):
        speed_value = int(speed / 0.015)

        data = [0b01010000 + direction]
        data = data +[(speed_value >> i & 0xff) for i in (16,8,0)]
        self.spi.SPI_write_byte(self.CS,data[0])       # Max Speed setup 
        self.spi.SPI_write_byte(self.CS,data[1])
        self.spi.SPI_write_byte(self.CS,data[2])  
        self.spi.SPI_write_byte(self.CS,data[3])
        print speed_value, data
        return (speed_value * 0.015)

    def Float(self):
        ' switch H-bridge to High impedance state '
        self.spi.SPI_write_byte(self.CS, 0xA0)

    def ReadStatusBit(self, bit):
        ' Report given status bit '
        self.spi.SPI_write_byte(self.CS, 0x39)   # Read from address 0x19 (STATUS)
        self.spi.SPI_write_byte(self.CS, 0x00)
        data0 = self.spi.SPI_read_byte()           # 1st byte
        self.spi.SPI_write_byte(self.CS, 0x00)
        data1 = self.spi.SPI_read_byte()           # 2nd byte
        #print hex(data0), hex(data1)
        if bit > 7:                                   # extract requested bit
            OutputBit = (data0 >> (bit - 8)) & 1
        else:
            OutputBit = (data1 >> bit) & 1        
        return OutputBit

    def ReadStatusReg(self):
        self.spi.SPI_write_byte(self.CS, 0x39)   # Read from address 0x19 (STATUS)
        self.spi.SPI_write_byte(self.CS, 0x00)
        data0 = self.spi.SPI_read_byte()           # 1st byte
        self.spi.SPI_write_byte(self.CS, 0x00)
        data1 = self.spi.SPI_read_byte() 
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
        self.spi.SPI_write_byte(self.CS, 0x20 | param)
        self.spi.SPI_write_byte(self.CS, 0x00)
        data0 = self.spi.SPI_read_byte()           # 1st byte
        self.spi.SPI_write_byte(self.CS, 0x00)
        data1 = self.spi.SPI_read_byte()           # 2nd byte
        self.spi.SPI_write_byte(self.CS, 0x00)
        data2 = self.spi.SPI_read_byte()
        return data0 << 16 | data1 <<8 | data2

    def ReadPosition(self):
        self.spi.SPI_write_byte(self.CS, 0x20 | 0x01)   # Read from address (STATUS)
        self.spi.SPI_write_byte(self.CS, 0x00)
        data0 = self.spi.SPI_read_byte()           # 1st byte
        self.spi.SPI_write_byte(self.CS, 0x00)
        data1 = self.spi.SPI_read_byte()           # 2nd byte
        self.spi.SPI_write_byte(self.CS, 0x00)
        data2 = self.spi.SPI_read_byte()           # 3rd byte
        return data0 << 16 | data1 <<8 | data2
    
    def IsBusy(self):
        """ Return True if tehre are motion """
        if self.ReadStatusBit(1) == 1:
            return False
        else:
            return True

# End Class axis --------------------------------------------------
