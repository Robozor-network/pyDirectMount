import sys
import sys
import time
import datetime
import math
from pymlab import config
from support import axis
from threading import Thread
import numpy as np
import rospy
from arom.srv import *
from arom.msg import *

from astropy import units as u
from astropy.time import Time
from astropy.coordinates import SkyCoord  # High-level coordinates
from astropy.coordinates import ICRS, Galactic, FK4, FK5, AltAz  # Low-level frames
from astropy.coordinates import Angle, Latitude, Longitude  # Angles
from astropy.coordinates import EarthLocation
from astropy.coordinates import get_sun

#from astropy.utils import iers
#print "IERS sould be updated from", iers.IERS_A_URL
#iers.IERS.iers_table = iers.IERS_A.open("/home/roman/finals2000A.all")

#from astroplan import download_IERS_A
#download_IERS_A()



class drive(object):

    def __init__(self, profile = False, mode = "eq", steps_per_revA = 0, reverseA = False,
                steps_per_revB = 0, reverseB = False, speedMode = 'off', connectMethod = 'pymlab',
                connect = True, obs_lat = 48.986976, obs_lon = 14.467532, obs_alt = 300, port = '/dev/ttyUSB0'):

        self.observatory = EarthLocation(lat=obs_lat*u.deg, lon=obs_lon*u.deg, height=obs_alt*u.m)
        self.port = port
        self.profile = profile # profil montaze - pokud je preddefinovany nemusi se znovu nastavovat
        self.mount_mode = mode # 'eq' - equatorial, 'az' - azimuthal
        self.mount_steps = (steps_per_revA, steps_per_revB)
        self.mount_direction = (reverseA, reverseB)
        self.speedMode = speedMode # 'sidereal' - rychlost pohybu hvezd, 'off' - vypnuty hodinovy stroj, 'custom' - vlastni rychlost, 'solar', 'lunal', ...
        self.speedModeVelocity = 0
        self.connectMethod = connectMethod
        self.mount_position = SkyCoord(0, 0, frame = "icrs", unit="deg")
        self.home_position = SkyCoord(alt = obs_lat, az = 0, obstime = Time.now(), frame = 'altaz', unit="deg", location = self.observatory)
        self.mount_position = self.home_position.icrs
        self.mount_target = False
        self.mount_slew = False
        self.park = True


        if connect:
            self.connect()

        #TODO moznost nastavit profil a pak teprve upravit parametry ze zadain
        if profile == 'HEQ5':
            self.mount_mode = 'eq'
            self.mount_steps = (2250429, 2250429) # osa RA zhruba zmerena na 13 238 879 kroku. bez nastaveni microsteppingu (tzn. 1/128), 
                                                  # 2250429 pri 1/16 microstep tzn. - cely registr je cca 1.9 otacky (mereno na RA)
            self.mount_direction = (False, False) # smer otaceni os

        self.mount_stepsperdeg = (self.mount_steps[0]/360.0, self.mount_steps[1]/360.0)

    def beep(self, duration = 500):
        self.spi.GPIO_write(0b11111111)
        time.sleep(duration/1000)
        self.spi.GPIO_write(0b00000000)

    def getObs(self):
        return self.observatory

    def movementMode(self, mode = False, velocity = 0):
        if mode:
            self.speedMode = mode
            self.speedModeVelocity = velocity
        print "movement Mode is", self.speedMode, " (", self.speedModeVelocity, ")"
        return (self.speedMode, self.speedModeVelocity)

    def mThread(self):
        while getattr(self, "run", True):
            AxA_mountDir, AxB_mountDir = self.mount_direction
            if self.park:
                print "park"
                self.AxA.Float()
                self.AxB.Float()
                self.AxA.GoHome(False)
                self.AxB.GoHome(False)
                while self.AxA.IsBusy() | self.AxB.IsBusy():
                    pass
            
            elif self.mount_slew:
                print ""
                print ""
                print ""
                print "slew mode"
                print "============"
                print "pozadovane souradnice:", self.mount_target

                target = self.mount_target
                self.mount_target = False

                now = datetime.datetime.utcnow()
                midnight = now.replace(hour=0, minute=0, second=0, microsecond=0)
                HAvariable = ((now - midnight).seconds/(24*60*60.0))/360.0 - 14.467532 #obs longtitude
                print "sidereal time", now - midnight, HAvariable


                t = Time(datetime.datetime.utcnow(), scale='utc',
                         location=self.observatory) 

                print t

                #HAvariable = t.sidereal_time('mean')
                print Time.now()
                t2 = Time(str(Time.now()), scale='utc', location=self.observatory)
                ##print t2.sidereal_time('apparent')
                HAvariable = t2.sidereal_time('mean').degree
                print t2.sidereal_time('mean'), "-->", HAvariable


                ha = HAvariable - target.ra.degree
                dec = target.dec.degree



                print "1, ha/dec:", ha, dec, "z ra", target.ra.degree
                
                ha = -(((ha + 180) % 360 - 180)+90)
                dec = -(((dec + 180) % 360 - 180))

                print "2, ha/dec:",ha, dec



                if -180 <= ha <= -90:
                    print "mode: -180 - -90"
                    ha = ha+180
                    dec = dec-180

                elif -90  <= ha <=   0:   # OK
                    print "mode: -90 - 0" ####
                    ha = ha
                    dec = dec

                elif 0 <= ha <=  90:    # OK
                    print "mode: 0 - 90"
                    ha = ha
                    dec = dec

                elif 90 <= ha <= 180:
                    print "mode: 90 - 180"
                    ha = ha
                    dec = dec

                else:
                    print "CHYBA --- out of range", ha, dec

                print "3, ha/dec:", ha, dec

                ha = ha + 0
                dec = dec + 90

                print "4, ha/dec:", ha, dec

                self.AxA.Float()
                self.AxB.Float()
                time.sleep(0.5)

                self.AxA.GoTo(int(ha *self.mount_stepsperdeg[0]))
                self.AxB.GoTo(int(dec*self.mount_stepsperdeg[1]))

                while self.AxA.IsBusy() | self.AxB.IsBusy():
                    print "blaB", self.AxA.IsBusy(), self.AxB.IsBusy()
                print "##################################################################"

                while self.AxA.IsBusy() or self.AxB.IsBusy():
                    reg_a = self.AxA.ReadStatusReg()
                    reg_b = self.AxB.ReadStatusReg()
                    print reg_a, reg_b, ">>>>>", self.AxA.IsBusy(), self.AxB.IsBusy()
                    #print  "\t\t ", bin(reg_a[0])[2:].zfill(8), bin(reg_a[1])[2:].zfill(8), '\t\t', bin(reg_b[0])[2:].zfill(8), bin(reg_b[1])[2:].zfill(8)

                #print "puvodni poloha enkonderu je ", pos_old, "nova", pos_req
               

                #print 'steps:', ra *self.mount_steps[0]/360, dec*self.mount_steps[1]/360

                #while self.AxA.IsBusy() or self.AxB.IsBusy():
                #    pass

                print "Slew dokonceno"
                self.mount_slew = False
                self.mount_position = target

                self.AxA.Float()
                self.AxB.Float()
                time.sleep(0.5)
                self.speedMode = 'sidereal'

            elif self.speedMode == 'off':
                self.AxA.Float()
                self.AxB.Float()
                lastA = 0
                lastB = 0
                #print "off"

            elif self.speedMode == 'sidereal':
                newA = 50
                newB = 0
                #if lastA != newA or lastB != newB:
                self.AxB.Run(int(AxA_mountDir), 3)
                self.AxA.Float()
                #self.AxA.ReadStatusReg()
                #self.AxB.ReadStatusReg()
                lastA = 50
                lastB = 0
                reg_a = self.AxA.ReadStatusReg()
                reg_b = self.AxB.ReadStatusReg()
                print reg_a, reg_b
                #print bin(reg_a1), bin(reg_a2), bin(reg_b1), bin(reg_b2), "<<<"
                #print  "\t\t ", bin(reg_a[0])[2:].zfill(8), bin(reg_a[1])[2:].zfill(8), '\t\t', bin(reg_b[0])[2:].zfill(8), bin(reg_b[1])[2:].zfill(8)

                    #print self.AxA.Run(1, self.mount_steps[0]/(24*60*60))

    def run(self):
        print 'Zapinam vlakno'
        self.t1 = Thread(target = self.mThread)
        self.t1.setDaemon(True)
        self.t1.start()

    def connect(self):
        
        self.pymlab = rospy.ServiceProxy('pymlab_drive', PymlabDrive)
        
        print eval(self.pymlab(device="telescope_spi", method="route").value)
        print eval(self.pymlab(device="telescope_spi", method="SPI_config", parameters="0b1001").value) # self.spi.I2CSPI_MSB_FIRST| self.spi.I2CSPI_MODE_CLK_IDLE_HIGH_DATA_EDGE_TRAILING| self.spi.I2CSPI_CLK_461kHz

        time.sleep(.25)
        print self.pymlab
        self.AxA = axis.axis(self.pymlab, 0b0001, 1, 1, protocol = 'arom')    # set Number of Steps per axis Unit and set Direction of Rotation
        self.AxA.Reset()
        self.AxA.MaxSpeed(0x33ffff)                      # set maximal motor speed 


        self.AxB = axis.axis(self.pymlab, 0b0010, 1, 1, protocol = 'arom')    
        self.AxB.Reset()
        self.AxB.MaxSpeed(0x33ffff)                      # set maximal motor speed


        self.AxB.MoveWait(2000)
        self.AxA.MoveWait(2000)
        self.AxB.MoveWait(-2000)
        self.AxA.MoveWait(-2000)

        

    def Slew(self, target):
        print "Slew to", target
        self.mount_target = target
        self.mount_slew = True

    def GoPark(self):
        self.park = True

    def UnPark(self):
        self.park = False

    def hardStop(self):
        self.speedMode = 'off'
        self.AxA.Run(0, 0)
        self.AxB.Run(0, 0)
        self.AxA.Float()
        self.AxB.Float()

    def setMode(self, mode):
        self.speedMode = mode
        return True



def main():
    try:
        driver = drive(profile = 'HEQ5')
        driver.run()
        driver.UnPark()

        driver.movementMode('sidereal')
        print "A"
        time.sleep(1)
        print "B"

        print "*************************"
        #driver.Slew(SkyCoord(alt = 1, az = 90, obstime = Time.now(), frame = 'altaz', unit="deg", location = driver.getObs()).icrs)

        #driver.Slew(SkyCoord(ra = 1.3, dec = 0, obstime = Time.now(), frame = 'icrs', unit="deg", location = driver.getObs()))
        


        print "sun", get_sun(Time.now()).icrs
        #driver.Slew(get_sun(Time.now()))




        #driver.Slew(SkyCoord(alt = 45, az = 45+180+90, obstime = Time.now(), frame = 'altaz', unit="deg", location = driver.getObs()).icrs)

        driver.Slew(SkyCoord(alt = 45, az = 45, obstime = Time.now(), frame = 'altaz', unit="deg", location = driver.getObs()).icrs)

        #driver.Slew(SkyCoord(alt = 10, az = 180, obstime = Time.now(), frame = 'altaz', unit="deg", location = driver.getObs()).icrs)

        #driver.Slew(SkyCoord(alt = 10, az = 180, obstime = Time.now(), frame = 'altaz', unit="deg", location = driver.getObs()).icrs)

        time.sleep(1)

        #driver.Slew(SkyCoord(alt = 10, az = 180, obstime = Time.now(), frame = 'altaz', unit="deg", location = driver.getObs()).icrs)

        #time.sleep(1)

        #driver.Slew(SkyCoord(alt = 10, az = 180+90, obstime = Time.now(), frame = 'altaz', unit="deg", location = driver.getObs()).icrs)

        #time.sleep(5)

        driver.GoPark()
        #driver.Slew(SkyCoord(alt = 1, az = 90, obstime = Time.now(), frame = 'altaz', unit="deg", location = driver.getObs()).icrs)

        #driver.movementMode('sidereal')

        while True:
            pass

    except KeyboardInterrupt:
        print "Ukoncuji"
        driver.hardStop()
        print "Konec", Time.now()
    #drive.connect()


if __name__ == '__main__':
    main()
