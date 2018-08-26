#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import sys
import time
import datetime
import math
from pymlab import config
#from support import axis
import axis
from threading import Thread
import numpy as np
#from arom.srv import *
#from arom.msg import *

import ephem
from astropy import units as u
from astropy.time import Time
from astropy.coordinates import SkyCoord  # High-level coordinates
from astropy.coordinates import ICRS, Galactic, FK4, FK5, AltAz  # Low-level frames
from astropy.coordinates import Angle, Latitude, Longitude  # Angles
from astropy.coordinates import EarthLocation
from astropy.coordinates import get_sun

from astropy.utils import iers
#print "IERS sould be updated from", iers.IERS_A_URL
iers.IERS.iers_table = iers.IERS_A.open("/home/odroid/finals2000A.all")

#from astroplan import download_IERS_A
#download_IERS_A()



class drive(object):
    status_callback = None

    def __init__(self, profile = False, mode = "eq", steps_per_revA = 0, reverseA = False,
                steps_per_revB = 0, reverseB = False, speedMode = 'off', connectMethod = 'pymlab', connectInstance = None,
                connect = True, obs_lat = 48.986976, obs_lon = 14.467532, obs_alt = 300, port = '/dev/ttyUSB0'):

        self.driver_action_buffer = [{'home':True}]
        self.driver_action_now = 0 # kolikata akce v predchozim poli se prave vykonava
        self.driver_action_status = 1 # v jakem stavu je aktualni akce - 0=dokonceno; 1=prubeh ale lze ihned prerusit (napr. tracking, home); 2=prubet a je zamceno cizi aktivitou (napr. fotak), 3=prubeh a nelze prerusit (napr slew)


        self.observatory = EarthLocation(lat=obs_lat*u.deg, lon=obs_lon*u.deg, height=obs_alt*u.m)
        self.port = port
        self.profile = profile # profil montaze - pokud je preddefinovany nemusi se znovu nastavovat
        self.mount_mode = mode # 'eq' - equatorial, 'az' - azimuthal
        self.mount_steps = (steps_per_revA, steps_per_revB)
        self.mount_direction = (reverseA, reverseB)
        self.speedMode = speedMode # 'sidereal' - rychlost pohybu hvezd, 'off' - vypnuty hodinovy stroj, 'custom' - vlastni rychlost, 'solar', 'lunal', ...
        self.connectMethod = connectMethod
        self.connectInstance = connectInstance
        self.mount_position = SkyCoord(0, 0, frame = "icrs", unit="deg")
        self.home_position = SkyCoord(alt = 0, az = 90, obstime = Time.now(), frame = 'altaz', unit="deg", location = self.observatory)
        self.mountCoord = self.mount_position
        self.mount_position = self.home_position.icrs
        self.mount_target = False
        self.mount_slew = False
        self.park = True


        if connect:
            self.connect()

        #TODO moznost nastavit profil a pak teprve upravit parametry ze zadain
        if profile == 'HEQ5':
            self.mount_mode = 'eq'
            # 141 000 kroku na 1 otoceni - ziskano z EQmod ovladace
            #self.mount_steps = (2256000, 2256000) # osa RA zhruba zmerena na 18 048 000 kroku. bez nastaveni microsteppingu (tzn. 1/128), 
            #                                      # 2256000 pri 1/16 microstep tzn. - cely registr je cca 1.9 otacky (mereno na RA)
            self.mount_steps = (18048000, 18048000) # osa RA zhruba zmerena na 18 048 000 kroku. bez nastaveni microsteppingu (tzn. 1/128), 
            self.mount_direction = (False, False) # smer otaceni os

        self.mount_stepsperdeg = (self.mount_steps[0]/360.0, self.mount_steps[1]/360.0)


        self.preset_speed = {
            'sidereal': {
                'name': 'Sidereal speed', 
                'ra': self.mount_steps[0]/86164.09053083,
                'dec': 0,
            },
            'solar': {
                'name': 'Solar mean speed', 
                'ra': self.mount_steps[0]/86400.002,
                'dec': 0,
            },
            'lunar': {
                'name': 'Lunar speed', 
                'ra': self.mount_steps[0]/86000.00,
                'dec': 0,
            }
        }
        self.trackingSpeed = [self.preset_speed['sidereal']['ra'], self.preset_speed['sidereal']['dec']]

    def new_action(self, action, data=None):
        self.driver_action_buffer.append({action: data})

    def getObs(self):
        return self.observatory

    def movementMode(self, mode = False, velocity = 0):
        self.driver_action_buffer.append({'tracking': {'mode': mode} })
        if mode:
            self.speedMode = mode
        return (self.speedMode)

    def calc_stepper_position(self, coordinates):
        '''
        This function determinate stepper positions from coordinates (icrs variable)
        input:
            coordinates: IRCS
        output:
            (ra, dec): positions of ra and dec axis
        '''
        ra, dec = self.GetHaDecCoordinates(coordinates)
        return(ra*self.mount_stepsperdeg[0], dec*self.mount_stepsperdeg[1])

    def get_LST(self):
        """
        Calculate LST (Local sideral time).
        This value is useful for converting RA to HA angle

        input:
        output:
            LST: angle of LST
        """
        t = Time(str(Time.now()), scale='utc', location=self.observatory)
        return t.sidereal_time('mean').degree


    def GetHaDecCoordinates(self, coordinates):                
        '''
        This functions returns Ha (hour angle) and Dec (declination angle) from coordinates of target
        input:
            coordinates: ICRS
        output:
            (ha, dec):
        '''

        ha = Angle((self.get_LST() - coordinates.ra.degree), unit="deg").wrap_at('180d')
        dec = Angle(coordinates.dec.degree, unit="deg").wrap_at('180d')

        if ha.is_within_bounds('-180d', '0d'):
            #print "mode: -180 ~ 0"
            ha = -(ha.degree + 90)
            dec= -(dec.degree - 90)

        elif ha.is_within_bounds('0d', '180d'):
            #print "mode: 0 ~ 180"
            ha = -(ha.degree - 90)
            dec=  (dec.degree - 90)
        else:
            print "Err --- out of range", ha, dec
            ha = None
            dec = None

        return (ha, dec)



    def run(self):
        print 'Zapinam vlakno'
        self.t1 = Thread(target = self.mThread)
        self.t1.setDaemon(True)
        self.t1.start()

    def connect(self):
        if self.connectMethod == 'pymlab_bridge':
            print("CONNECT")
            import rospy
            from arom.srv import PymlabDrive

            self.pymlab = rospy.ServiceProxy('pymlab_drive', PymlabDrive)
            rospy.set_param('/arom/node'+rospy.get_name()+"/pymlab", True)

            self.m_ra = axis.axis(SPI = self.pymlab, SPI_CS = 0b0001, Direction = True, StepsPerUnit = 1, protocol = 'arom', arom_spi_name = 'telescope')
            self.m_dec = axis.axis(SPI = self.pymlab, SPI_CS = 0b0010, Direction = True, StepsPerUnit = 1, protocol = 'arom', arom_spi_name = 'telescope')


        elif self.connectMethod == 'pymlab':
            print("CONNECT, pymlab")

            self.m_ra = axis.axis(SPI = self.connectInstance, SPI_CS = 0b0001, Direction = True, StepsPerUnit = 1, protocol = 'i2c')
            self.m_dec = axis.axis(SPI = self.connectInstance, SPI_CS = 0b0010, Direction = True, StepsPerUnit = 1, protocol = 'i2c')
            

        self.m_ra.Setup(
            MAX_SPEED = 700,
            KVAL_ACC=0.3,
            KVAL_RUN=0.3,
            KVAL_DEC=0.3,
            ACC = 1000,
            DEC = 1000,
            FS_SPD=3000,
            STEP_MODE=axis.axis.STEP_MODE_1_128)
        self.m_dec.Setup(
            MAX_SPEED = 700,
            KVAL_ACC=0.3,
            KVAL_RUN=0.3,
            KVAL_DEC=0.3,
            ACC = 1000,
            DEC = 1000,
            FS_SPD=3000,
            STEP_MODE=axis.axis.STEP_MODE_1_128)
        time.sleep(.25)

        print("Movement Test")
        self.m_ra.Move(1000)
        self.m_dec.MoveWait(1000)
        time.sleep(0.5)
        self.m_ra.Move(-1000)
        self.m_dec.MoveWait(-1000)
        self.m_ra.Wait()
        print("Movement test DONE")

        p1 = self.m_ra.virtual_position()
        self.m_ra.MoveWait(-100)
        p2 = self.m_ra.virtual_position()
        self.m_ra.MoveWait(100)
        p3 = self.m_ra.virtual_position()
        print("POS", p1, p2, p3, p1-p2)

    def Slew(self, target):
        self.driver_action_buffer.append({'slew': {'target': target} })
        print "Slew to", target
        self.mount_target = target
        self.mount_slew = True

    def GoPark(self):
        self.park = True
        self.driver_action_buffer.append({'home': True})

    def UnPark(self):
        self.park = False
        #self.driver_action_buffer.append({'tracking': False})

    def hardStop(self):
        self.driver_action_buffer.append({'stop': {'mode': 'hard'}})
        self.speedMode = 'off'
        self.m_ra.Run(0, 0)
        self.m_dec.Run(0, 0)
        self.m_ra.Float()
        self.m_dec.Float()

    def setMode(self, mode):
        self.driver_action_buffer.append({'tracking_mode': {'mode': mode}})
        self.speedMode = mode
        return True

    ### setTracking ###
    ## set_tracking
    def set_tracking(self, mode = 'sidereal', units = 'steps', ra = 0, dec = 0, ra_multiplication = 1, dec_multiplication = 1):
        '''
            This function set tracking with multpile parameters...

        input:
            mode: Tracking modes: ['sidereal', 'solar', 'lunar', 'custom']
            units: Units of ra, dec values (all per second): ['steps', 'seconds']
            ra: speed of Ra axis in selected units
            dec: speed of Dec axis in selected units
            ra_multiplication: multiplicator of RA speed
        '''

        if mode in ['sidereal', 'solar', 'lunar', 'custom']:
            if mode in self.preset_speed:
                ra = self.preset_speed[mode]['ra']
                dec = self.preset_speed[mode]['dec']
            
            if ra == 0 and ra_multiplication != 1:
                ra = self.preset_speed['sidereal']['ra']
            if dec == 0 and dec_multiplication != 1:
                dec = self.preset_speed['sidereal']['ra']
            if ra_multiplication !=1 or dec_multiplication != 1:
                mode = 'custom'
            
            ra = ra*ra_multiplication
            dec = dec*dec_multiplication
            self.addAction('tracking', {'mode': mode, 'units': units, 'ra':ra, 'dec': dec})
        else:
            raise("Unknown tracking mode, try one from this list: 'sidereal', 'solar', 'lunar', 'custom'")
        return True

    ### addAction ###
    def addAction(self, name, param = {}):
        '''
            This function add new operation to operation buffer
        
        input:
            name: name of operation
            param: operation param in dictionary type
        output:
        '''
        #TODO: udelat kontrolu, jestli tato operace existuje
        
        self.driver_action_buffer.append({name:param})
        return True

    def tracking(self, state):
        print "tracking:", state
        if state:
            self.driver_action_status = 1
            self.driver_action_buffer.append({'tracking': {'mode': 'sidereal'} })
        else:
            self.hardStop()

    def setTrackingSpeed(self, ra, dec):
        print "tracking speed:", ra, dec
        self.trackingSpeed[0] = float(dec)
        self.trackingSpeed[1] = float(ra)

    def setMaxSpeed(self, spd):
        print "maximal speed:", spd
        self.m_ra.MaxSpeed(int(spd))
        self.m_dec.MaxSpeed(int(spd))

    def getCoordinates(self, sky = True):
        try:
            return self.mountCoord
            #return self.mount_position
        except Exception, e:
            return None

    def getStepperStatus(self):
        return (self.m_ra.getStatus(), self.m_dec.getStatus())
        #return (getStatus,{})

    def getStepperPosition(self):
        m_ra = self.m_ra.getPosition()
        m_dec = self.m_dec.getPosition()
        return (m_ra, m_dec)

    def getDefaultTrackingSpd(self):
        return self.trackingSpeed

    def getTrackingModes(self):
        '''
            This function return dictionary with supported tracking modes

            input:
            output:
                dict: dictionary with tracking modes
        '''
        return self.preset_speed

    def getStepsPerRev(self):
        return self.mount_steps

    def get_motor_steps(self):
        '''
            This function obtain count of steps in HBSTEP driver
        '''
        ra = self.m_ra.getPosition()
        dec = self.m_dec.getPosition()
        return(ra, dec)

    def get_motor_status(self, ra=True, dec=True):
        if not ra:
            ra = {}
        else:
            ra = self.m_ra.getStatus()
        
        if not dec:
            dec = {}
        else:
            dec = self.m_dec.getStatus()
        
        return(ra, dec)

    def getAxisPosition(self):
        #pos_ra = self.m_ra.getPosition()
        #pos_dec = self.m_dec.getPosition()

        pos_ra = self.m_ra.virtual_position()
        pos_dec = self.m_dec.virtual_position()


        print "*******************************************"
        print "raw", (pos_ra, pos_dec)
        #print "count1C ", ( Angle(90*u.deg)-Angle(m_ra/self.mount_stepsperdeg[0], unit="deg").wrap_at('360d') , Angle(90, unit="deg")-(Angle(pos_dec/self.mount_stepsperdeg[1], unit="deg").wrap_at('360d')) ),              Angle((90*u.deg)-Angle(m_ra/self.mount_stepsperdeg[0], unit="deg").wrap_at('360d')).to_string(unit=u.hour)
        #print "count1D ", ( Angle(90*u.deg)-Angle(m_ra/self.mount_stepsperdeg[0], unit="deg").wrap_at('360d')+(180*u.deg) , Angle(90, unit="deg")-(Angle(pos_dec/self.mount_stepsperdeg[1], unit="deg").wrap_at('360d')) ),        Angle((90*u.deg)-Angle(m_ra/self.mount_stepsperdeg[0], unit="deg").wrap_at('360d')+(180*u.deg)).to_string(unit=u.hour)

        out_a = Angle(0*u.deg)
        out_b = Angle(0*u.deg)

        if pos_ra < (2**22)/2:                                          ## kdyz je RA natoceno doprava
            x_ha = pos_ra/self.mount_stepsperdeg[0]
            if pos_dec < (2**22)/2:
                print "SEKTOR I (0-90)"
                out_a = Angle(-1*(x_ha+90)*u.deg).wrap_at('360d')
            else:
                print "SEKTOR III (180-270)"
                out_a = Angle(-1*(x_ha-90)*u.deg).wrap_at('360d')
        else:
            x_ha = (2**22-pos_ra)/self.mount_stepsperdeg[0]
            if pos_dec < (2**22)/2:
                print "SEKTOR II (90-180)"
                out_a = Angle(x_ha*u.deg+270*u.deg)
            else:
                print "SEKTOR IV (270-360)"
                out_a = Angle(x_ha*u.deg)


        LST = Time(str(Time.now()), scale='utc', location=self.observatory).sidereal_time('mean')
        print LST
        print Angle(LST)
        print out_a
        print Angle(out_a)
        try:
            print Angle(LST) - Angle(out_a)
            self.mountCoord = SkyCoord(ra = Angle(Angle(LST) - Angle(out_a)), dec = self.mount_position.dec, obstime = Time.now(), unit="deg", frame = "icrs", location = self.observatory)
            print Angle(out_b)

        except Exception, e:
            print "ERRRRRRRRRR"
            print e
            self.mountCoord = SkyCoord(ra = 0, dec = 0, obstime = Time.now(), unit="deg", frame = "icrs", location = self.observatory)
        
        print self.mountCoord
        return self.mountCoord

    def realAxisCoord(self, AxisPos = None):
        if AxisPos:
            ra, dec = AxisPos
        else:
            ra, dec = self.m_ra.getPosition(), self.m_dec.getPosition()
        ha_c = ra/self.mount_stepsperdeg[0]
        dec_c= dec/self.mount_stepsperdeg[1]

        return (ha_c, dec_c)

    def getStatus(self):
        pass

    def mThread(self):
        last_status_callback = time.time()
        m_ra_mountDir, m_dec_mountDir = self.mount_direction
        ha = 0
        while getattr(self, "run", True):
            time.sleep(0.25)
            #print("ActBuff:", len(self.driver_action_buffer), self.driver_action_now,  self.driver_action_buffer[self.driver_action_now-1])

            if self.driver_action_status in [0,1] and len(self.driver_action_buffer) > self.driver_action_now:
                # kdyz je status akce je 0 nebo 1 (dokonceno nebo lze prerusit) a v bufferu je nejaka dalsi akce
                # 999 - je novy prikaz
                # tak prejdu na dalsi akci
                self.driver_action_now += 1
                print("Mam novou akci - prechazim na dalsi: ", self.driver_action_now, self.driver_action_buffer[self.driver_action_now-1])
                self.driver_action_status = 999

            #if 'STOP1' in self.driver_action_buffer:
                #TODO: umoznit preruseni 
            
            driver_action, driver_action_params = self.driver_action_buffer[self.driver_action_now-1].items()[0]


            if driver_action == 'home' and self.driver_action_status != 0:
                # TODO: zkontrolovat cca polohu dle accel
                # TODO: kalibrace podle HALL
                print("Parking started...")
                self.m_ra.GoHome(False)
                self.m_dec.GoHome(False)
                #while self.m_ra.IsBusy() | self.m_dec.IsBusy():
                #    time.sleep(0.35)
                self.driver_action_status = 0
            
            elif driver_action == 'slew' and self.driver_action_status > 0:
                if self.driver_action_status == 999:
                    self.driver_action_status = 3 # pracuji - neprerusovat
                    print ""
                    print ""
                    print "slew mode"
                    print "============"
                    print "pozadovane souradnice: ", self.mount_target.to_string('hmsdms')
                    print "pozadovane souradnice: ", self.mount_target.to_string('dms')
                    print "AzAlt                : ", self.mount_target.transform_to(AltAz(obstime = Time.now(), location=self.observatory)).to_string('dms')

                    target = self.mount_target
                    ha, dec = self.GetHaDecCoordinates(target)

                    print "Ha [deg]  [h]        : ", ha, ha/15
                    print "Pozad pozice montaze ra,dec:", ha*self.mount_stepsperdeg[0], dec*self.mount_stepsperdeg[1]
                    print ""

                    #self.m_ra.GoTo(int(ha *self.mount_stepsperdeg[0]))
                    #self.m_dec.GoTo(int(dec*self.mount_stepsperdeg[1]))
                    
                    rs = self.m_ra.goto_virtual(int(ha*self.mount_stepsperdeg[0]))
                    rd = self.m_dec.goto_virtual(int(dec*self.mount_stepsperdeg[1]))
                    self.driver_action_status = 1

                elif self.driver_action_status == 1:
                    if rs: rs = self.m_ra.goto_virtual(int(ha*self.mount_stepsperdeg[0]))
                    if rd: rd = self.m_dec.goto_virtual(int(dec*self.mount_stepsperdeg[1]))
                    time.sleep(0.5)

                    if rs==0 and rd==0:
                        self.driver_action_buffer.append({'tracking': {'mode': 'sidereal'} })
                        self.driver_action_status = 0

                if self.driver_action_status == 0:
                    print "Slew dokonceno"
                    self.mount_position = target


            elif driver_action == 'tracking' and self.driver_action_status >= 1:
                print("tracking", driver_action_params['mode'], driver_action, self.driver_action_status >= 1)
                tracking_mode = driver_action_params['mode']
                self.driver_action_status = 1
                
                if tracking_mode == False:
                    self.driver_action_status = 0

                elif tracking_mode == 'stop':
                    self.m_ra.Float()
                    self.m_dec.Float()

                elif tracking_mode == 'custom':
                    if driver_action_params.get('units', 'steps') == 'steps':
                        self.m_ra.Run(int(m_ra_mountDir), driver_action_params.get('ra', 0))
                        self.m_dec.Run(int(m_dec_mountDir), driver_action_params.get('dec', 0))

                elif self.preset_speed.get(tracking_mode, None):
                    print("Start tracking by preset....", tracking_mode)
                    ra = self.preset_speed[tracking_mode]['ra']
                    dec = self.preset_speed[tracking_mode]['dec']
                    print('Rychlosti:', ra, dec)
                    self.m_ra.Run( int(m_ra_mountDir), ra)
                    self.m_dec.Run(int(m_dec_mountDir), dec)
                    
                elif tracking_mode == 'orbit':
                    if self.mount_position is not None and True:
                        time.sleep(0.1)
                
                self.driver_action_status = 0


            if self.status_callback and time.time() > last_status_callback + 1:
                try:
                    self.status_callback(self.mount_target, self.mount_position, self.m_ra.getStatus(), self.m_dec.getStatus())
                    last_status_callback = time.time()
                except Exception as e:
                    print(e)
                
def main():
    try:
        driver = drive(profile = 'HEQ5')
        driver.run()
        driver.UnPark()

        driver.movementMode('sidereal')
        print "A"
        time.sleep(1)
        print "B"

        #driver.Slew(SkyCoord(alt = 1, az = 90, obstime = Time.now(), frame = 'altaz', unit="deg", location = driver.getObs()).icrs)
        #driver.Slew(SkyCoord(ra = 1.3, dec = 0, obstime = Time.now(), frame = 'icrs', unit="deg", location = driver.getObs()))
        driver.Slew(SkyCoord(alt = 45, az = 45, obstime = Time.now(), frame = 'altaz', unit="deg", location = driver.getObs()).icrs)

        time.sleep(1)
        driver.GoPark()
        
        while True:
            pass

    except KeyboardInterrupt:
        print "Ukoncuji"
        driver.hardStop()
        print "Konec", Time.now()
    #drive.connect()


if __name__ == '__main__':
    main()

