#!/usr/bin/env python
"""
Created by Cory Stewart on 2013-12-30
Based on the i2c-test.py script by Peter Huewe
and on the ina219 arduino library by adafruit.
"""
import os
import sys
from pyBusPirateLite.I2C import *

class Ina219():
    # =========================================================================
    # I2C ADDRESS/BITS
    # -------------------------------------------------------------------------
    INA219_ADDRESS                         = (0x40)    # 1000000 (A0+A1=GND)
    INA219_READ                            = (0x01)
    # =========================================================================

    # =========================================================================
    # CONFIG REGISTER (R/W)
    # -------------------------------------------------------------------------
    INA219_REG_CONFIG                      = (0x00)
    # -------------------------------------------------------------------------
    INA219_CONFIG_RESET                    = (0x8000)  # Reset Bit

    INA219_CONFIG_BVOLTAGERANGE_MASK       = (0x2000)  # Bus Voltage Range Mask
    INA219_CONFIG_BVOLTAGERANGE_16V        = (0x0000)  # 0-16V Range
    INA219_CONFIG_BVOLTAGERANGE_32V        = (0x2000)  # 0-32V Range

    INA219_CONFIG_GAIN_MASK                = (0x1800)  # Gain Mask
    INA219_CONFIG_GAIN_1_40MV              = (0x0000)  # Gain 1, 40mV Range
    INA219_CONFIG_GAIN_2_80MV              = (0x0800)  # Gain 2, 80mV Range
    INA219_CONFIG_GAIN_4_160MV             = (0x1000)  # Gain 4, 160mV Range
    INA219_CONFIG_GAIN_8_320MV             = (0x1800)  # Gain 8, 320mV Range

    INA219_CONFIG_BADCRES_MASK             = (0x0780)  # Bus ADC Resolution Mask
    INA219_CONFIG_BADCRES_9BIT             = (0x0080)  # 9-bit bus res = 0..511
    INA219_CONFIG_BADCRES_10BIT            = (0x0100)  # 10-bit bus res = 0..1023
    INA219_CONFIG_BADCRES_11BIT            = (0x0200)  # 11-bit bus res = 0..2047
    INA219_CONFIG_BADCRES_12BIT            = (0x0400)  # 12-bit bus res = 0..4097
    
    INA219_CONFIG_SADCRES_MASK             = (0x0078)  # Shunt ADC Resolution and Averaging Mask
    INA219_CONFIG_SADCRES_9BIT_1S_84US     = (0x0000)  # 1 x 9-bit shunt sample
    INA219_CONFIG_SADCRES_10BIT_1S_148US   = (0x0008)  # 1 x 10-bit shunt sample
    INA219_CONFIG_SADCRES_11BIT_1S_276US   = (0x0010)  # 1 x 11-bit shunt sample
    INA219_CONFIG_SADCRES_12BIT_1S_532US   = (0x0018)  # 1 x 12-bit shunt sample
    INA219_CONFIG_SADCRES_12BIT_2S_1060US  = (0x0048)  # 2 x 12-bit shunt samples averaged together
    INA219_CONFIG_SADCRES_12BIT_4S_2130US  = (0x0050)  # 4 x 12-bit shunt samples averaged together
    INA219_CONFIG_SADCRES_12BIT_8S_4260US  = (0x0058)  # 8 x 12-bit shunt samples averaged together
    INA219_CONFIG_SADCRES_12BIT_16S_8510US = (0x0060)  # 16 x 12-bit shunt samples averaged together
    INA219_CONFIG_SADCRES_12BIT_32S_17MS   = (0x0068)  # 32 x 12-bit shunt samples averaged together
    INA219_CONFIG_SADCRES_12BIT_64S_34MS   = (0x0070)  # 64 x 12-bit shunt samples averaged together
    INA219_CONFIG_SADCRES_12BIT_128S_69MS  = (0x0078)  # 128 x 12-bit shunt samples averaged together

    INA219_CONFIG_MODE_MASK                = (0x0007)  # Operating Mode Mask
    INA219_CONFIG_MODE_POWERDOWN           = (0x0000)
    INA219_CONFIG_MODE_SVOLT_TRIGGERED     = (0x0001)
    INA219_CONFIG_MODE_BVOLT_TRIGGERED     = (0x0002)
    INA219_CONFIG_MODE_SANDBVOLT_TRIGGERED = (0x0003)
    INA219_CONFIG_MODE_ADCOFF              = (0x0004)
    INA219_CONFIG_MODE_SVOLT_CONTINUOUS    = (0x0005)
    INA219_CONFIG_MODE_BVOLT_CONTINUOUS    = (0x0006)
    INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS = (0x0007)
    # =========================================================================

    # =========================================================================
    # SHUNT VOLTAGE REGISTER (R)
    # -------------------------------------------------------------------------
    INA219_REG_SHUNTVOLTAGE                = (0x01)
    # =========================================================================

    # =========================================================================
    # BUS VOLTAGE REGISTER (R)
    # -------------------------------------------------------------------------
    INA219_REG_BUSVOLTAGE                  = (0x02)
    # =========================================================================

    # =========================================================================
    # POWER REGISTER (R)
    # -------------------------------------------------------------------------
    INA219_REG_POWER                       = (0x03)
    # =========================================================================

    # =========================================================================
    # CURRENT REGISTER (R)
    # -------------------------------------------------------------------------
    INA219_REG_CURRENT                     = (0x04)
    # =========================================================================

    # =========================================================================
    # CALIBRATION REGISTER (R/W)
    # -------------------------------------------------------------------------
    INA219_REG_CALIBRATION                 = (0x05)
    # =========================================================================

    # ----------------------------------------------------------------
    def _i2c_write_data(self, data):
        #print '_i2c_write_data(', data, ')'
        self.i2c.send_start_bit()
        self.i2c.bulk_trans(len(data),data)
        self.i2c.send_stop_bit()

    # ----------------------------------------------------------------
    def _i2c_read_bytes(self, address, numbytes, ret=False):
        t = (address, numbytes, ret)
        #print '_i2c_read_bytes(', t, ')'
        data_out=[]
        self.i2c.send_start_bit()
        self.i2c.bulk_trans(len(address),address)
        while numbytes > 0:
            if not ret:
                print '0x%x' % ( ord(self.i2c.read_byte()) )
            else:
                data_out.append(ord(self.i2c.read_byte()))
            if numbytes > 1:
                self.i2c.send_ack()
            numbytes-=1
        self.i2c.send_nack()
        self.i2c.send_stop_bit()
        if ret:
            return data_out

    def _write_ina219( self, addr, val ):
        #print 'Writing Reg @(0x%x) = (0x%x,0x%x)' % (addr, val >> 8 & 0xff, val & 0xff )
        self._i2c_write_data( [self.INA219_ADDRESS << 1, addr, val >> 8 & 0xff, val & 0xff] )

    # ----------------------------------------------------------------
    def _read_ina219( self, addr, return_it=False ):
        #print 'Reading @(0x%x)' % (addr)
        self._i2c_write_data( [self.INA219_ADDRESS << 1, addr] )
        val = self._i2c_read_bytes( [self.INA219_ADDRESS << 1 | self.INA219_READ], 2, return_it ) # read 2 bytes
        if return_it:
            return val

    # ----------------------------------------------------------------
    def _bytelist2int( self, bytelist ):
        bytelist.reverse()
        intval = 0
        for (i, val) in enumerate( bytelist ):
            #print 'i=', i, 'val=', val
            intval += val * 2 ** (i*8)
        return intval
        
    # ----------------------------------------------------------------
    # Public methods:
    #def __init__( self, i2c, debug=False ):
    def __init__( self, ser_port, debug=False ):
        self.serialport = ser_port
        print 'self.serialport = ', self.serialport
        self.i2c = I2C( self.serialport, 115200 )

        if self.i2c == None:
            print '** ERROR ** : INA219 constructor passed a handle to an I2C object handle = None.'
            sys.exit()

        if debug: print "Entering binmode: ",
        if self.i2c.BBmode():
            if debug: print "OK."
        else:
            print "failed to enter binmode on the Bus Pirate."
            sys.exit()

        if debug: print "Entering raw I2C mode: ",
        if self.i2c.enter_I2C():
            if debug: print "OK."
        else:
            print "failed to enter I2C raw mode on the Bus Pirate."
            sys.exit()
            
        if debug: print "Configuring I2C."
        if not self.i2c.cfg_pins(I2CPins.POWER | I2CPins.PULLUPS):
            print "Failed to set I2C pin configuration on the Bus Pirate."
            sys.exit()
        if not self.i2c.set_speed(I2CSpeed._100KHZ):
            print "Failed to set I2C Speed on the Bus Pirate."
            sys.exit()
        self.i2c.timeout(0.2)

    def __repr__( self ):
        "Returns a string suitable for printing with the names and values of all 6 INA219 Registers."
        str = ''
        # Read addr = 0x00
        reg_addr = self.INA219_REG_CONFIG
        str += '    INA219_REG_CONFIG (addr=0x%x): ' % (reg_addr)
        self._i2c_write_data( [self.INA219_ADDRESS << 1, reg_addr] )
        val16 = self._bytelist2int( self._i2c_read_bytes( [self.INA219_ADDRESS << 1 | self.INA219_READ], 2, True ) )
        str += '0x%04x\n' % (val16)

        # Read addr = 0x01
        reg_addr = self.INA219_REG_SHUNTVOLTAGE
        str += '    INA219_REG_SHUNTVOLTAGE (addr=0x%x): ' % (reg_addr)
        self._i2c_write_data( [self.INA219_ADDRESS << 1, reg_addr] )
        val16 = self._bytelist2int( self._i2c_read_bytes( [self.INA219_ADDRESS << 1 | self.INA219_READ], 2, True ) )
        str += '0x%04x\n' % (val16)

        # Read addr = 0x02
        reg_addr = self.INA219_REG_BUSVOLTAGE
        str += '    INA219_REG_BUSVOLTAGE (addr=0x%x): ' % (reg_addr)
        self._i2c_write_data( [self.INA219_ADDRESS << 1, reg_addr] )
        val16 = self._bytelist2int( self._i2c_read_bytes( [self.INA219_ADDRESS << 1 | self.INA219_READ], 2, True ) )
        str += '0x%04x\n' % (val16)

        # Read addr = 0x03
        reg_addr = self.INA219_REG_POWER
        str += '    INA219_REG_POWER (addr=0x%x): ' % (reg_addr)
        self._i2c_write_data( [self.INA219_ADDRESS << 1, reg_addr] )
        val16 = self._bytelist2int( self._i2c_read_bytes( [self.INA219_ADDRESS << 1 | self.INA219_READ], 2, True ) )
        str += '0x%04x\n' % (val16)

        # Read addr = 0x04
        reg_addr = self.INA219_REG_CURRENT
        str += '    INA219_REG_CURRENT (addr=0x%x): ' % (reg_addr)
        self._i2c_write_data( [self.INA219_ADDRESS << 1, reg_addr] )
        val16 = self._bytelist2int( self._i2c_read_bytes( [self.INA219_ADDRESS << 1 | self.INA219_READ], 2, True ) )
        str += '0x%04x\n' % (val16)

        # Read addr = 0x05
        reg_addr = self.INA219_REG_CALIBRATION
        str += '    INA219_REG_CALIBRATION (addr=0x%x): ' % (reg_addr)
        self._i2c_write_data( [self.INA219_ADDRESS << 1, reg_addr] )
        val16 = self._bytelist2int( self._i2c_read_bytes( [self.INA219_ADDRESS << 1 | self.INA219_READ], 2, True ) )
        str += '0x%04x' % (val16)

        return str

    # ----------------------------------------------------------------
    def close( self, print_it=False ):
        """
        Ina219.close( self, print_it=False )
            Gracefully shuts down the Bus Pirate.
            Tristates all outputs and removes power from the +5V and 3V3 pins.
            Optionally prints out result msg.
        """
        if print_it: print "Reset Bus Pirate: ",
        if self.i2c.resetBP():
            if print_it: print "OK."
        else:
            if print_it: print "failed."

    # ----------------------------------------------------------------
    def getBusVoltage_raw( self ):
        "Gets the raw bus voltage (16-bit signed integer, so +-32767)"
        val_list = self._read_ina219( self.INA219_REG_BUSVOLTAGE, True )
        val16 = self._bytelist2int( val_list )
        #print 'getBusVoltage_raw() list:', val_list
        #print 'getBusVoltage_raw() val16:', val16
        # Bus Voltage is in bits [15:3].  Shift to the right 3 to 
        # drop [1] CNVR (conversion ready) and [0] OVF (overflow) bits,
        # and then multiply by Bus voltage LSB (which is always 4mV)
        val = (val16 >> 3) * 4
        #print 'getBusVoltage_raw() val:', val
        return val

    # ----------------------------------------------------------------
    def getShuntVoltage_raw( self ):
        "Gets the raw shunt voltage (16-bit signed integer, so +-32767)"
        val_list = self._read_ina219( self.INA219_REG_SHUNTVOLTAGE, True )
        val16 = self._bytelist2int( val_list )
        return val16

    # ----------------------------------------------------------------
    def getCurrent_raw( self ):
        "Gets the raw current value (16-bit signed integer, so +-32767)"
        # Sometimes a sharp load will reset the INA219, which will
        # reset the cal register, meaning CURRENT and POWER will
        # not be available ... avoid this by always setting a cal
        # value even if it's an unfortunate extra step
        self._write_ina219( self.INA219_REG_CALIBRATION, self.ina219_calValue )

        # Now we can safely read the CURRENT register!
        val_list = self._read_ina219( self.INA219_REG_CURRENT, True )
        val16 = self._bytelist2int( val_list )
        return val16

    # ----------------------------------------------------------------
    def getBusVoltage_V( self ):
        "Gets the bus voltage in volts"
        val = self.getBusVoltage_raw()
        return val * 0.001

    # ----------------------------------------------------------------
    def getShuntVoltage_mV( self ):
        "Gets the shunt voltage in mV (so +-327mV)"
        val = self.getShuntVoltage_raw()
        return val * 0.01

    # ----------------------------------------------------------------
    def getCurrent_mA( self ):
        """
        Gets the current value in mA, taking into account the config settings and current LSB
        """
        val = self.getCurrent_raw()
        val /= float( self.ina219_currentDivider_mA )
        return val

    # ----------------------------------------------------------------
    def ina219SetCalibration_16V_500mA( self ):  # for USB2PA
        "Sets the calibration register for fullscale at 16V and 500mA"
        # By default we use a pretty huge range for the input voltage,
        # which probably isn't the most appropriate choice for system
        # that don't use a lot of power.  But all of the calculations
        # are shown below if you want to change the settings.  You will
        # also need to change any relevant register settings, such as
        # setting the VBUS_MAX to 16V instead of 32V, etc.

        # VBUS_MAX = 16V             (32V or 16V)
        # VSHUNT_MAX = 0.32          (PGA Gain 8, 4, 2, 1 corresponding to: 0.32, 0.16, 0.08, or 0.04 vshunt)
        # RSHUNT = 0.5   1%          (USB2PA Resistor value in ohms)

        # 1. Determine max possible current
        # MaxPossible_I = VSHUNT_MAX / RSHUNT = 0.32 / 0.5
        # MaxPossible_I = 640mA

        # 2. Determine max expected current
        # MaxExpected_I = 500mA

        # 3. Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
        # MinimumLSB = MaxExpected_I/32767 = 0.5 / 32767 = 15.259255E-6
        # MinimumLSB = 0.000015259           (15.26uA per bit)
        # MaximumLSB = MaxExpected_I/4096 = 0.5 / 4096 = 122.070312E-6
        # MaximumLSB = 0.0001220703          (122.07uA per bit)

        # 4. Choose an LSB between the min and max values
        #    (Preferrably a roundish number close to MinLSB)
        # CurrentLSB = 0.000016 (16.0uA per bit)

        # 5. Compute the calibration register
        # Cal = trunc (0.04096 / (Current_LSB * RSHUNT))
        # Cal = trunc (0.04096 / (16e-6 * 0.5))
        # Cal = 5120 (0x1400)

        self.ina219_calValue = 5120

        # 6. Calculate the power LSB
        # PowerLSB = 20 * CurrentLSB = 20 * 16e-6
        # PowerLSB = 0.00032 (320uW per bit)

        # 7. Compute the maximum current and shunt voltage values before overflow
        #
        # Max_Current = Current_LSB * 32767 = 16e-6 * 32767
        # Max_Current = 524.3mA before overflow
        #
        # If Max_Current > Max_Possible_I then
        #    Max_Current_Before_Overflow = MaxPossible_I
        # Else
        #    Max_Current_Before_Overflow = Max_Current
        # End If
        #
        # Max_ShuntVoltage = Max_Current_Before_Overflow * RSHUNT = 524.3mA * 0.5
        # Max_ShuntVoltage = 0.2621V
        #
        # If Max_ShuntVoltage >= VSHUNT_MAX
        #    Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
        # Else
        #    Max_ShuntVoltage_Before_Overflow = Max_ShuntVoltage
        # End If

        # 8. Compute the Maximum Power
        # MaximumPower = Max_Current_Before_Overflow * VBUS_MAX = 524.3mA * 16V
        # MaximumPower = 8.39W

        # Set multipliers to convert raw current/power values
        self.ina219_currentDivider_mA = 62.5  # Current LSB = 16uA per bit (1000/16 = 62.5)
        self.ina219_powerDivider_mW = 2     # Power LSB = 320uW per bit (2/1)  ****** Probably wrong - check again ***

        # Set Calibration register to 'Cal' calculated above	
        #wireWriteRegister(INA219_REG_CALIBRATION, ina219_calValue);
        self._write_ina219( self.INA219_REG_CALIBRATION, self.ina219_calValue )

        ## Set Config register to take into account the settings above
        config_val = self.INA219_CONFIG_BVOLTAGERANGE_16V | \
                          self.INA219_CONFIG_GAIN_8_320MV | \
                          self.INA219_CONFIG_BADCRES_12BIT | \
                          self.INA219_CONFIG_SADCRES_12BIT_1S_532US | \
                          self.INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
        #wireWriteRegister(INA219_REG_CONFIG, config);
        self._write_ina219( self.INA219_REG_CONFIG, config_val )

    # ----------------------------------------------------------------
    def ina219SetCalibration_32V_2A( self ):
        "Sets the calibration register for fullscale at 32V and 2A"
        # By default we use a pretty huge range for the input voltage,
        # which probably isn't the most appropriate choice for system
        # that don't use a lot of power.  But all of the calculations
        # are shown below if you want to change the settings.  You will
        # also need to change any relevant register settings, such as
        # setting the VBUS_MAX to 16V instead of 32V, etc.

        # VBUS_MAX = 32V             (Assumes 32V, can also be set to 16V)
        # VSHUNT_MAX = 0.32          (Assumes Gain 8, 320mV, can also be 0.16, 0.08, 0.04)
        # RSHUNT = 0.1               (Resistor value in ohms)

        # 1. Determine max possible current
        # MaxPossible_I = VSHUNT_MAX / RSHUNT
        # MaxPossible_I = 3.2A

        # 2. Determine max expected current
        # MaxExpected_I = 2.0A

        # 3. Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
        # MinimumLSB = MaxExpected_I/32767
        # MinimumLSB = 0.000061              (61uA per bit)
        # MaximumLSB = MaxExpected_I/4096
        # MaximumLSB = 0,000488              (488uA per bit)

        # 4. Choose an LSB between the min and max values
        #    (Preferrably a roundish number close to MinLSB)
        # CurrentLSB = 0.0001 (100uA per bit)

        # 5. Compute the calibration register
        # Cal = trunc (0.04096 / (Current_LSB * RSHUNT))
        # Cal = 4096 (0x1000)

        self.ina219_calValue = 4096

        # 6. Calculate the power LSB
        # PowerLSB = 20 * CurrentLSB
        # PowerLSB = 0.002 (2mW per bit)

        # 7. Compute the maximum current and shunt voltage values before overflow
        #
        # Max_Current = Current_LSB * 32767
        # Max_Current = 3.2767A before overflow
        #
        # If Max_Current > Max_Possible_I then
        #    Max_Current_Before_Overflow = MaxPossible_I
        # Else
        #    Max_Current_Before_Overflow = Max_Current
        # End If
        #
        # Max_ShuntVoltage = Max_Current_Before_Overflow * RSHUNT
        # Max_ShuntVoltage = 0.32V
        #
        # If Max_ShuntVoltage >= VSHUNT_MAX
        #    Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
        # Else
        #    Max_ShuntVoltage_Before_Overflow = Max_ShuntVoltage
        # End If

        # 8. Compute the Maximum Power
        # MaximumPower = Max_Current_Before_Overflow * VBUS_MAX
        # MaximumPower = 3.2 * 32V
        # MaximumPower = 102.4W

        # Set multipliers to convert raw current/power values
        self.ina219_currentDivider_mA = 10  # Current LSB = 100uA per bit (1000/100 = 10)
        self.ina219_powerDivider_mW = 2     # Power LSB = 1mW per bit (2/1)

        # Set Calibration register to 'Cal' calculated above	
        #wireWriteRegister(INA219_REG_CALIBRATION, ina219_calValue);
        self._write_ina219( self.INA219_REG_CALIBRATION, self.ina219_calValue )

        ## Set Config register to take into account the settings above
        config_val = self.INA219_CONFIG_BVOLTAGERANGE_32V | \
                          self.INA219_CONFIG_GAIN_8_320MV | \
                          self.INA219_CONFIG_BADCRES_12BIT | \
                          self.INA219_CONFIG_SADCRES_12BIT_1S_532US | \
                          self.INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
        #wireWriteRegister(INA219_REG_CONFIG, config);
        self._write_ina219( self.INA219_REG_CONFIG, config_val )

    # ----------------------------------------------------------------
    def ina219SetCalibration_32V_1A( self ):
        "Sets the calibration register for fullscale at 32V and 1A"
        # By default we use a pretty huge range for the input voltage,
        # which probably isn't the most appropriate choice for system
        # that don't use a lot of power.  But all of the calculations
        # are shown below if you want to change the settings.  You will
        # also need to change any relevant register settings, such as
        # setting the VBUS_MAX to 16V instead of 32V, etc.

        # VBUS_MAX = 32V		(Assumes 32V, can also be set to 16V)
        # VSHUNT_MAX = 0.32	(Assumes Gain 8, 320mV, can also be 0.16, 0.08, 0.04)
        # RSHUNT = 0.1			(Resistor value in ohms)

        # 1. Determine max possible current
        # MaxPossible_I = VSHUNT_MAX / RSHUNT
        # MaxPossible_I = 3.2A

        # 2. Determine max expected current
        # MaxExpected_I = 1.0A

        # 3. Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
        # MinimumLSB = MaxExpected_I/32767
        # MinimumLSB = 0.0000305             (30.5uA per bit)
        # MaximumLSB = MaxExpected_I/4096
        # MaximumLSB = 0.000244              (244uA per bit)

        # 4. Choose an LSB between the min and max values
        #    (Preferrably a roundish number close to MinLSB)
        # CurrentLSB = 0.0000400 (40uA per bit)

        # 5. Compute the calibration register
        # Cal = trunc (0.04096 / (Current_LSB * RSHUNT))
        # Cal = 10240 (0x2800)

        self.ina219_calValue = 10240
      
        # 6. Calculate the power LSB
        # PowerLSB = 20 * CurrentLSB
        # PowerLSB = 0.0008 (800uW per bit)

        # 7. Compute the maximum current and shunt voltage values before overflow
        #
        # Max_Current = Current_LSB * 32767
        # Max_Current = 1.31068A before overflow
        #
        # If Max_Current > Max_Possible_I then
        #    Max_Current_Before_Overflow = MaxPossible_I
        # Else
        #    Max_Current_Before_Overflow = Max_Current
        # End If
        #
        # ... In this case, we're good though since Max_Current is less than MaxPossible_I
        #
        # Max_ShuntVoltage = Max_Current_Before_Overflow * RSHUNT
        # Max_ShuntVoltage = 0.131068V
        #
        # If Max_ShuntVoltage >= VSHUNT_MAX
        #    Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
        # Else
        #    Max_ShuntVoltage_Before_Overflow = Max_ShuntVoltage
        # End If

        # 8. Compute the Maximum Power
        # MaximumPower = Max_Current_Before_Overflow * VBUS_MAX
        # MaximumPower = 1.31068 * 32V
        # MaximumPower = 41.94176W

        # Set multipliers to convert raw current/power values
        self.ina219_currentDivider_mA = 25      # Current LSB = 40uA per bit (1000/40 = 25)
        self.ina219_powerDivider_mW = 1         # Power LSB = 800uW per bit

        # Set Calibration register to 'Cal' calculated above	
        #wireWriteRegister(INA219_REG_CALIBRATION, ina219_calValue);
        self._write_ina219( self.INA219_REG_CALIBRATION, self.ina219_calValue )

        # Set Config register to take into account the settings above
        config_val = self.INA219_CONFIG_BVOLTAGERANGE_32V | \
                        self.INA219_CONFIG_GAIN_8_320MV | \
                        self.INA219_CONFIG_BADCRES_12BIT | \
                        self.INA219_CONFIG_SADCRES_12BIT_1S_532US | \
                        self.INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS
        #wireWriteRegister(INA219_REG_CONFIG, config);
        self._write_ina219( self.INA219_REG_CONFIG, config_val )

    # ----------------------------------------------------------------

def findSerialPort():
    if( os.name == 'posix' ): # mac os x
        if( sys.platform.startswith( 'linux' ) ):
            # Linux
            serial_port = '/dev/serial/by-id/usb-FTDI_Dual_RS232-if01-port0'
        else:
            # Mac
            serial_port = '/dev/cu.usbserial-A901LN3O'
    else:
        serial_port = 'COM4'
    return serial_port

if __name__ == '__main__':
    ser_port = findSerialPort();
    print 'Instantiating Ina219( %s )...' % (ser_port)
    ina219 = Ina219( ser_port )

    print 'print( ina219 ):\n', ina219

    print 'ina219SetCalibration_16V_500mA()'
    ina219.ina219SetCalibration_16V_500mA()

    print 'getBusVoltage_raw(): ',          str( ina219.getBusVoltage_raw() )
    print 'getShuntVoltage_raw(): ',        str( ina219.getShuntVoltage_raw() )
    print 'getCurrent_raw(): ',             str( ina219.getCurrent_raw() )
    print 'getBusVoltage_V(): ',            str( ina219.getBusVoltage_V() )
    print 'getShuntVoltage_mV(): ',         str( ina219.getShuntVoltage_mV() )
    print 'getCurrent_mA(): ',              str( ina219.getCurrent_mA() )

    ina219.close()
    sys.exit()
