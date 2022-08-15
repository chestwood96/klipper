# Support for filament width sensor
#
# Copyright (C) 2019  Mustafa YILDIZ <mydiz@hotmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from asyncio import new_event_loop
import logging
from . import filament_switch_sensor
from . import bus

MLX90395_CHIP_ADDR = 0x0C

class MLX90395HallFilamentWidthSensor:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.i2c = bus.MCU_I2C_from_config(config, default_addr=MLX90395_CHIP_ADDR, default_speed=100000)
        self.mcu = self.i2c.get_mcu()

        self.dia1=config.getfloat('Cal_dia1', 1.5)
        self.dia2=config.getfloat('Cal_dia2', 2.0)
        self.rawdia1=config.getint('Raw_dia1', 10000)
        self.rawdia2=config.getint('Raw_dia2', -10000)
        self.MEASUREMENT_INTERVAL_MM=config.getint('measurement_interval',10)
        self.nominal_filament_dia = config.getfloat('default_nominal_filament_diameter', above=1)
        self.measurement_delay = config.getfloat('measurement_delay', above=0.)
        self.measurement_max_difference = config.getfloat('max_difference', 0.2)
        self.max_diameter = (self.nominal_filament_dia + self.measurement_max_difference)
        self.min_diameter = (self.nominal_filament_dia - self.measurement_max_difference)
        self.diameter =self.nominal_filament_dia
        self.is_active =config.getboolean('enable', False)
        self.runout_dia=config.getfloat('min_diameter', 1.0)
        self.is_log =config.getboolean('logging', False)
        self.averaging =config.getint('averaging', 1)
        # Use the current diameter instead of nominal while the first
        # measurement isn't in place
        self.use_current_dia_while_delay = config.getboolean(
            'use_current_dia_while_delay', False)
        # filament array [position, filamentWidth]
        self.filament_array = []
        self.lastFilamentWidthReading = 0
        self.lastFilamentWidthReading2 = 0
        self.firstExtruderUpdatePosition = 0
        self.filament_width = self.nominal_filament_dia
        # printer objects
        self.toolhead = self.ppins = None
        self.printer.register_event_handler("klippy:connect", self.handle_connect)
        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        # extrude factor updating
        self.extrude_factor_update_timer = self.reactor.register_timer(
            self.extrude_factor_update_event)
        # Register commands
        self.gcode = self.printer.lookup_object('gcode')

        
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command('QUERY_FILAMENT_WIDTH', self.cmd_M407)
        self.gcode.register_command('RESET_FILAMENT_WIDTH_SENSOR', self.cmd_ClearFilamentArray)
        self.gcode.register_command('DISABLE_FILAMENT_WIDTH_SENSOR', self.cmd_M406)
        self.gcode.register_command('ENABLE_FILAMENT_WIDTH_SENSOR', self.cmd_M405)
        self.gcode.register_command('QUERY_RAW_FILAMENT_WIDTH', self.cmd_Get_Raw_Values)
        self.gcode.register_command('ENABLE_FILAMENT_WIDTH_LOG', self.cmd_log_enable)
        self.gcode.register_command('DISABLE_FILAMENT_WIDTH_LOG', self.cmd_log_disable)

        self.runout_helper = filament_switch_sensor.RunoutHelper(config)
    # Initialization
    def handle_connect(self):
        self._init_sensor()

    def handle_ready(self):
        # Load printer objects
        self.toolhead = self.printer.lookup_object('toolhead')

        # Start extrude factor update timer
        self.reactor.update_timer(self.extrude_factor_update_timer, self.reactor.NOW)

    def update_filament_array(self, last_epos):
        # Fill array
        if len(self.filament_array) > 0:
            # Get last reading position in array & calculate next
            # reading position
          next_reading_position = (self.filament_array[-1][0] +
          self.MEASUREMENT_INTERVAL_MM)
          if next_reading_position <= (last_epos + self.measurement_delay):
            self.filament_array.append([last_epos + self.measurement_delay,
                                            self.diameter])
            if self.is_log:
                 self.gcode.respond_info("Filament width:%.3f" %
                                         ( self.diameter ))

        else:
            # add first item to array
            self.filament_array.append([self.measurement_delay + last_epos,
                                        self.diameter])
            self.firstExtruderUpdatePosition = (self.measurement_delay
                                                + last_epos)

    def extrude_factor_update_event(self, eventtime):
        self.diameter = self.read_diameter()
        logging.info(f"DERP {self.diameter}")
        # Update extrude factor
        pos = self.toolhead.get_position()
        last_epos = pos[3]
        # Update filament array for lastFilamentWidthReading
        self.update_filament_array(last_epos)
        # Check runout
        self.runout_helper.note_filament_present(
            self.diameter > self.runout_dia)
        # Does filament exists
        if self.diameter > 0.5:
            if len(self.filament_array) > 0:
                # Get first position in filament array
                pending_position = self.filament_array[0][0]
                if pending_position <= last_epos:
                    # Get first item in filament_array queue
                    item = self.filament_array.pop(0)
                    self.filament_width = item[1]
                else:
                    if ((self.use_current_dia_while_delay)
                        and (self.firstExtruderUpdatePosition
                             == pending_position)):
                        self.filament_width = self.diameter
                    elif  self.firstExtruderUpdatePosition == pending_position:
                        self.filament_width = self.nominal_filament_dia
                if ((self.filament_width <= self.max_diameter)
                    and (self.filament_width >= self.min_diameter)):
                    percentage = round(self.nominal_filament_dia**2
                                       / self.filament_width**2 * 100)
                    self.gcode.run_script("M221 S" + str(percentage))
                else:
                    self.gcode.run_script("M221 S100")
        else:
            self.gcode.run_script("M221 S100")
            self.filament_array = []

        if self.is_active:
            return eventtime + 1
        else:
            return self.reactor.NEVER

    def cmd_M407(self, gcmd):
        response = ""
        if self.diameter > 0:
            response += ("Filament dia (measured mm): "
                         + str(self.diameter))
        else:
            response += "Filament NOT present"
        gcmd.respond_info(response)

    def cmd_ClearFilamentArray(self, gcmd):
        self.filament_array = []
        gcmd.respond_info("Filament width measurements cleared!")
        # Set extrude multiplier to 100%
        self.gcode.run_script_from_command("M221 S100")

    def cmd_M405(self, gcmd):
        response = "Filament width sensor Turned On"
        if self.is_active:
            response = "Filament width sensor is already On"
        else:
            self.is_active = True
            # Start extrude factor update timer
            self.reactor.update_timer(self.extrude_factor_update_timer,
                                      self.reactor.NOW)
        gcmd.respond_info(response)

    def cmd_M406(self, gcmd):
        response = "Filament width sensor Turned Off"
        if not self.is_active:
            response = "Filament width sensor is already Off"
        else:
            self.is_active = False
            # Stop extrude factor update timer
            self.reactor.update_timer(self.extrude_factor_update_timer,
                                      self.reactor.NEVER)
            # Clear filament array
            self.filament_array = []
            # Set extrude multiplier to 100%
            self.gcode.run_script_from_command("M221 S100")
        gcmd.respond_info(response)

    def cmd_Get_Raw_Values(self, gcmd):
        response = "RAW=" + str(self.lastFilamentWidthReading)
        gcmd.respond_info(response)
    def get_status(self, eventtime):
        return {'Diameter': self.diameter,
                'Raw':(self.lastFilamentWidthReading),
                'is_active':self.is_active}
    def cmd_log_enable(self, gcmd):
        self.is_log = True
        gcmd.respond_info("Filament width logging Turned On")

    def cmd_log_disable(self, gcmd):
        self.is_log = False
        gcmd.respond_info("Filament width logging Turned Off")

    def _init_sensor(self):
        self.execute_command(0xF0) # Soft reset
        self.reactor.pause(self.reactor.monotonic() + .01)
        self.execute_command(0x80) # Exit mode (may not be nessecary)
        self.reactor.pause(self.reactor.monotonic() + .01)
        logging.info("MLX90395 with ID:" + self.read_register(0x26, 6).hex() + " found") # Read chip ID
        self.write_register_bits(0x01, 15, 1, 0x00) # Disable the int pin output incase it is shorted to gnd # TODO configurable?
        self.write_register_bits(0x00, 4, 4, 0x00) # Set gain # TODO configurable
        self.write_register_bits(0x02, 9, 2, 0x00) # Set res # TODO configurable and configurable axis
        self.write_register_bits(0x02, 0, 2, 0x03) # Set osr # TODO configurable
        self.write_register_bits(0x02, 2, 3, 0x07) # Set averaging # TODO configurable
        self.execute_command(0x10 | 0x08) # Start burst mode on the Z axis # TODO configurable axis

    def read_diameter(self):
        raw = self.read_raw_val()
        return round((self.dia2 - self.dia1) / (self.rawdia2 - self.rawdia1) * (raw - self.rawdia1) + self.dia1 , 3)

    def read_raw_val(self):
        accumulator = 0
        for x in range(self.averaging):
            answer = self.read_register(0x80 >> 1, 12)
            while(answer[0] & 0x01 > 0): # Retry if data isn't fresh
                answer = self.read_register(0x80 >> 1, 12)
            accumulator += int.from_bytes([answer[6], answer[7]], "big", signed=True) # TODO configurable axis
        value = int(accumulator / self.averaging)
        self.lastFilamentWidthReading = value
        return value

    def read_register(self, reg, read_len):
        params = self.i2c.i2c_read([reg << 1], read_len)
        return bytearray(params['response'])

    def write_register(self, reg, data):
        if type(data) is not list:
            data = [data]
        data.insert(0, reg << 1)
        self.i2c.i2c_write(data)

    def write_register_bits(self, reg, offset, length, data):
        register_value = int.from_bytes(self.read_register(reg, 2), "big") # read whole register
        #logging.info(f"{register_value:16b}")
        mask = (pow(2, length)-1 << offset)
        register_value = register_value & (0xFFFF ^ mask) # zero the requested range
        register_value = register_value | ((data << offset) & mask) # insert data at requested range
        #logging.info(f"{register_value:16b}")
        self.write_register(reg, list(register_value.to_bytes(2, 'big'))) # write
        new_register_value = int.from_bytes(self.read_register(reg, 2), "big") # read whole register
        if register_value != new_register_value:
            logging.error(f"Setting {offset}:{length} in {reg} to {data} failed, expected {register_value:16b} got {new_register_value:16b}")
    
    def execute_command(self, command):
        if type(command) is not list:
            command = [command]
        command.insert(0, 0x80)
        self.i2c.i2c_write(command)

def load_config(config):
    return MLX90395HallFilamentWidthSensor(config)
