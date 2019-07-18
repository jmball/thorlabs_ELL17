# created by James Ball, 17-07-2019

import serial


def dec2hex(x, bits):
    """Generate hex string of a signed decimal int using 2's complement.
    
    Parameters
    ----------
    x : int
        signed decimal integer
    bits : int
        number of bits required in representation
    
    Returns
    -------
    hexstr : str
        hex string
    """
    if x > 0:
        return f'{x:0{bits // 4}x}'
    else:
        return f'{2**bits-abs(x):0{bits // 4}x}'


def hex2dec(x):
    """Generate signed decimal int from hex string using 2's complement.
    
    Parameters
    ----------
    x : int
        hex string
    
    Returns
    -------
    dec : int
        signed decimal integer
    """
    bits = 4 * len(x)
    if int(x, 16) < 2**(bits-1):
        return int(x, 16)
    else:
        return - (2**bits - int(x, 16))


class ell17():

    err_msg_lst = ['no error',
                    'communication time out',
                    'mechanical time out',
                    'command error or not supported',
                    'value out of range',
                    'module isolated',
                    'module out of isolation',
                    'initializing error',
                    'thermal error',
                    'busy',
                    'sensor error (may appear during self test. If code persists there is an error)',
                    'motor error (may appear during self test. If code persists there is an error)',
                    'out of range (e.g. stage has been instructed to move beyond its travel range)',
                    'over current error']

    def __init__(self, port='COM4', baud=9600, timeout=2):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.ser = serial.Serial(port, baud, timeout=timeout)
        # find address
        self.ser.timeout = 0.1
        for a in '0123456789ABCDEF':
            self.ser.write(bytes(f'{a}gs', 'ascii'))
            r = self.ser.read(7)
            if len(r) > 0:
                self.addr = chr(r[0])
                break
        self.ser.timeout = self.timeout
        self.get_info()
        self.get_motor_info(1)
        self.get_motor_info(2)
        self.get_pos()
        self.get_velocity()
        self.get_jog_stepsize()


    def __del__(self):
        pass


    def _query_msg(self, cmd, resp_len, *args):
        """Send message to stage and read response"""
        msg = bytes(f'{self.addr}', 'ascii') + bytes(cmd, 'ascii')
        if len(args) > 0:
            for arg in args:
                msg += bytes(f'{arg}', 'ascii')
        self.ser.write(msg)
        resp = self.ser.read(resp_len)
        try:
            r_addr = chr(resp[0])
        except IndexError:
            # for positions around zero the sensor sometimes struggles to return
            # in time, so returns nothing. Therefore, extend timeout and get status.
            self.ser.timeout = 10
            self.ser.write(bytes(f'{self.addr}gs', 'ascii'))
            resp = self.ser.read(7)
            self.ser.timeout = self.timeout
            r_addr = chr(resp[0])
        r_cmd = resp[1:3].decode('ascii')
        return resp, r_addr, r_cmd


    def _handle_status(self, resp, r_addr, r_cmd):
        """Handle response of get_status"""
        self.err = int(resp[3:5], 16)
        if self.err >= 14:
            self.err_msg = 'reserved'
        else:
            self.err_msg = self.err_msg_lst[self.err]
        s_resp = (f'Status:\n'
            f'-------\n'
            f'addr={r_addr}\n'
            f'cmd={r_cmd}\n'
            f'error={self.err}, {self.err_msg}\n')
        print(s_resp)


    def get_info(self):
        """Get stage information"""
        cmd = 'in'
        resp_len = 35
        resp, r_addr, r_cmd = self._query_msg(cmd, resp_len)
        self.model = int(resp[3:5], base=16)
        self.serial = int(resp[5:13])
        self.year = int(resp[13:17])
        self.fwrel = int(resp[17:19], 16)
        if bool(int(resp[19:20], 16)):
            self.thread = 'imperial'
        else:
            self.thread = 'metric'
        self.hwrel = int(resp[20:21], 16)
        self.travel = int(resp[21:25], 16)
        self.pulses = int(resp[25:33], 16)
        s_resp = (f'Stage info:\n'
            f'-----------\n'
            f'addr={r_addr}\n'
            f'cmd={r_cmd}\n'
            f'model={self.model}\n'
            f'serial #={self.serial}\n'
            f'year={self.year}\n'
            f'firmware release={self.fwrel}\n'
            f'thread={self.thread}\n'
            f'harware release={self.hwrel}\n'
            f'travel={self.travel} mm\n'
            f'pulses per mm={self.pulses}\n')
        print(s_resp)


    def get_status(self):
        """Get status"""
        cmd = 'gs'
        resp_len = 7
        resp, r_addr, r_cmd = self._query_msg(cmd, resp_len)
        self._handle_status(resp, r_addr, r_cmd)


    def save_user_data(self):
        """Save user data"""
        cmd = 'us'
        resp_len = 7
        resp, r_addr, r_cmd = self._query_msg(cmd, resp_len)
        self._handle_status(resp, r_addr, r_cmd)


    def change_addr(self, new_addr):
        """Change stage address in hex string range 0-F. 
        
        Parameters
        ----------
        new_addr : str
            new address in hex string range 0-F
        """
        cmd = 'ca'
        resp_len = 7
        resp, r_addr, r_cmd = self._query_msg(cmd, resp_len, new_addr)
        self._handle_status(resp, r_addr, r_cmd)
        if r_addr == new_addr:
            self.addr = new_addr
        else:
            # need to reset old address to clear error
            resp, r_addr, r_cmd = self._query_msg(cmd, resp_len, self.addr)


    def _get_motor1_info(self):
        """Get motor 1 information"""
        cmd = 'i1'
        resp_len = 27
        resp, r_addr, r_cmd = self._query_msg(cmd, resp_len)
        if r_cmd == 'GS':
            self._handle_status(resp, r_addr, r_cmd)
            return
        if int(resp[3:4]) == 1:
            self.m1_loop = 'on'
        else:
            self.m1_loop = 'off'
        if int(resp[4:5]) == 1:
            self.m1_motor = 'on'
        else:
            self.m1_motor = 'off'
        self.m1_current = int(resp[5:9], 16) / 1866
        if resp[9:13] == b'FFFF':
            self.m1_ru = 'not defined'
        else:
            self.m1_ru = int(resp[9:13], 16)
        if resp[13:17] == b'FFFF':
            self.m1_rd = 'not defined'
        else:
            self.m1_rd = int(resp[13:17], 16)
        self.m1_fp = int(resp[17:21], 16) / 14740000
        self.m1_ff = 1 / self.m1_fp
        self.m1_bp = int(resp[21:25], 16) / 14740000
        self.m1_bf = 1 / self.m1_bp
        s_resp = (f'Motor 1 info:\n'
            f'-------------\n'
            f'addr={r_addr}\n'
            f'cmd={r_cmd}\n'
            f'm1 loop={self.m1_loop}\n'
            f'm1 motor={self.m1_motor}\n'
            f'm1 current={self.m1_current} A\n'
            f'm1 ramp up={self.m1_ru} PWM increase /ms\n'
            f'm1 ramp down={self.m1_rd} PWM decrease /ms\n'
            f'm1 fwd period={self.m1_fp} s\n'
            f'm1 fwd frequency={self.m1_ff} Hz\n'
            f'm1 bwd period={self.m1_bp} s\n'
            f'm1 bwd frequency={self.m1_bf} Hz\n')
        print(s_resp)


    def _get_motor2_info(self):
        """Get motor 1 information"""
        cmd = 'i2'
        resp_len = 27
        resp, r_addr, r_cmd = self._query_msg(cmd, resp_len)
        if r_cmd == 'GS':
            self._handle_status(resp, r_addr, r_cmd)
            return
        if int(resp[3:4]) == 1:
            self.m2_loop = 'on'
        else:
            self.m2_loop = 'off'
        if int(resp[4:5]) == 1:
            self.m2_motor = 'on'
        else:
            self.m2_motor = 'off'
        self.m2_current = int(resp[5:9], 16) / 1866
        if resp[9:13] == b'FFFF':
            self.m2_ru = 'not defined'
        else:
            self.m2_ru = int(resp[9:13], 16)
        if resp[13:17] == b'FFFF':
            self.m2_rd = 'not defined'
        else:
            self.m2_rd = int(resp[13:17], 16)
        self.m2_fp = int(resp[17:21], 16) / 14740000
        self.m2_ff = 1 / self.m2_fp
        self.m2_bp = int(resp[21:25], 16) / 14740000
        self.m2_bf = 1 / self.m2_bp
        s_resp = (f'Motor 2 info:\n'
            f'-------------\n'
            f'addr={r_addr}\n'
            f'cmd={r_cmd}\n'
            f'm2 loop={self.m2_loop}\n'
            f'm2 motor={self.m2_motor}\n'
            f'm2 current={self.m2_current} A\n'
            f'm2 ramp up={self.m2_ru} PWM increase /ms\n'
            f'm2 ramp down={self.m2_rd} PWM decrease /ms\n'
            f'm2 fwd period={self.m2_fp} s\n'
            f'm2 fwd frequency={self.m2_ff} Hz\n'
            f'm2 bwd period={self.m2_bp} s\n'
            f'm2 bwd frequency={self.m2_bf} Hz\n')
        print(s_resp)


    def get_motor_info(self, motor):
        """Get motor information.
        
        Parameters
        ----------
        motor : int
            motor number
        """
        if motor == 1:
            self._get_motor1_info()
        elif motor == 2:
            self._get_motor2_info()
        else:
            cmd = f'i{motor}'
            resp_len = 7
            resp, r_addr, r_cmd = self._query_msg(cmd, resp_len)
            self._handle_status(resp, r_addr, r_cmd)
        

    def set_motor_fwd_period(self, period, motor):
        pass


    def set_motor_bwd_period(self, period, motor):
        pass


    def search_motor_freq(self, motor):
        """Search for optimal motor frequency in Hz.
        
        Parameters
        ----------
        motor : int
            motor number
        """
        # increase timeout for frequency search
        self.ser.timeout = 10

        cmd = f's{motor}'
        resp_len = 7
        resp, r_addr, r_cmd = self._query_msg(cmd, resp_len)
        self._handle_status(resp, r_addr, r_cmd)
        
        # reset timeout
        self.ser.timeout = self.timeout
        
        # write new frequency to attribute
        self.get_motor_info(motor)


    def scan_motor_current_curve(self, motor):
        pass


    def move(self, pos, mode):
        """Move stage to position in mm.
        
        Parameters
        ----------
        pos : float
            position in mm
        mode : str
            'abs' for absolute, 'rel' for relative position
        """
        # TODO: check why negative relative movements don't seem to work
        # TODO: check why some absolute movements return to zero e.g. 11 mm
        pulses = int(pos * self.pulses)
        hpulses = dec2hex(pulses, 32)
        if mode == 'abs':
            cmd = f'ma{hpulses}'
        elif mode == 'rel':
            cmd = f'mr{hpulses}'
        else:
            print(f'\'{mode}\' mode not recognised. Use \'abs\' or \'rel\'.')
            return
        resp_len = 13
        resp, r_addr, r_cmd = self._query_msg(cmd, resp_len)
        if r_cmd == 'GS':
            self._handle_status(resp, r_addr, r_cmd)
            self.get_pos()
            return
        self.pos = hex2dec(resp[3:11]) / self.pulses
        s_resp = (f'Position:\n'
            f'---------\n'
            f'addr={r_addr}\n'
            f'cmd={r_cmd}\n'
            f'pos={self.pos} mm\n')
        print(s_resp)


    def _get_home_offset(self):
        pass


    def _set_home_offset(self, offset):
        pass


    def get_jog_stepsize(self):
        """Get jog step size in mm"""
        cmd = 'gj'
        resp_len = 13
        resp, r_addr, r_cmd = self._query_msg(cmd, resp_len)
        self.jogsize = int(resp[3:11], 16) / self.pulses
        s_resp = (f'Jog stepsize:\n'
            f'-------------\n'
            f'addr={r_addr}\n'
            f'cmd={r_cmd}\n'
            f'jog={self.jogsize} mm\n')
        print(s_resp)


    def set_jog_stepsize(self, stepsize):
        """Set jog step size in mm.

        Parameters
        ----------
        stepsize : float
            jog step size in mm
        """
        pulses = int(stepsize * self.pulses)
        hpulses = f'{pulses:0{8}x}'
        cmd = f'sj{hpulses}'
        resp_len = 7
        resp, r_addr, r_cmd = self._query_msg(cmd, resp_len)
        self._handle_status(resp, r_addr, r_cmd)
        if self.err == 0:
            self.get_jog_stepsize()


    def jog(self, direction):
        """
        Jog the stage one step.

        Parameters
        ----------
        direction : str
            'fw' for forwards, 'bw' for backwards
        """
        cmd = f'{direction}'
        resp_len = 13
        resp, r_addr, r_cmd = self._query_msg(cmd, resp_len)
        if r_cmd == 'GS':
            self._handle_status(resp, r_addr, r_cmd)
            self.get_pos()
            return
        self.pos = hex2dec(resp[3:11]) / self.pulses
        s_resp = (f'Position:\n'
            f'---------\n'
            f'addr={r_addr}\n'
            f'cmd={r_cmd}\n'
            f'pos={self.pos} mm\n')
        print(s_resp)

    
    def get_pos(self):
        """Get position in mm"""
        cmd = 'gp'
        resp_len = 13
        resp, r_addr, r_cmd = self._query_msg(cmd, resp_len)
        if r_cmd == 'GS':
            self._handle_status(resp, r_addr, r_cmd)
            return
        self.pos = hex2dec(resp[3:11]) / self.pulses
        s_resp = (f'Position:\n'
            f'---------\n'
            f'addr={r_addr}\n'
            f'cmd={r_cmd}\n'
            f'pos={self.pos} mm\n')
        print(s_resp)


    def get_velocity(self):
        """Get velocity in %"""
        cmd = 'gv'
        resp_len = 7
        resp, r_addr, r_cmd = self._query_msg(cmd, resp_len)
        self.velocity = int(resp[3:5], 16)
        s_resp = (f'Velocity:\n'
            f'---------\n'
            f'addr={r_addr}\n'
            f'cmd={r_cmd}\n'
            f'velocity={self.velocity} %\n')
        print(s_resp)


    def set_velocity(self, velocity):
        """Set velocity in %.

        Note that depending on the load, velocity less than 25% to
        45% of max may cause the device to stall.

        Parameters
        ----------
        velocity : int
            velocity in %
        """
        cmd = f'sv{velocity:0{2}x}'
        resp_len = 7
        resp, r_addr, r_cmd = self._query_msg(cmd, resp_len)
        self._handle_status(resp, r_addr, r_cmd)
        if self.err == 0:
            self.get_velocity()

    