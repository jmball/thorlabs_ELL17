# created by James Ball, 17-07-2019

import serial

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
        self.ser = serial.Serial(port, baud, timeout)
        self.addr = 0
        get_info(self)

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
        r_addr = int(chr(resp[0]))
        r_cmd = resp[1:3].decode('ascii')
        return resp, r_addr, r_cmd


    def _handle_status(self, resp, r_addr, r_cmd):
        """Handle response of get_status"""
        self.err = int(resp[3:5])
        if self.err >= 14:
            self.err_msg = 'reserved'
        else:
            self.err_msg = err_msg_lst[self.err]
        s_resp = f"""addr={r_addr}\n
                cmd={r_cmd}\n
                error={self.err}, {self.err_msg}"""
        print(s_resp)


    def get_info(self):
        """Get stage information"""
        cmd = 'in'
        resp_len = 35
        resp, r_addr, r_cmd = _query_msg(self, cmd, resp_len)
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
        s_resp = f"""addr={r_addr}\n
                cmd={r_cmd}\n
                model={self.model}\n
                serial #={self.serial}\n
                year={self.year}\n
                firmware release={self.fwrel}\n
                thread={self.thread}\n
                harware release={self.hwrel}\n
                travel={self.travel} mm\n
                pulses per mm={self.pulses}"""
        print(s_resp)


    def get_status(self):
        """Get status"""
        cmd = 'gs'
        resp_len = 7
        resp, r_addr, r_cmd = _query_msg(self, cmd, resp_len)
        _handle_status(self, resp, r_addr, r_cmd)


    def save_user_data(self):
        """Save user data"""
        cmd = 'us'
        resp_len = 7
        resp, r_addr, r_cmd = _query_msg(self, cmd, resp_len)
        _handle_status(self, resp, r_addr, r_cmd)


    def change_addr(self, new_addr):
        """Change stage address in hex string range 0-F. 
        
        Parameters
        ----------
        new_addr : str
            new address in hex string range 0-F
        """
        cmd = 'ca'
        resp_len = 7
        resp, r_addr, r_cmd = _query_msg(self, cmd, resp_len, new_addr)
        _handle_status(self, resp, r_addr, r_cmd)
        if r_addr == new_addr:
            self.addr = new_addr
        else:
            # need to reset old address to clear error
            resp, r_addr, r_cmd = _query_msg(self, cmd, resp_len, self.addr)


    def get_motor_info(self, motor):
        """Get motor information.
        
        Parameters
        ----------
        motor : int
            motor number
        """
        # TODO: handle assigning attributes for multiple motors
        cmd = f'i{motor}'
        resp_len = 27
        resp, r_addr, r_cmd = _query_msg(self, cmd, resp_len)
        if r_cmd == 'GS':
            _handle_status(self, resp, r_addr, r_cmd)
            return
        if int(resp[3:4]) == 1:
            r_loop = 'on'
        else:
            r_loop = 'off'
        if int(resp[4:5]) == 1:
            r_motor = 'on'
        else:
            r_motor = 'off'
        r_current = int(resp[5:9], 16) / 1866
        if resp[9:13] == b'FFFF':
            r_ru = 'not defined'
        else:
            r_ru = int(resp[9:13], 16)
        if resp[13:17] == b'FFFF':
            r_rd = 'not defined'
        else:
            r_rd = int(resp[13:17], 16)
        r_fp = int(resp[17:21], 16) / 14740000
        r_ff = 1 / r_fp
        r_bp = int(resp[21:25], 16) / 14740000
        r_bf = 1 / r_bp
        s_resp = f"""addr={r_addr}\n
                cmd={r_cmd}\n
                loop={r_loop}\n
                motor={r_motor}\n
                current={r_current} A\n
                ramp up={r_ru} PWM increase /ms\n
                ramp down={r_rd} PWM decrease /ms\n
                fwd period={r_fp} s\n
                fwd frequency={r_ff} Hz\n
                bwd period={r_bp} s\n
                bwd frequency={r_bf} Hz"""
        print(s_resp)


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
        resp, r_addr, r_cmd = _query_msg(self, cmd, resp_len)
        _handle_status(self, resp, r_addr, r_cmd)
        
        # reset timeout
        self.ser.timeout = self.timeout
        
        # write new frequency to attribute
        get_motor_info(self, motor)


    def scan_motor_current_curve(self, motor):
        pass


    def move_abs(self, pos):
        """Move stage to absolute position in mm.
        
        Parameters
        ----------
        pos : float
            absolute position in mm
        """
        pulses = int(pos * self.pulses)
        hpulses = f'{pulses:0{8}x}'
        cmd = f'ma{hpulses}'
        resp_len = 13
        resp, r_addr, r_cmd = _query_msg(self, cmd, resp_len)
        if r_cmd == 'GS':
            _handle_status(self, resp, r_addr, r_cmd)
            return
        self.pos = int(resp[3:11], 16) / self.pulses
        s_resp = f"""addr={r_addr}\n
                cmd={r_cmd}\n
                pos={self.pos} mm"""
        print(s_resp)


    def move_rel(self, pos):
        """Move stage relative to current position in mm.
        
        Parameters
        ----------
        pos : float
            relative position in mm
        """
        pulses = int(pos * self.pulses)
        if pos > 0:
            hpulses = f'{pulses:0{8}x}'
        else:
            # TODO: add 2's complement conversion for negative numbers
            pass
        cmd = f'ma{hpulses}'
        resp_len = 13
        resp, r_addr, r_cmd = _query_msg(self, cmd, resp_len)
        if r_cmd == 'GS':
            _handle_status(self, resp, r_addr, r_cmd)
            return
        self.pos = int(resp[3:11], 16) / self.pulses
        s_resp = f"""addr={r_addr}\n
                cmd={r_cmd}\n
                pos={self.pos} mm"""
        print(s_resp)


    def _get_home_offset(self):
        pass


    def _set_home_offset(self, offset):
        pass


    def get_jog_stepsize(self):
        """Get jog step size in mm"""
        cmd = 'gj'
        resp_len = 13
        resp, r_addr, r_cmd = _query_msg(self, cmd, resp_len)
        self.jogsize = int(resp[3:11], 16) / self.pulses
        s_resp = f"""addr={r_addr}\n
                cmd={r_cmd}\n
                jog={self.jogsize} mm"""
        print(s_resp)


    def set_jog_stepsize(self, stepsize):
        """Set jog step size in mm.

        Parameters
        ----------
        stepsize : float
            jog step size in mm
        """
        pulses = stepsize * self.pulses
        hpulses = f'{pulses:0{8}x}'
        cmd = f'sj{hpulses}'
        resp_len = 7
        resp, r_addr, r_cmd = _query_msg(self, cmd, resp_len)
        _handle_status(self, resp, r_addr, r_cmd)
        if self.err == 0:
            get_jog_stepsize(self)


    def jog_stage(self, direction):
        """
        Jog the stage one step.

        Parameters
        ----------
        direction : str
            'fw' for forwards, 'bw' for backwards
        """
        cmd = f'{direction}'
        resp_len = 13
        resp, r_addr, r_cmd = _query_msg(self, cmd, resp_len)
        if r_cmd == 'GS':
            _handle_status(self, resp, r_addr, r_cmd)
            return
        self.pos = int(resp[3:11], 16) / self.pulses
        s_resp = f"""addr={r_addr}\n
                cmd={r_cmd}\n
                pos={self.pos} mm"""
        print(s_resp)

    
    def get_pos(self):
        """Get position in mm"""
        cmd = 'gp'
        resp_len = 13
        resp, r_addr, r_cmd = _query_msg(self, cmd, resp_len)
        if r_cmd == 'GS':
            _handle_status(self, resp, r_addr, r_cmd)
            return
        self.pos = int(resp[3:11], 16) / self.pulses
        s_resp = f"""addr={r_addr}\n
                cmd={r_cmd}\n
                pos={self.pos} mm"""
        print(s_resp)


    def get_velocity(self):
        """Get velocity in %"""
        cmd = 'gv'
        resp_len = 7
        resp, r_addr, r_cmd = _query_msg(self, cmd, resp_len)
        self.velocity = int(resp[3:5], 16)
        s_resp = f"""addr={r_addr}\n
                cmd={r_cmd}\n
                velocity={self.velocity} %"""
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
        resp, r_addr, r_cmd = _query_msg(self, cmd, resp_len)
        _handle_status(self, resp, r_addr, r_cmd)
        if self.err == 0:
            get_velocity(self)

    