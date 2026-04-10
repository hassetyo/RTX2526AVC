import serial # pulls in the usb serial driver
import struct # pulls in binary packing/unpacking tools
import threading # lets us run background workers
import time # pulls in the clock functions

# V2V BRIDGE LIBRARY 📡
# This is the "Translator" for the Jetson and the RPi
# and makes Python talk the same binary grammar as the ESP32 radio boxes

################################# config stuff
# constants for the binary headers (must match the ESP32 code exactly!)
SOF = 0xAA         # Start of packet marker (The "Wait for it..." byte)
TYPE_TELEM = 1     # status/telemetry (UAV stats)
TYPE_CMD = 2       # missions/instructions (UGV orders)
TYPE_MSG = 3       # raw crap talking strings (debugging)

# command codes for the UGV mission engine
CMD_ARM          = 1  # wake up the motors
CMD_DISARM       = 2  # sleep the motors
CMD_TAKEOFF      = 3  # drone specific
CMD_LAND         = 4  # drone specific
CMD_MOVE_FORWARD = 5  # drive 10ft
CMD_MOVE_2FT     = 6  # drive 2ft
CMD_TURN_RIGHT   = 7  # pivot right 90
CMD_TURN_LEFT    = 8  # pivot left 90
CMD_CIRCLE       = 9  # double circle stunt

# mode identifiers for the Pixhawk
MODE_INITIAL  = 0 # just turned on
MODE_GUIDED   = 1 # taking our commands
MODE_AUTO     = 2 # running a flight plan
MODE_LAND     = 3 # coming home
MODE_DISARMED = 4 # safe state

# pack formats so the radio knows how to handle the binary junk
TELEM_FMT = "<IIffBB"  # Little-endian: uint32, uint32, float, float, uint8, uint8
CMD_FMT   = "<IBB"     # Little-endian: uint32, uint8, uint8

class V2VBridge: # the main bridge class
    def __init__(self, port, baud=115200, name="Bridge"): # initialize the link
        self.port = port # save the usb port name
        self.baud = baud # save the baud rate speed
        self.name = name # save the display name for logs
        self.ser = None # haven't opened serial yet
        
        # Trash cans for the latest data we caught
        self.latest_telemetry = None # stores last status packet
        self.latest_command = None # stores last instruction packet
        self.latest_msg = None # stores last debug string
        self._running = False # worker thread flag
        self._lock = threading.Lock() # thread-safety guard
        self._thread = None # background thread object

    def connect(self): # kicks off the link
        # Kicks off the serial port and starts a background thread so we dont drop packets
        print(f"[{self.name}] Connecting to {self.port} at {self.baud}...") # log it
        self.ser = serial.Serial(self.port, self.baud, timeout=0.01) # open the usb wire
        self._running = True # set running flag to true
        self._thread = threading.Thread(target=self._read_loop, daemon=True) # define worker
        self._thread.start() # launch worker thread
        print(f"[{self.name}] Bridge Thread Running... Listening for the radio.") # log success

    def stop(self): # shuts down the link
        # Kill the thread and close the port
        self._running = False # tell worker to stop
        if self._thread: # if thread exists
            self._thread.join(timeout=1.0) # wait for it to die
        if self.ser: # if serial port is open
            self.ser.close() # close the hardware link

    ############################ helper logic

    # i had to add this casue , needed a way to seal them so i know they werent tampered with
    # m using XOR checksum to prove the data is clean
    def _chk_xor(self, type_b, len_b, payload: bytes) -> int: # checksum tool
        c = (type_b ^ len_b) & 0xFF # seed with type and length
        for b in payload: # loop every byte in the data
            c ^= b # flip bits against the seed
        return c & 0xFF # return 8-bit result

    #################### the brain (USB Listener)

    def _read_loop(self): # infinite background worker loop
        # This sits in the background and looks for the 0xAA header in the serial stream
        while self._running: # while we haven't stopped
            if self.ser.in_waiting == 0: # if no bytes are on the wire
                time.sleep(0.001) # relax the cpu for 1ms
                continue # loop back
                
            # Look for the starting 0xAA byte
            b = self.ser.read(1) # read one byte
            if not b or b[0] != SOF: # check if it's our magic 0xAA
                continue # ignore if it isn't
            
            # Grab what type it is and how long
            hdr = self.ser.read(2) # read type and length bytes
            if len(hdr) < 2: continue # abort if not enough bytes
            f_type, f_len = hdr[0], hdr[1] # extract type and size

            # Pull the actual data payload
            payload = self.ser.read(f_len) # read the actual message
            if len(payload) < f_len: continue # abort if data is missing

            # Check the "signature" byte
            chk_byte = self.ser.read(1) # read the final checksum
            if not chk_byte: continue # abort if no checksum found
            
            # If signature matches, save it in the global state
            if chk_byte[0] == self._chk_xor(f_type, f_len, payload): # verify seal
                with self._lock: # lock the state variables
                    if f_type == TYPE_TELEM and f_len == struct.calcsize(TELEM_FMT): # if status
                        self.latest_telemetry = struct.unpack(TELEM_FMT, payload) # unpack into tuple
                    elif f_type == TYPE_CMD and f_len == struct.calcsize(CMD_FMT): # if command
                        self.latest_command = struct.unpack(CMD_FMT, payload) # unpack into tuple
                    elif f_type == TYPE_MSG: # if debug string
                        self.latest_msg = payload.decode('ascii', errors='ignore') # decode to text

    #################### API (shouting at the bridge)

    def get_telemetry(self): # getter for latest status
        # Grabs the latest status packet we found
        with self._lock: # thread-safe access
            return self.latest_telemetry # return value

    def get_command(self, consume=True): # getter for latest command
        # Grabs a command and clears it so we dont repeat it
        with self._lock: # thread-safe access
            val = self.latest_command # save value
            if consume: # if we want to delete it after reading
                self.latest_command = None # clear it
            return val # return saved value

    def get_message(self, consume=True): # getter for latest string
        # Pulls out a raw string message (like the "hello" ones)
        with self._lock: # thread-safe access
            val = self.latest_msg # save value
            if consume: # if we want to delete it after reading
                self.latest_msg = None # clear it
            return val # return saved value

    def send_telemetry(self, seq, t_ms, vx, vy, marker, estop): # sender for status
        # Packages up status into binary and shoves it down the USB wire
        payload = struct.pack(TELEM_FMT, seq, t_ms, vx, vy, marker, estop) # pack to binary
        chk = self._chk_xor(TYPE_TELEM, len(payload), payload) # get the checksum
        self.ser.write(bytes([SOF, TYPE_TELEM, len(payload)]) + payload + bytes([chk])) # ship it
        self.ser.flush() # force it out now

    def send_command(self, cmdSeq, cmd, estop): # sender for mission orders
        # Packages a command and sends it to the other ESP32
        payload = struct.pack(CMD_FMT, cmdSeq, cmd, estop) # pack to binary
        chk = self._chk_xor(TYPE_CMD, len(payload), payload) # get the checksum
        self.ser.write(bytes([SOF, TYPE_CMD, len(payload)]) + payload + bytes([chk])) # ship it
        self.ser.flush() # force it out now

    def send_message(self, text: str): # sender for raw talk
        # Sends a raw line of text (like "hello uav") over the air
        payload = text[:60].encode('ascii', errors='ignore') # clip and encode
        chk = self._chk_xor(TYPE_MSG, len(payload), payload) # get the checksum
        self.ser.write(bytes([SOF, TYPE_MSG, len(payload)]) + payload + bytes([chk])) # ship it
        self.ser.flush() # force it out now
