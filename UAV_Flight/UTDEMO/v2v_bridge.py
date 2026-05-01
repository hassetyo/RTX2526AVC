import serial # i had to pull in the usb serial driver so it can actually talk
import struct # pulled in the binary packing and unpacking tools cause python is annoying
import threading # lets me hire background workers to do the reading
import time # pulls in the clock functions so i can sleep

# V2V BRIDGE LIBRARY
# this is the translator i built for the jetson and the rpi
# it makes python talk the exact same binary grammar as the esp32 radio boxes i made

################################# config stuff i setup
# constants for the binary headers that must match the esp32 code exactly or it breaks
SOF = 0xAA         # start of packet marker basically the wait for it byte
TYPE_TELEM = 1     # status and telemetry stuff like uav stats
TYPE_CMD = 2       # missions and instructions for bossing the ugv around
TYPE_MSG = 3       # raw crap talking strings i use for debugging

# command codes i made for the ugv mission engine
CMD_ARM          = 1  # wake up the scary motors
CMD_DISARM       = 2  # put the motors to sleep before they chop my fingers
CMD_TAKEOFF      = 3  # drone specific takeoff command
CMD_LAND         = 4  # drone specific land command
CMD_MOVE_FORWARD = 5  # drive 10ft forward
CMD_MOVE_2FT     = 6  # drive 2ft forward
CMD_TURN_RIGHT   = 7  # pivot right 90 degrees
CMD_TURN_LEFT    = 8  # pivot left 90 degrees
CMD_CIRCLE       = 9  # double circle stunt i added
CMD_MISSION_1    = 10 # trigger the competition mission 1
CMD_MISSION_2    = 11 # trigger the competition mission 2
CMD_MISSION_3    = 12 # trigger the competition mission 3
CMD_STOP         = 13 # kill all movement immediately so it doesnt crash

# parametric commands added for visual-servo UGV navigation
# for these two commands the cmdSeq field is REPURPOSED to carry the manoeuvre magnitude
# instead of acting as a deduplication key - this is intentional and documented here
CMD_TURN_ANGLE_RIGHT = 14  # rotate UGV right (CW)  by cmdSeq degrees  (integer, 0-360)
CMD_TURN_ANGLE_LEFT  = 15  # rotate UGV left  (CCW) by cmdSeq degrees  (integer, 0-360)
CMD_MOVE_DIST        = 16  # drive UGV forward by cmdSeq centimetres    (integer, cm)

# mode identifiers i ripped for the pixhawk
MODE_INITIAL  = 0 # just turned on and stupid
MODE_GUIDED   = 1 # taking our code commands
MODE_AUTO     = 2 # running a preplanned flight
MODE_LAND     = 3 # coming home to land
MODE_DISARMED = 4 # safe state finally

# pack formats i wrote so the radio knows how to handle the binary junk
TELEM_FMT = "<IIffBB"  # little endian packing uint32 uint32 float float uint8 uint8
CMD_FMT   = "<IBB"     # little endian packing uint32 uint8 uint8

class V2VBridge: # the main bridge class i wrote
    def __init__(self, port, baud=115200, name="Bridge"): # initialize the serial link
        self.port = port # save the usb port name here
        self.baud = baud # save the baud rate speed i picked
        self.name = name # save the display name just for logs
        self.ser = None # i havent actually opened the serial port yet
        
        # trash cans i made for the latest data we catch from the air
        self.latest_telemetry = None # stores the very last status packet
        self.latest_command = None # stores the very last instruction packet
        self.latest_msg = None # stores the very last debug string
        self._running = False # worker thread flag to see if he is alive
        self._lock = threading.Lock() # thread safety guard lock i had to add
        self._thread = None # empty background thread object

    def connect(self): # kicks off the actual physical link
        # kicks off the serial port and starts a background thread so we dont drop packets
        print(f"[{self.name}] Connecting to {self.port} at {self.baud}...") # log that we are trying
        self.ser = serial.Serial(self.port, self.baud, timeout=0.01) # officially open the usb wire
        self._running = True # set running flag to true so the loop starts
        self._thread = threading.Thread(target=self._read_loop, daemon=True) # define the background worker
        self._thread.start() # actually launch the worker thread into the void
        print(f"[{self.name}] Bridge Thread Running... Listening for the radio.") # log massive success

    def stop(self): # shuts down the link safely
        # i had to make it kill the thread and close the port cleanly
        self._running = False # tell the worker to please stop
        if self._thread: # if the thread actually exists
            self._thread.join(timeout=1.0) # wait a second for it to die
        if self.ser: # if the serial port is still open
            self.ser.close() # slam the hardware link closed

    ############################ helper logic i wrote

    # i had to add this cause i needed a way to seal them so i know they werent tampered with
    # im using XOR checksum math to prove the data is totally clean
    def _chk_xor(self, type_b, len_b, payload: bytes) -> int: # the checksum math tool
        c = (type_b ^ len_b) & 0xFF # seed it with the type and length
        for b in payload: # loop through every single byte in the meat
            c ^= b # flip the bits against the seed
        return c & 0xFF # spit out the 8 bit result

    #################### the brain that listens to usb

    

    def _read_loop(self): # the infinite background worker loop i made
        # this guy just sits in the background and looks for the 0xAA header in the serial stream
        while self._running: # while we literally havent stopped him
            if self.ser.in_waiting == 0: # if absolutely no bytes are on the wire
                time.sleep(0.001) # relax the poor cpu for 1ms
                continue # loop back to the top
                
            # look for the starting 0xAA byte i picked
            b = self.ser.read(1) # yank one byte off the wire
            if not b or b[0] != SOF: # check if it is our magic 0xAA start byte
                continue # just ignore it if it is static
            
            # grab what type it is and exactly how long
            hdr = self.ser.read(2) # read the type and length bytes
            if len(hdr) < 2: continue # abort if we didnt get enough bytes
            f_type, f_len = hdr[0], hdr[1] # extract the type and size

            # pull the actual data payload meat
            payload = self.ser.read(f_len) # read the actual message chunk
            if len(payload) < f_len: continue # abort if the data got cut off

            # check the signature byte at the end
            chk_byte = self.ser.read(1) # read the final checksum fingerprint
            if not chk_byte: continue # abort if there is no checksum found
            
            # if the signature actually matches i save it in the global state
            if chk_byte[0] == self._chk_xor(f_type, f_len, payload): # verify the seal with math
                with self._lock: # lock the state variables so workers dont fight
                    if f_type == TYPE_TELEM and f_len == struct.calcsize(TELEM_FMT): # if it is a status
                        self.latest_telemetry = struct.unpack(TELEM_FMT, payload) # unpack the binary into a tuple
                    elif f_type == TYPE_CMD and f_len == struct.calcsize(CMD_FMT): # if it is a command
                        self.latest_command = struct.unpack(CMD_FMT, payload) # unpack it into a tuple
                    elif f_type == TYPE_MSG: # if it is just a debug string
                        self.latest_msg = payload.decode('ascii', errors='ignore') # decode the binary text to ascii

    #################### API for shouting at the bridge

    def get_telemetry(self): # the getter i wrote for the latest status
        # grabs the very latest status packet we managed to find
        with self._lock: # thread safe access lock
            return self.latest_telemetry # spit the value out

    def get_command(self, consume=True): # the getter for the latest command
        # grabs a command and immediately clears it so we dont repeat it like an idiot
        with self._lock: # thread safe access again
            val = self.latest_command # save the value temporary
            if consume: # if we actually want to delete it after reading
                self.latest_command = None # wipe it clean
            return val # spit out the saved value

    def get_message(self, consume=True): # getter for the latest string
        # pulls out a raw string message like the hello ones
        with self._lock: # lock it up
            val = self.latest_msg # save the value
            if consume: # if we want to consume and delete
                self.latest_msg = None # clear it out
            return val # spit it out

    def send_telemetry(self, seq, t_ms, vx, vy, marker, estop): # the sender for status stuff
        # packages up the status into binary and literally shoves it down the usb wire
        payload = struct.pack(TELEM_FMT, seq, t_ms, vx, vy, marker, estop) # pack all the variables to binary
        chk = self._chk_xor(TYPE_TELEM, len(payload), payload) # do the math to get the checksum
        self.ser.write(bytes([SOF, TYPE_TELEM, len(payload)]) + payload + bytes([chk])) # ship the whole frame
        self.ser.flush() # force it out of the buffer right now

    def send_command(self, cmdSeq, cmd, estop): # the sender i made for mission orders
        # packages a command up and sends it over to the other esp32
        payload = struct.pack(CMD_FMT, cmdSeq, cmd, estop) # pack it down to tiny binary
        chk = self._chk_xor(TYPE_CMD, len(payload), payload) # get the checksum again
        self.ser.write(bytes([SOF, TYPE_CMD, len(payload)]) + payload + bytes([chk])) # ship the frame
        self.ser.flush() # force it out immediately

    def send_message(self, text: str): # sender i built for raw talk
        # sends a raw line of text like hello uav over the air
        payload = text[:60].encode('ascii', errors='ignore') # clip it so its not too long and encode it
        chk = self._chk_xor(TYPE_MSG, len(payload), payload) # calculate the checksum
        self.ser.write(bytes([SOF, TYPE_MSG, len(payload)]) + payload + bytes([chk])) # ship it down the wire
        self.ser.flush() # force it out