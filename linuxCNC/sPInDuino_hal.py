#!/usr/bin/python
import hal, time, serial, struct

h = hal.component("sPInDuino_hal")
h.newpin("spindle-at",  hal.HAL_FLOAT, hal.HAL_OUT)
h.newpin("spindle-cnt", hal.HAL_U32, hal.HAL_OUT)
h.newpin("spindle-cmd", hal.HAL_FLOAT, hal.HAL_IN)
h.ready()

ser = serial.Serial('/dev/ttyACM0', 115200, bytesize=8, parity='N', stopbits=1, timeout=0, xonxoff=0, rtscts=0)
ser.open()
ser.isOpen()

pidSP=pidIn=pidOut=pidP=pidI=pidD=0
pidAuto=pidReverse=0
cmdSpeed=cmdSP=0

def send_instruction():
#    print "Mover a ", cmdSpeed, " RPMs, SP: ", cmdSP
    ser.write(struct.pack('?', pidAuto));
    ser.write(struct.pack('?', pidReverse));
    ser.write(struct.pack('f', cmdSP));
    ser.write(struct.pack('f', pidIn));
    ser.write(struct.pack('f', pidOut));
    ser.write(struct.pack('f', pidP));
    ser.write(struct.pack('f', pidI));
    ser.write(struct.pack('f', pidD));

try:
    valor=0
    #Main Loop
    out = ''
    char = ''

    while 1:
        if cmdSpeed != abs(h['spindle-cmd']):
            cmdSpeed = abs(h['spindle-cmd'])
            cmdSP = 1024*cmdSpeed/30000;

        while ser.inWaiting() > 0:
            char = ser.read(1)
            if char == '\n':
                break;
            out += char

        if out != '' and char == '\n':
#            print "\n>> " + out
            campos = out.split(' ')
            if len(campos) >= 9:
                h['spindle-at']  = (30000*float(campos[2]))/1024
                h['spindle-cnt'] = int(float(campos[3]))
                pidSP = float(campos[1])
                pidIn = float(campos[2])
                pidOut = float(campos[3])
                pidP = float(campos[4])
                pidI = float(campos[5])
                pidD = float(campos[6])
                pidAuto    = 0 if campos[7] == "Manual" else 1
                pidReverse = 0 if campos[8] == "Direct" else 1
                if cmdSP != pidSP:
                    send_instruction()
            out = ''
       
except KeyboardInterrupt:
    raise SystemExit
