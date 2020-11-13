import serial
from transitions import Machine
import struct
import numpy as np

puertoSerie=serial.Serial()
puertoSerie.baudrate=115200
puertoSerie.port='COM3'
puertoSerie.open()
puertoSerie.flushInput()

amauntOfSynk=0 #cantidad de bytes de sinronismo
amauntOfWords=0 #cantidad de bytes esperados
currenteState=0 #estado actual 0 esperando sincronismo 1 esperando floats
samples=bytes()
samples2=[]

def getNewFloats_t (currentByte):
    global amauntOfSynk
    global amauntOfWords
    global currenteState
    global samples
    global samples2
    if(currenteState==0):
        if(amauntOfSynk>3):
            currenteState=1
            amauntOfSynk=0
            #samples2 = np.append(samples2, currentByte)
            samples=samples+currentByte
            amauntOfWords = amauntOfWords + 1
        else:
            if(currentByte==b'\xff'):
                amauntOfSynk=amauntOfSynk+1
            else:
                amauntOfSynk=0

    else:
        if(amauntOfWords<(40)):
            #samples2=np.append(samples2,currentByte)
            samples = samples + currentByte
            amauntOfWords=amauntOfWords+1
        else:
            amauntOfWords=0
            currenteState = 0
            return True
    return False

def getParsedData():
    a = np.frombuffer(samples, dtype=np.float32)
    return [a[0:3],a[3:6],a[6:10]]

def main():
        global samples
        global samples2
        if(puertoSerie.isOpen()):
            print("Se abrio el puerto")

        while True:
            if(puertoSerie.inWaiting()>0):
                ser_bytes = puertoSerie.read()
                if(getNewFloats_t(ser_bytes)):
                    print(getParsedData())
                    samples =bytes()
        puertoSerie.close()

if __name__ == "__main__":
    main()

