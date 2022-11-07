#importing
from time import sleep
import L1_ina as ina
import L1_log as logging
#program section

def LogTelem():
    inaVolt = ina.readVolts()
    print("Robot Voltage: ", inaVolt)
    logging.tmpFile(inaVolt, "TempVoltLog.txt")
    sleep(1)
    
if __name__ == "__main__":
    while True:
        LogTelem()