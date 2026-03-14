from tmc.TMC_2209_StepperDriver import *
import time
import sys


print("---")
print("SCRIPT START")
print("---")





#-----------------------------------------------------------------------
# initiate the TMC_2209 class
# use your pins for pin_step, pin_dir, pin_en here
#-----------------------------------------------------------------------
tmc1 = TMC_2209(18, 19, 0, 0) # you can populate pin_en, but I just tied it to ground
tmc2 = TMC_2209(22, 23, 0, 2) # you can populate pin_en, but I just tied it to ground
print("Constructed")




#-----------------------------------------------------------------------
# set the loglevel of the libary (currently only printed)
# set whether the movement should be relative or absolute
# both optional
#-----------------------------------------------------------------------
tmc1.setLoglevel(Loglevel.info)
tmc1.setMovementAbsRel(MovementAbsRel.absolute)

tmc2.setLoglevel(Loglevel.info)
tmc2.setMovementAbsRel(MovementAbsRel.absolute)

#     none = 0
#     error = 10
#     info = 20
#     debug = 30
#     movement = 40
#     all = 100



#-----------------------------------------------------------------------
# these functions change settings in the TMC register
#-----------------------------------------------------------------------
tmc1.setDirection_reg(False)
tmc1.setVSense(True)
tmc1.setCurrent(600)
tmc1.setIScaleAnalog(True)
tmc1.setInterpolation(True)
tmc1.setSpreadCycle(False)
tmc1.setMicrosteppingResolution(1)
tmc1.setInternalRSense(False)

tmc2.setDirection_reg(False)
tmc2.setVSense(True)
tmc2.setCurrent(600)
tmc2.setIScaleAnalog(True)
tmc2.setInterpolation(True)
tmc2.setSpreadCycle(False)
tmc2.setMicrosteppingResolution(1)
tmc2.setInternalRSense(False)

print("---\n---")





#-----------------------------------------------------------------------
# these functions read and print the current settings in the TMC register
#-----------------------------------------------------------------------
tmc1.readIOIN()
tmc1.readCHOPCONF()
tmc1.readDRVSTATUS()
tmc1.readGCONF()

print("---\n---")

tmc2.readIOIN()
tmc2.readCHOPCONF()
tmc2.readDRVSTATUS()
tmc2.readGCONF()

print("---\n---")





#-----------------------------------------------------------------------
# set the Accerleration and maximal Speed
#-----------------------------------------------------------------------
tmc1.setAcceleration(2000)
tmc1.setMaxSpeed(500)

tmc2.setAcceleration(2000)
tmc2.setMaxSpeed(500)

tmc2.setAcceleration(2000)
tmc2.setMaxSpeed(500)

tmc1.setMotorEnabled(False)
tmc2.setMotorEnabled(False)

#-----------------------------------------------------------------------
# activate the motor current output
#-----------------------------------------------------------------------
tmc1.setMotorEnabled(True)

tmc2.setMotorEnabled(True)




#-----------------------------------------------------------------------
# set a callback function for the stallguard interrupt based detection
# 1. param: pin connected to the tmc DIAG output
# 2. param: is the threshold StallGuard
# 3. param: is the callback function (threaded)
# 4. param (optional): min speed threshold (in steptime measured  in  clock  cycles)
#-----------------------------------------------------------------------
def my_callback1(channel):  
    print("StallGuard!")
    tmc1.stop()

def my_callback2(channel):  
    print("StallGuard!")
    tmc2.stop()

tmc1.setStallguard_Callback(13, 50, my_callback1) # after this function call, StallGuard is active

finishedsuccessfully = tmc1.runToPositionSteps(4000, MovementAbsRel.relative)    #move 4000 steps forward

if(finishedsuccessfully == True):
    print("Movement finished successfully")
else:
    print("Movement was not completed")

tmc2.setStallguard_Callback(13, 50, my_callback2) # after this function call, StallGuard is active

finishedsuccessfully = tmc2.runToPositionSteps(4000, MovementAbsRel.relative)    #move 4000 steps forward

if(finishedsuccessfully == True):
    print("Movement finished successfully")
else:
    print("Movement was not completed")




#-----------------------------------------------------------------------
# deactivate the motor current output
#-----------------------------------------------------------------------
tmc1.setMotorEnabled(False)
#tmc2.setMotorEnabled(False)

print("---\n---")





#-----------------------------------------------------------------------
# deinitiate the TMC_2209 class
#-----------------------------------------------------------------------
del tmc1

print("---")
print("SCRIPT FINISHED")
print("---")
