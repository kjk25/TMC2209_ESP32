from tmc.TMC_2209_StepperDriver import *
import time


print("---")
print("SCRIPT START")
print("---")





#-----------------------------------------------------------------------
# initiate the TMC_2209 class
# use your pins for pin_step, pin_dir, pin_en here
#-----------------------------------------------------------------------
tmc = TMC_2209(18, 19, 0) # you can populate pin_en, but I just tied it to ground





#-----------------------------------------------------------------------
# set the loglevel of the libary (currently only printed)
# set whether the movement should be relative or absolute
# both optional
#-----------------------------------------------------------------------
tmc.setLoglevel(Loglevel.debug)
tmc.setMovementAbsRel(MovementAbsRel.absolute)





#-----------------------------------------------------------------------
# these functions change settings in the TMC register
#-----------------------------------------------------------------------
tmc.setDirection_reg(False)
tmc.setVSense(True)
tmc.setCurrent(300)
tmc.setIScaleAnalog(True)
tmc.setInterpolation(True)
tmc.setSpreadCycle(False)
tmc.setMicrosteppingResolution(2)
tmc.setInternalRSense(False)


print("---\n---")





#-----------------------------------------------------------------------
# this function test whether the connection of the DIR, STEP and EN pin
# between Raspberry Pi and TMC driver is working
#-----------------------------------------------------------------------
tmc.testDirStepEn()

print("---\n---")





#-----------------------------------------------------------------------
# deactivate the motor current output
#-----------------------------------------------------------------------
tmc.setMotorEnabled(False)

print("---\n---")





#-----------------------------------------------------------------------
# deinitiate the TMC_2209 class
#-----------------------------------------------------------------------
del tmc

print("---")
print("SCRIPT FINISHED")
print("---")
