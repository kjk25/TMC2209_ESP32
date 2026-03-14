from tmc.TMC_2209_StepperDriver import *
import time


print("---")
print("SCRIPT START")
print("---")





#-----------------------------------------------------------------------
# initiate the TMC_2209 class
# use your pins for pin_step, pin_dir, pin_en here
#-----------------------------------------------------------------------
tmc = TMC_2209(22, 23, 0, 2)





#-----------------------------------------------------------------------
# set the loglevel of the libary (currently only printed)
# set whether the movement should be relative or absolute
# both optional
#-----------------------------------------------------------------------
tmc.setLoglevel(Loglevel.debug)
tmc.setMovementAbsRel(MovementAbsRel.relative)





#-----------------------------------------------------------------------
# these functions change settings in the TMC register
#-----------------------------------------------------------------------
tmc.setDirection_reg(False)
tmc.setVSense(True)
tmc.setCurrent(200)
tmc.setIScaleAnalog(True)
tmc.setInterpolation(True)
tmc.setSpreadCycle(False)
tmc.setMicrosteppingResolution(1)
tmc.setInternalRSense(False)


print("---\n---")





#-----------------------------------------------------------------------
# these functions read and print the current settings in the TMC register
#-----------------------------------------------------------------------
#tmc.readIOIN()
#tmc.readCHOPCONF()
#tmc.readDRVSTATUS()
#tmc.readGCONF()

print("---\n---")





#-----------------------------------------------------------------------
# set the Accerleration and maximal Speed
#-----------------------------------------------------------------------
tmc.setAcceleration(2000)
tmc.setMaxSpeed(500)





#-----------------------------------------------------------------------
# activate the motor current output
#-----------------------------------------------------------------------
tmc.setMotorEnabled(True)





#-----------------------------------------------------------------------
# move the motor 1 revolution
#-----------------------------------------------------------------------
tmc.runToPositionSteps(800)                             #move to position 400
tmc.runToPositionSteps(0)                               #move to position 0


tmc.runToPositionSteps(1000, MovementAbsRel.relative)    #move 400 steps forward
tmc.runToPositionSteps(-600, MovementAbsRel.relative)   #move 400 steps backward


tmc.runToPositionSteps(400)                             #move to position 400
tmc.runToPositionSteps(0)                               #move to position 0





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
