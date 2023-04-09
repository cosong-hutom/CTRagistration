#!/bin/bash 

source env/bin/activate

#eapPath="/Users/scosco/Desktop/scosco/hutom/Project/Registration/Project/Data/01009ug_9/eap"
#ppPath="/Users/scosco/Desktop/scosco/hutom/Project/Registration/Project/Data/01009ug_9/pp"
#outEapPath="/Users/scosco/Desktop/scosco/hutom/Project/Registration/Project/Data/01009ug_9/eapReg"

#eapPath="/Users/scosco/Desktop/scosco/hutom/Project/Registration/Project/Data/01009ug_10/eap"
#ppPath="/Users/scosco/Desktop/scosco/hutom/Project/Registration/Project/Data/01009ug_10/pp"
#outEapPath="/Users/scosco/Desktop/scosco/hutom/Project/Registration/Project/Data/01009ug_10/eapReg"

#eapPath="/Users/scosco/Desktop/scosco/hutom/Project/Registration/Project/Data/01009ug_11/eap"
#ppPath="/Users/scosco/Desktop/scosco/hutom/Project/Registration/Project/Data/01009ug_11/pp"
#outEapPath="/Users/scosco/Desktop/scosco/hutom/Project/Registration/Project/Data/01009ug_11/eapReg"

eapPath="/Users/scosco/Desktop/scosco/hutom/Project/Registration/Project/Data/01009ug_12/eap"
ppPath="/Users/scosco/Desktop/scosco/hutom/Project/Registration/Project/Data/01009ug_12/pp"
outEapPath="/Users/scosco/Desktop/scosco/hutom/Project/Registration/Project/Data/01009ug_12/eapReg"

#eapPath="/Users/scosco/Desktop/scosco/hutom/Project/Registration/Project/Data/01009ug_13/eap"
#ppPath="/Users/scosco/Desktop/scosco/hutom/Project/Registration/Project/Data/01009ug_13/pp"
#outEapPath="/Users/scosco/Desktop/scosco/hutom/Project/Registration/Project/Data/01009ug_13/eapReg"

#eapPath="/Users/scosco/Desktop/scosco/hutom/Project/Registration/Project/Data/01009ug_14/eap"
#ppPath="/Users/scosco/Desktop/scosco/hutom/Project/Registration/Project/Data/01009ug_14/pp"
#outEapPath="/Users/scosco/Desktop/scosco/hutom/Project/Registration/Project/Data/01009ug_14/eapReg"

#eapPath="/Users/scosco/Desktop/scosco/hutom/Project/Registration/Project/Data/01009ug_15/eap"
#ppPath="/Users/scosco/Desktop/scosco/hutom/Project/Registration/Project/Data/01009ug_15/pp"
#outEapPath="/Users/scosco/Desktop/scosco/hutom/Project/Registration/Project/Data/01009ug_15/eapReg"

#eapPath="/Users/scosco/Desktop/scosco/hutom/Project/Registration/Project/Data/01009ug_16/eap"
#ppPath="/Users/scosco/Desktop/scosco/hutom/Project/Registration/Project/Data/01009ug_16/pp"
#outEapPath="/Users/scosco/Desktop/scosco/hutom/Project/Registration/Project/Data/01009ug_16/eapReg"

#eapPath="/Users/scosco/Desktop/scosco/hutom/Project/Registration/Project/Data/01009ug_17/eap"
#ppPath="/Users/scosco/Desktop/scosco/hutom/Project/Registration/Project/Data/01009ug_17/pp"
#outEapPath="/Users/scosco/Desktop/scosco/hutom/Project/Registration/Project/Data/01009ug_17/eapReg"

#eapPath="/Users/scosco/Desktop/scosco/hutom/Project/Registration/Project/Data/01009ug_18/eap"
#ppPath="/Users/scosco/Desktop/scosco/hutom/Project/Registration/Project/Data/01009ug_18/pp"
#outEapPath="/Users/scosco/Desktop/scosco/hutom/Project/Registration/Project/Data/01009ug_18/eapReg"

python registrationStep.py -eap ${eapPath} -pp ${ppPath} -o ${outEapPath} 

deactivate 