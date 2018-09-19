# Python Error Logging Code

Utilizing a USB connection to STM_C, the Python scripts monitor and log output from the microcontroller. 

logger.py is a general logger and will print any text output from the STM(anything STM C prints) to a text file. 

logger2.py is specific to the radiation testing. The outputs that it looks for are specified to the Radiation Testing version of the STM code. The script Logger2.py logs all outputs from STM C to a csv file in a parsed form so that the information from C is easy to interpret. Every time the script is run it appends to the same file. If the file is deleted or removed from the folder they will make a new file. 


