import Arduino_Logging_API
import serial.tools.list_ports
import serial
import time

if __name__ == "__main__":

    comports = serial.tools.list_ports.comports()
    for port in comports:
        # if '5543733343735150C062' in port[2]:
        if '85435333231351E0C291' in port[2]:
            print(port[0])
            uart_port = port[0]

    uart_connection = Arduino_Logging_API.uart_connection(uart_port)

    while(True):
        text_file = open("logging_file.txt", "a")
        input11 = uart_connection.getData()
        if input11:
            text_file.write(time.strftime("%H-%M-%S"))
            text_file.write(":\n")
            text_file.write(input11)
            text_file.write("\n\n")
            print(input11)


        text_file.close()




