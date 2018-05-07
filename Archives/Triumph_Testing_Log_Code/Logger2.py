import Arduino_Logging_API
import serial.tools.list_ports
import serial
import csv

if __name__ == "__main__":

    comports = serial.tools.list_ports.comports()
    for port in comports:
        if '5543733343735150C062' in port[2]:
        # if '85435333231351E0C291' in port[2]:
            print(port[0])
            uart_port = port[0]

    uart_connection = Arduino_Logging_API.uart_connection(uart_port)

    disagree_resets = 0
    successful_matches = 0
    initializations = 0
    timeouts = 0

    while(True):
        log_file = open("logging_file.csv", "a")
        w = csv.writer(log_file)
        row = list()
        input11 = uart_connection.getData()
        if input11:
            print(input11)
            if input11 == "C: Initialized.\n":
                initializations += 1
            elif input11 == "C: A and B match. No reset needed.\n":
                successful_matches += 1
            elif input11 == "C: A and B disagree. Reset.\n":
                disagree_resets += 1
            elif input11 == "C: STMA Timeout\n":
                timeouts += 1

            row.append("C Initializations:" + str(initializations))
            row.append("Non-Resets:" + str(successful_matches))
            row.append("Disagree Resets:" + str(disagree_resets))
            row.append("Timeouts:" + str(timeouts))

            w.writerow(row)

        log_file.close()




