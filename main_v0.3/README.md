# Main
Contains main code, corresponding to Trillium v3.

As this version of Trillium involved a separate hardware connection for each MCU in the system, a separate set of firmware was required for each one. In this setup:

* mcu_a behaves as the central master of the whole system, dealing with all external communications to other subteams and components
* mcu_b behaves as a redundant source of data for the system
* mcu_c behaves as a redundant source of data for the system
