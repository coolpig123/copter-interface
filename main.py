from interface import Interface

copter = Interface("udpin:127.0.0.1:14550")

copter.set_mode("RTL")


