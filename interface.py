from pymavlink import mavutil


class Interface:

    # Constructor
    def __init__(self, address):
        # creating the master connection
        self.master = mavutil.mavlink_connection(address)
        # waiting for a heartbeat before continuing
        self.master.wait_heartbeat()

    # Function to arm copter
    def arm(self):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0)

        print("ARM : WAITING FOR VEHICLE TO ARM")
        self.master.motors_armed_wait()
        print('ARM : ARMED')

    # Function to disarm copter
    def disarm(self):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0)

        print("DISARM : WAITING FOR VEHICLE TO DISARM")
        self.master.motors_disarmed_wait()
        print("DISARMED : DISARMED")

    # Function to change mode
    def set_mode(self, mode):
        # Check if mode is available
        if mode not in self.master.mode_mapping():
            print('Unknown mode : {}'.format(mode))
            print('Try:', list(self.master.mode_mapping().keys()))

        # Get mode id
        mode_id = self.master.mode_mapping()[mode]

        # Change mode
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id)

    # Function to takeoff to a specified altitude
    def takeoff(self, alt):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, alt)

    # Function to reposition drone
    def reposition(self, longitude, latitude, alt, speed):
        self.master.mav.send(
            mavutil.mavlink.MAVLink_set_position_target_global_int_message(10, self.master.target_system,
                                                                           self.master.target_component,
                                                                           mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                                                           int(0b110111111000),
                                                                           int(-35.3629849 * 10 ** 7),
                                                                           int(149.1649185 * 10 ** 7), 10, 0, 0, 0, 0,
                                                                           0, 0, 1.57, 0.5)
        )

