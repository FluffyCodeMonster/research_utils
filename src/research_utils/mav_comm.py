# FT 21/4/26
# Code for communicating with UAV via Mavlink

import time

import numpy as np
from pymavlink import mavutil


# TODO Make this a singleton class?
class MissionBuilder:
    def __init__(self, target_system=0, target_component=0):
        self.target_system = target_system
        self.target_component = target_component

        self.mission_items = []
        self.mission_item_names = []
        self.seq = 0

        # Waypoint zero is the home location (for RTL) in ArudPlane - a dummy waypoint is sent for position zero.
        # See: https://mavlink.io/en/services/mission.html#mavlink_commands (under ArduPilot heading)
        self.add_waypoint(0, 0, 0, 0, 0, "dummy waypoint")

    def _count(self):
        return len(self.mission_items)

    def _add_mission_item(self, item, name):
        self.mission_items.append(item)
        self.mission_item_names.append(name)
        self.seq += 1

    # TODO Check this works in the right way
    def add_waypoint(
        self, lat, lon, rel_alt, accept_radius=0, pass_radius=0, name="waypoint"
    ):
        wp = mavutil.mavlink.MAVLink_mission_item_int_message(
            self.target_system,
            self.target_component,
            self.seq,  # seq
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,  # frame
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,  # command
            1,  # current   TODO what does this do?
            1,  # autocontinue
            0,  # hold time (ignored by fixed wing)
            accept_radius,  # acceptance radius
            pass_radius,  # pass radius
            np.nan,  # desired yaw angle at waypoint
            int(lat * 1e7),  # latitude
            int(lon * 1e7),  # longitude
            rel_alt,
        )  # relative altitude
        # 0)          # mission_type

        self._add_mission_item(wp, name)

    # Adds unlimited loiter
    def add_loiter_unlimited(
        self,
        lat,
        lon,
        rel_alt,
        loiter_radius
    ):
        l = mavutil.mavlink.MAVLink_mission_item_int_message(
            self.target_system,
            self.target_component,
            self.seq,  # seq
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,  # frame
            mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,  # command
            1,  # current   TODO what does this do?
            1,  # autocontinue
            # == MAV_CMD_NAV_LOITER_UNLIM ==
            0,  # np.nan,         # Param 1: empty
            0,  # np.nan,         # Param 2: empty
            loiter_radius,  # Param 3: radius
            0,  # np.nan,         # Param 4: yaw
            int(lat * 1e7),  # latitude
            int(lon * 1e7),  # longitude
            rel_alt,
        )  # relative altitude
        # ====
        # 0)          # mission_type

        self._add_mission_item(l, "unlimited loiter")

    # Adds loiter to a given altitude
    def add_loiter_to_alt(
        self,
        lat,
        lon,
        rel_alt,
        loiter_radius
    ):
        l = mavutil.mavlink.MAVLink_mission_item_int_message(
            self.target_system,
            self.target_component,
            self.seq,  # seq
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,  # frame
            mavutil.mavlink.MAV_CMD_NAV_LOITER_TO_ALT,  # command
            1,  # current   TODO what does this do?
            1,  # autocontinue
            # == MAV_CMD_NAV_LOITER_TO_ALT ==
            1,  # Param 1: Heading required - 1 to leave in the direction of the next waypoint
            loiter_radius,  # Param 3: radius
            0,  # Param 3: empty
            1,  # Param 4: Xtrack location - 1 to track direct line to next waypoint
            int(lat * 1e7),  # latitude
            int(lon * 1e7),  # longitude
            rel_alt,
        )  # relative altitude
        # ====
        # 0)          # mission_type

        self._add_mission_item(l, "loiter to alt")

    def add_speed_change(self, va):
        s = mavutil.mavlink.MAVLink_mission_item_int_message(
            self.target_system,
            self.target_component,
            self.seq,  # seq
            mavutil.mavlink.MAV_FRAME_MISSION,  # frame
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,  # command
            1,  # current   TODO what does this do?
            1,  # autocontinue
            # == MAV_CMD_DO_CHANGE_SPEED ==
            0,  # Param 1: 0 for airspeed
            va,  # Param 2: speed (m/s)
            0,  # Param 3: throttle (%) - TODO assume this has no effect if assigning airspeed?
            0,  # Param 4: empty
            0,  # Param 5: empty
            0,  # Param 6: empty
            0,
        )  # Param 7: empty
        # ====
        # 0)          # mission_type

        self._add_mission_item(s, "speed change")

    # lat, lon and rel_alt provided so that aircraft unlatches in direction of next waypoint
    def add_scripttime(self, time, cmd):
        s = mavutil.mavlink.MAVLink_mission_item_int_message(
            self.target_system,
            self.target_component,
            self.seq,  # seq
            mavutil.mavlink.MAV_FRAME_MISSION,  # frame
            mavutil.mavlink.MAV_CMD_NAV_SCRIPT_TIME,  # command
            1,  # current   TODO what does this do?
            1,  # autocontinue
            # == MAV_CMD_NAV_SCRIPT_TIME ==
            cmd,  # Param 1: time
            time,  # Param 2: empty
            0,  # Param 3: empty
            0,  # Param 4: empty
            # int(lat*1e7),        # latitude
            # int(lon*1e7),        # longitude
            # rel_alt)    # relative altitude
            0,  # Param 5: empty
            0,  # Param 6: empty
            0,
        )  # Param 7: empty
        # # ====
        # # 0)          # mission_type

        self._add_mission_item(s, "script time")

    # Adds infinite jump command
    # num_back: number of waypoints back to jump to
    def add_jump(self, num_back):
        j = mavutil.mavlink.MAVLink_mission_item_message(
            self.target_system,
            self.target_component,
            self.seq,  # seq
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,  # frame # TODO Should I be using this?
            mavutil.mavlink.MAV_CMD_DO_JUMP,  # command
            1,  # current   TODO what does this do?
            1,  # autocontinue
            # == MAV_CMD_DO_JUMP ==
            self.seq - num_back,  # Sequence number
            -1,  # Repeat count
            0,  # Empty
            0,  # Empty
            0,  # Empty
            0,  # Empty
            0,
        )  # Empty
        # ====
        # 0)          # mission_type

        self._add_mission_item(j, "jump")

    # Just for testing in SITL
    def add_takeoff(self, yaw_deg, min_pitch_deg, lat, lon, rel_alt):
        # Check that seq is 1
        if self.seq != 1:
            raise Exception("Takeoff waypoint must be added before any other waypoints")

        t = mavutil.mavlink.MAVLink_mission_item_int_message(
            self.target_system,
            self.target_component,
            self.seq,  # seq
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,  # frame
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,  # command
            1,  # current   TODO what does this do?
            1,  # autocontinue
            min_pitch_deg,  # minimum pitch (with airspeed sensor) / desired pitch without sensor
            0,  # not a valid field for takeoff
            0,  # not a valid field for takeoff
            yaw_deg,  # yaw (NaN to use the current system yaw heading mode)
            int(lat * 1e7),  # latitude
            int(lon * 1e7),  # longitude
            rel_alt,
        )  # relative altitude
        # 0)          # mission_type

        self._add_mission_item(t, "takeoff")

    # Just for testing in SITL
    def add_landing(self):
        # Lock the mission builder after this - nothing else should be allowed to be added
        pass

    def upload(self, conn):
        print("Uploading plan")
        num_waypoints = self._count()

        # If only the dummy waypoint is present
        if num_waypoints == 1:
            print("No mission plan to upload")
        else:
            conn.waypoint_count_send(num_waypoints)

            # TODO 'while True' - does this have the potential to hang?
            while True:
                msg = conn.recv_match(
                    type=["MISSION_REQUEST", "MISSION_ACK"], blocking=True
                )
                if msg.msgname == "MISSION_ACK":
                    # print(msg.type)
                    if msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                        print("Plan uploaded")
                        break
                    else:
                        raise Exception("An error occurred sending waypoints")
                else:
                    # ArduPlane doesn't use item zero - this is the home location
                    seq = msg.seq
                    print(f"Sending {self.mission_item_names[seq]} command {seq}")
                    conn.mav.send(self.mission_items[seq])


def connect(conn_str, setup_wait_s=2):
    # Connect
    print(f"Connecting to UAV on {conn_str}")
    conn = mavutil.mavlink_connection(conn_str)

    # Wait for heartbeat
    conn.wait_heartbeat()
    print("Connection established")

    # Wait to set up
    time.sleep(setup_wait_s)

    return conn


def read_param(conn, param_str):
    conn.mav.param_request_read_send(
        conn.target_system,
        conn.target_component,
        bytes(param_str, "utf-8"),
        -1,  # Use the 'param_id' field to select the parameter to send
    )

    # TODO Probably not the best way to do this
    while (
        msg := conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=5).to_dict()
    )["param_id"] != param_str:
        pass

    return msg["param_value"]

    # msg = conn.recv_match(type='PARAM_VALUE', blocking=True) # , timeout=5)

    # if (msg.param_id == param_str):
    #     return msg.param_value
    # else:
    #     # TODO Probably not the best way to do this!
    #     raise Exception("Unexpected parameter returned while reading")


# Update a parameter, and verify that the update has worked successfully.
def update_param(conn, param_str, val):
    print(f"Setting param {param_str} to value {val}")
    val = float(val)  # val has to be a float

    conn.mav.param_set_send(
        conn.target_system,
        conn.target_component,
        bytes(param_str, "utf-8"),
        val,
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
    )

    # Code from: https://www.ardusub.com/developers/pymavlink.html#read-and-write-parameters
    # Check for broadcast message - check that parameter has been received and updated successfully
    # This has to happen quickly, else it will be missed?
    msg = conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=100)  # 5)
    if msg is None:
        raise Exception("Parameter not set successfully!")

    # Read new value to confirm parameter set correctly
    read_param_val = read_param(conn, param_str)

    # if (msg['param_id'] == param_str) and (np.isclose(msg['param_value'], val)):
    if np.isclose(read_param_val, val):
        print("Confirm parameter set successfully")
    else:
        raise Exception(f"Parameter not set successfully! Error code: {'a'}.")


def set_message_rate(conn, msg_id, intv=3e4):
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        msg_id,
        intv * 1e6,  # interval
        0,
        0,
        0,
        0,
        0,
    )
    msg = conn.recv_match(type="COMMAND_ACK", blocking=True)
    result = msg.to_dict()["result"]
    if result != 0:
        raise Exception(
            f"Message rate not set successfully. Command result (MAV_RESULT) code: {result}."
        )


# Arm
# Code from: https://www.ardusub.com/developers/pymavlink.html#armdisarm-the-vehicle
def arm(conn):
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,
        0,
        0,
        0,
        0,
        0,
        0,
    )

    # wait until arming confirmed (can manually check with master.motors_armed())
    print("Waiting for the vehicle to arm")
    conn.motors_armed_wait()
    print("Armed!")


def set_mode_auto(conn):
    print("Sending AUTO mode command")
    conn.mav.set_mode_send(
        conn.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        conn.mode_mapping()["AUTO"],
    )

    # TODO Still have to check for acknowledgement, as shown here: https://www.ardusub.com/developers/pymavlink.html#change-flight-mode
