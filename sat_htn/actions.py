"""
Action definitions for the sat_htn Domain
"""

import gtpyhop

"""
Each gtpyhop action is a Python function. The 1st argument is the current
state, and the others are the action's usual arguments. This is analogous to
how methods are defined for Python classes (where the first argument is
always the name of the class instance). For example, the function
pickup(s,b) implements the action ('pickup', b).

The sat actions use these state variables:
-supports[instrument]: a list of supported modes for the instrument
-calibration_target:[instrument]: the direction that the instrument is calibrated for
-on_board[instrument]: the Satellite that that the instrument is on
-power_avail[satellite]: True if the satellite has available power; False otherwise
-pointing[satellite]: the direction the satellite is pointing
-data_capacity[satellite]: The amount of data left on the satellite
-fuel[satellite]: The amount of fuel left on the satellite
-data[(direction, mode)]: The amount of data an image will take
-slew_time[(direction, direction)]: the amount of fuel it will take to pivot from direction to direction
-power_on[instrument]: True if the instrument has power; False otherwise
-calibrated[instrument]: True if the instrument is calibrated; False otherwise
-current_powered_instrument[satellite]: The instrument on the Satellite with power
-instruments_on_satellite[satellite]: a list of the instruments on a satellite.
-data_stored: The total amount of data used
-fuel_used: The total amount of fuel used
-instruments: a list of the instruments in the environment
-satellites: a list of the satellites in the environment
-modes: a list of the modes in the environment
"""
def  turn_to(s, sat, d_new, d_prev):
    if s.pointing[sat] == d_prev and d_new != d_prev and s.fuel[sat] >= s.slew_time[(d_new, d_prev)]:
        s.pointing[sat] = d_new
        s.fuel[sat] -= s.slew_time[(d_new, d_prev)]
        s.fuel_used += s.slew_time[(d_new, d_prev)]
        return s

def switch_on(s, instrument, satellite):
    if s.on_board[instrument] == satellite and s.power_avail[satellite]:
        s.power_on[instrument] = True
        s.current_powered_instrument[satellite] = instrument
        s.calibrated[instrument] = False
        s.power_avail[satellite] = False
        return s

def switch_off(s, instrument, satellite):
    if s.on_board[instrument] == satellite and s.power_on[instrument]:
        s.power_on[instrument] = False
        s.power_avail[satellite] = True
        s.current_powered_instrument[satellite] = None
        return s

def calibrate(s, satellite, instrument, direction):
    if s.on_board[instrument] == satellite and s.calibration_target[instrument] == direction and s.pointing[satellite] == direction and s.power_on[instrument]:
        s.calibrated[instrument] = True
        return s

def take_image(s, satellite, direction, instrument, mode):
    if s.calibrated[instrument] and s.on_board[instrument] == satellite and mode in s.supports[instrument] and s.power_on[instrument] and s.pointing[satellite] == direction and s.data_capacity[satellite] >= s.data[(direction,mode)]:
        s.data_capacity[satellite] -= s.data[(direction,mode)]
        s.have_image.append((direction,mode))
        s.data_stored += s.data[(direction,mode)]
        return s

# Tell Pyhop what the actions are
#
gtpyhop.declare_actions(turn_to, switch_on, switch_off, calibrate, take_image)
