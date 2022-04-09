"""
Method definitions for sat_htn
Zachary Wilton April 7, 2022
"""

import gtpyhop

################################################################################
# Helper functions for Satellite Domain

def safe_cost_move(state, dir1, dir2):
    """Just a wrapper for slew time that is safe if we try to move where we already are."""
    if dir1 == dir2:
        return 0
    else:
        return state.slew_time[(dir1, dir2)]

def safe_cost_move_end_position(state, mgoal, satellite, expected_position):
    """Calculates the necessary cost for a satellite to move to its final position"""
    if satellite not in mgoal.pointing.keys():
        return 0
    else : return safe_cost_move(state, mgoal.pointing[satellite], expected_position)

def safe_min(lst):
    """Just a wrapper for min that is ok with bad input."""
    if lst is None or len(lst) == 0:
        return 9999999
    return min(lst)

def calculate_cost_to_acquire_image(state, satellite, direction, mode):
    """
    Returns how much fuel will be used by a given satellite accomplishing a given take_image objective
    Returns None if the satellite is not capable of actually taking said image.
    """
    best_cost = 99999
    best_instrument = None
    cur_pos = state.pointing[satellite]
    for instrument in state.instruments_on_satellite[satellite]:
        if mode in state.supports[instrument]:
            # Check how much fuel it will cost us to take an image with a given instrument
            if state.power_on[instrument] and state.calibrated[instrument]:
                cur_cost = safe_cost_move(state, cur_pos, direction)
            else :
                cal_pos = state.calibration_target[instrument]
                cur_cost = safe_cost_move(state, cur_pos, cal_pos)
                cur_cost += safe_cost_move(state, cal_pos, direction)
            if cur_cost < best_cost:
                best_cost = cur_cost
                best_instrument = instrument
    if best_cost < 99999:
        return (best_cost, best_instrument)


def images_still_needed(state, mgoal):
    """Calculates the take_image objectives that still need to be achieved."""
    return {x  for x in mgoal.have_image if x not in state.have_image}

def calculate_options_costs(state, mgoal, needed_images):
    """
    Calculates the fuel cost of every possible hunt_image task.
    Only hunt_image tasks that do not result in an immediate or near loss are considered.
    The returned hunt_image options are guaranteed to:
        - properly execute all the way down to, and including, the action level
        - fulfill one take_image goal
        - Leave enough fuel for the satellite to move to its required final location.
    """
    costs = {sat:{} for sat in state.satellites}
    for satellite in state.satellites:
        for img_dir, img_mode in needed_images:
            result = calculate_cost_to_acquire_image(state, satellite, img_dir, img_mode)
            if result is not None and state.fuel[satellite] >= result[0] + safe_cost_move_end_position(state, mgoal, satellite, img_dir) and state.data_capacity[satellite] >= state.data[(img_dir,img_mode)]:
                #print("HEREEE")
                #print(result[0] + safe_cost_move_end_position(state, mgoal, satellite))
                cost = result[0]
                if cost in costs[satellite].keys():
                    costs[satellite][cost].append((img_dir, img_mode, result[1]))
                else:
                    costs[satellite][cost] = [(img_dir, img_mode, result[1])]
    return costs

def get_cheapest_satellite(state):
    """Gets the satellite that can currently take an image with the lowest cost."""
    best_min_cost = 999999
    best_satellite = None
    for satellite in state.satellites:
        cur_min = safe_min(state.costs[satellite].keys())
        if cur_min < best_min_cost:
            best_min_cost = cur_min
            best_satellite = satellite
    return best_satellite

def calculate_number_achievable_satellites(state, needed_images):
    """
    For each remaining take_image goal, calculate the number of satellites that could actually achieve it as a part of a successful plan.
    If any goals have 0 possible satellites, we should just backtrack now.
    if any goals have 1 possible satellite, we should do that immediately
    """
    possible_satellites_for_image_goals = {goal: [] for goal in needed_images}
    possible_plans_for_image_goals = {goal: [] for goal in needed_images}
    for satellite in state.satellites:
        for cost in state.costs[satellite].keys():
            for img_dir, img_mode, instrument in state.costs[satellite][cost]:
                possible_satellites_for_image_goals[(img_dir,img_mode)].append(satellite)
                possible_plans_for_image_goals[(img_dir, img_mode)].append((cost, satellite, instrument))
    return (possible_satellites_for_image_goals, possible_plans_for_image_goals)

def unachievable_goal(needed_images, possible_satellites_for_image_goals):
    for goal in needed_images:
        if len(possible_satellites_for_image_goals[goal]) == 0:
            print(f"UNACHIEVABLE GOAL: {goal}")
            return True
    return False



################################################################################
### methods for Satellite Domain

def m_disbatch(state, mgoal):
    """
    This Method is for the achieve task and is the highest overview of the plan.
    We collect some information here, so that we don't waste time recomputing it
        multiple times deeper in the search tree.
    Each time this method is called, it will either plan for the capture of one image
        or it will tell the satellites to move to their final destination, if we're done.
        It may also give up if it computes that this branch is destined to fail.
    """
    # Figure out which image goals still need to be completed.
    needed_images = images_still_needed(state, mgoal)
    # If we still have more images to get
    if needed_images:
        state.costs = calculate_options_costs(state, mgoal, needed_images)
        state.possible_satellites_for_image_goals, state.possible_plans_for_image_goals = calculate_number_achievable_satellites(state, needed_images)
        # If there is a extant goal that cannot be completed, backtrack now.
        if unachievable_goal(needed_images, state.possible_satellites_for_image_goals):
            return
        state.best_satellite = get_cheapest_satellite(state)
        return [('choose_next_image',), ('achieve', mgoal)]
    # If we only need to move the satellites to their final destination
    else:
        return [('final_move', mgoal)]

def m_move_satellites_to_final_position(state, mgoal):
    """
    This method simply informs all the satellites where they need to be to finish out the problem.
    """
    return [('move', sat, goal)   for sat, goal in mgoal.pointing.items()]

def m_move(state, satellite, direction):
    """
    This method ensures that a satellite is at direction using actions.
    """
    cur_pos  = state.pointing[satellite]
    # If the satellite is already there, success with nothing to be done.
    if cur_pos == direction:
        return []
    return [('turn_to', satellite, direction, cur_pos)]
    # Otherwise, we can't make it

def m_go_take_image(state, satellite, direction, instrument, mode):
    """
    This method is for the hunt_image task.
    It will complete one have_image goal by taking whatever
        actions are required to complete just that one objective
    This is purely a refinement method, no planning is done here. 
    """
    todo = []
    # Turn off the wrong instrument, if we need to
    cur_instrument = state.current_powered_instrument[satellite]
    if cur_instrument != instrument and cur_instrument is not None:
        todo.append(('switch_off', cur_instrument, satellite))
    # Turn on the correct instrument, if we need to
    if state.current_powered_instrument[satellite] != instrument:
        todo.append(('switch_on', instrument, satellite))
    # Calibrate the instrument, if we need to
    if not state.calibrated[instrument] or not state.power_on[instrument]:
        todo.append(('move', satellite, state.calibration_target[instrument]))
        todo.append(('calibrate', satellite, instrument,state.calibration_target[instrument] ))
    # move and take the image
    todo.append(('move',satellite, direction))
    todo.append(('take_image', satellite, direction, instrument, mode))
    return todo

##############################################################################################
### Methods specifically for choose_next_image. This is where all of the backtracking is done
def m_choose_cheapest_sat_1(state):
    """chooses to complete the cheapest possible objective"""
    best_cost = min(state.costs[state.best_satellite].keys())
    img_dir, img_mode, instrument  = state.costs[state.best_satellite][best_cost][0]
    return [('hunt_image', state.best_satellite, img_dir, instrument, img_mode)]

def m_choose_last_chance(state):
    """
    If there is a goal that can only be achieved by 1 satellite, we should do that now.
    If there are several possible plans, execute the cheapest.
    """
    for goal, satellites in state.possible_satellites_for_image_goals.items():
        if len(satellites) == 1:
            priority_satellite = satellites[0]
            cheapest_cost = 99999
            cheapest_instrument = None
            for cost, _, instrument in state.possible_plans_for_image_goals[goal]:
                if cost < cheapest_cost:
                    cheapest_cost = cost
                    cheapest_instrument = instrument
            return [('hunt_image', priority_satellite, goal[0], cheapest_instrument,goal[1])]
                


gtpyhop.declare_task_methods('achieve', m_disbatch)
gtpyhop.declare_task_methods('final_move', m_move_satellites_to_final_position)        
gtpyhop.declare_task_methods('move', m_move)
gtpyhop.declare_task_methods('choose_next_image', m_choose_last_chance, m_choose_cheapest_sat_1 )
gtpyhop.declare_task_methods('hunt_image', m_go_take_image)