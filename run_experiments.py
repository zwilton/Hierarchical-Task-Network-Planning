import argparse
import os
import random
import sys
import timeit
import re
sys.path.append('./GTPyhop')
import gtpyhop as gtpyhop
sys.setrecursionlimit(5000)

parser = argparse.ArgumentParser(description='Setup Experiments')
parser.add_argument('domain',help="Choose either block or sat")
parser.add_argument('planner', help="Choose either htn or domain_independent")
parser.add_argument('repeats', help="Number of times each experiment will be repeated", type=int)
parser.add_argument('schedule',  nargs="+", help="The trial sizes, as an array", type=int)
parser.add_argument('--seed', type=int, default=10)
parser.add_argument('--sat_params', nargs="+", help="The extra parameters used in the Satellite domain: num_satellites, num_modes, num_instruments, num_observations", default= [10, 5, 10, 2])
args = parser.parse_args()
##print(args.domain, args.planner, args.repeats, args.schedule, args.sat_params)

"""Begin Methods for Domain Independent Planner"""

def parse_block_line(str, add_state_vars = False, n = 0):
    """A Helper function for parsing block world problems"""
    output = ""
    for i, v in enumerate(str.split()):
        #Add block positions
        if v == "0" :
            output += f"(ontable b{i+1})\n" 
        else :
            output += f"(on b{i+1} b{v})\n"
    if add_state_vars:
        #Need to add which blocks are clear
        clear_blocks = [x for x in range(1,n+1) if x not in [int(x) for x in str.split()]]
        for block in clear_blocks : 
            output += f"(clear b{block})\n"
        output += "(handempty)"
    return output

def create_block_problem(n, cur_repeat):
    """Converts the generated block problem into a PDDL problem"""
    #Add preliminaries  
    output = ""
    seed = random.random() * 100000
    stream = os.popen(f"./bwstates_src/bwstates -n {n} -r {seed}")
    gen_text = stream.readlines()
    output += f"(define (problem BW-{n}-{cur_repeat+1})\n"
    output += f"(:domain blocks)\n"
    output += "(:objects"
    #Add block objects
    for i in range(n):
        output += f" b{i+1}"
    output += ")\n(:init\n"
    #Add initial state 
    output += parse_block_line(gen_text[1][:-1], True, n) +")\n"
    #Add goal state
    output += "(:goal\n(and"
    output += parse_block_line(gen_text[3][:-1]) + ")\n)\n)"
    return(output)

def create_sat_problem(n):
    seed = random.random() * 100000
    if args.sat_params is not None:
        max_instuments = args.sat_params[2]
        num_satellites = args.sat_params[0]
        num_modes = args.sat_params[1]
        num_observations = args.sat_params[3]
        num_targets = n
        stream = os.popen(f"./satellite-generator/satgen -n {seed} {num_satellites} {max_instuments} {num_modes} {num_targets} {num_observations}")
    else:
        stream = os.popen(f"./satellite-generator/satgen -n {seed} {n} {n} {n} {n} {n}")
    return stream.read()

def write_current_problem(problem):
    f = open("current_problem.pddl", "w")
    f.write(problem)
    f.close()

def run_domain_independent_experiment(command, log_file,n, add_sat_params = False):
    #Send request to terminal
    stream = os.popen(command)
    #Process results to file
    results = stream.readlines()
    #print(results[-2])
    f = open(log_file, "a")
    if args.domain == "block" :
        f.write(f"\n{n}, {results[-2].split()[0]}, {results[-2].split()[1]}, {results[-3].split()[4]}, {int(results[-11].split()[0][:-1]) +1}")
    else :
        f.write(f"\n{n}, {results[-2].split()[0]}, {results[-2].split()[1]}, {results[-3].split()[4]}, {int(results[-11].split()[0][:-1]) +1}, "+ 
        f"{args.sat_params[0]}, {args.sat_params[1]}, {args.sat_params[2]}, {args.sat_params[3]}")
    f.close()

""""Begin Methods for HTN Planner"""""
def run_block_problem_htn(n, current_repeat):
    seed = random.random() * 100000
    stream = os.popen(f"./bwstates_src/bwstates -n {n} -r {seed}")
    gen_text = stream.readlines()
    #print(gen_text)
    state = gtpyhop.State('state')
    state.pos = {i+1:int(p) if int(p) > 0 else 'table'  for  i, p in enumerate(gen_text[1].split())}
    state.clear={i+1: True for i in range(n)}
    state.clear.update({int(i):False for i in gen_text[1].split()})
    del state.clear[0]
    state.holding={'hand':False}
    state.display('Initial state is')
    goal = gtpyhop.Multigoal('goal')
    goal.pos={i+1:int(p) if int(p) > 0 else 'table'  for  i, p in enumerate(gen_text[3].split())}
    goal.display()
    start_time = timeit.default_timer()
    gtpyhop.verbose = 3
    plan = gtpyhop.find_plan(state, [('achieve', goal)])
    time = timeit.default_timer() - start_time
    length = len(plan)
    f = open("results_data/block_htn.csv", "a")
    f.write(f"{n}, {time}, {length}\n")
    f.close()


    
def init_htn_blocks():
    the_domain = gtpyhop.Domain("blocks")
    import Examples.blocks_htn.actions
    import Examples.blocks_htn.methods

def init_htn_sat():
    the_domain = gtpyhop.Domain("sat")
    import sat_htn.actions
    import sat_htn.methods

def run_htn_block_experiment():
    import GTPyhop.Examples.blocks_htn.actions
    gtpyhop.find_plan(state2,[('achieve',goal2a)])

def run_sat_problem_htn(n):
    problem = create_sat_problem(n)
    print(problem)
    state = gtpyhop.State('state')
    #Create starting state
    s_0_text = problem[problem.find(":init") : problem.find(")\n(:goal")]
    g_text = problem[problem.find("and") : problem.find("\n))")]
    #print(g_text)
    #print(s_0_text)
    pattern = re.compile("[a-zA-Z0-9]+\s\-\ss")
    state.satellites = [x[:-4] for x in pattern.findall(problem)]
    pattern = re.compile("[a-zA-Z0-9]+\s\-\si")
    state.instruments = [x[:-4] for x in pattern.findall(problem)]
    pattern = re.compile("[a-zA-Z0-9]+\s\-\sm")
    state.modes = [x[:-4] for x in pattern.findall(problem)]
    pattern = re.compile("[a-zA-Z0-9]+\s\-\sd")
    directions = [x[:-4] for x in pattern.findall(problem)]
    ##print(instruments, satellites, modes, directions)
    pattern = re.compile("supports\s\w+\s\w+")
    state.supports = {x:[] for x in state.instruments}
    for x in pattern.findall(s_0_text):
        state.supports[x.split()[1]].append(x.split()[2])
    #state.supports = {x.split()[1]:x.split()[2] for x in pattern.findall(s_0_text)}
    #print(state.supports)
    pattern = re.compile("calibration_target\s\w+\s\w+")
    state.calibration_target = {x.split()[1]:x.split()[2] for x in pattern.findall(s_0_text)}
    #print(state.calibration_target)
    pattern = re.compile("on_board\s\w+\s\w+")
    state.on_board = {x.split()[1]:x.split()[2] for x in pattern.findall(s_0_text)}
    #print(state.on_board)
    state.power_avail = {x:False for x in state.satellites}
    pattern = re.compile("power_avail\s\w+")
    state.power_avail.update({x.split()[1]:True for x in pattern.findall(s_0_text)})
    #print(state.power_avail)
    pattern = re.compile("pointing\s\w+\s\w+")
    state.pointing = {x.split()[1]:x.split()[2] for x in pattern.findall(s_0_text)}
    #print(state.pointing)
    pattern = re.compile("data_capacity\s\w+\)\s\w+")
    state.data_capacity = {x.split()[1][:-1]:int(x.split()[2]) for x in pattern.findall(s_0_text)}
    #print(state.data_capacity)
    pattern = re.compile("fuel\s\w+\)\s\w+")
    state.fuel = {x.split()[1][:-1]:int(x.split()[2]) for x in pattern.findall(s_0_text)}
    #print(state.fuel)
    pattern = re.compile("data\s\w+\s\w+\)\s\w+")
    state.data = {(x.split()[1],x.split()[2][:-1]):int(x.split()[3]) for x in pattern.findall(s_0_text)}
    #print(state.data)
    pattern = re.compile("slew_time\s\w+\s\w+\)\s\w+")
    state.slew_time = {(x.split()[1],x.split()[2][:-1]):int(x.split()[3]) for x in pattern.findall(s_0_text)}
    #print(state.slew_time)
    state.data_stored = 0
    state.fuel_used = 0
    state.have_image = []
    state.calibrated = {instr : False for instr in state.instruments}
    state.current_powered_instrument = {sat: None for sat in state.satellites}
    state.power_on = {instr: False for instr in state.instruments} 
    state.instruments_on_satellite = {sat:[] for sat in state.satellites}
    for instrument in state.instruments:
        state.instruments_on_satellite[state.on_board[instrument]].append(instrument)

    #Construct Goal State
    goal = gtpyhop.Multigoal('goal')
    pattern = re.compile("pointing\s\w+\s\w+")
    goal.pointing = {x.split()[1]:x.split()[2] for x in pattern.findall(g_text)}
    #print(goal.pointing)
    pattern = re.compile("have_image\s\w+\s\w+")
    goal.have_image = [(x.split()[1],x.split()[2]) for x in pattern.findall(g_text)]
    #print(goal.have_image)

    #Run Experiment
    start_time = timeit.default_timer()
    gtpyhop.verbose = 3
    plan = gtpyhop.find_plan(state, [('achieve', goal)])
    time = timeit.default_timer() - start_time
    length = len(plan)
    f = open("results_data/sat_htn.csv", "a")
    f.write(f"{n}, {time}, {length}, {args.sat_params[0]}, {args.sat_params[1]}, {args.sat_params[2]}, {args.sat_params[3]}\n")
    f.close()


#Start Experiments
random.seed(args.seed)
#run Domain Independent Problems
if args.planner == "domain_independent":
    #Block Problems
    if args.domain == "block":
        for n in args.schedule: 
            for repeat in range(1,args.repeats+1):
                write_current_problem(create_block_problem(n,repeat))
                run_domain_independent_experiment("Metric-FF/ff -o BLKDomain.pddl -f current_problem.pddl -E -g 1 -h 4", "results_data/block_ind.csv", n)
    #Satellite Problems
    else:
        for n in args.schedule: 
            for repeat in range(1,args.repeats+1):
                write_current_problem(create_sat_problem(n))
                run_domain_independent_experiment("Metric-FF/ff -o SATDomain.pddl -f current_problem.pddl -E -g 1 -h 5", "results_data/sat_ind.csv", n)
#HTN Problems
else :
    #Block Problems
    if args.domain == "block":
        init_htn_blocks()
        for n in args.schedule: 
            for repeat in range(1,args.repeats+1):
                run_block_problem_htn(n, repeat)
    #Satellite Problems
    else:
        init_htn_sat()
        for n in args.schedule: 
            for repeat in range(1,args.repeats+1):
                run_sat_problem_htn(n)