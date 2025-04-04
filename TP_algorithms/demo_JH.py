import argparse
import yaml
import json
import os
from Simulation.TP_with_recovery import TokenPassingRecovery
import RoothPath
from Simulation.tasks_and_delays_maker import *
from Simulation.simulation_new_recovery import SimulationNewRecovery
import subprocess
import sys
import logging

logging.basicConfig(filename='simulation_log.txt', level=logging.INFO,
                    format='%(asctime)s:%(levelname)s:%(message)s')

if __name__ == '__main__':
    # random.seed(0)
    parser = argparse.ArgumentParser()
    parser.add_argument('-k', help='Robustness parameter for k-TP', default=None, type=int)
    parser.add_argument('-p', help='Robustness parameter for p-TP', default=None, type=float)
    parser.add_argument('-pd', help='Expected probability of an agent of being delayed at any time step (p-TP)',
                        default=0.02, type=float)
    parser.add_argument('-p_iter', help='Number of times a new path can be recalculated if the one calculated '
                                        'before exceeds the probability threshold (p-TP)',
                        default=1, type=int)
    parser.add_argument('-a_star_max_iter', help='Maximum number of states explored by the low-level algorithm',
                        default=5000, type=int)
    parser.add_argument('-slow_factor', help='Slow factor of visualization', default=1, type=int)
    parser.add_argument('-not_rand', help='Use if input has fixed tasks and delays', action='store_true')

    args = parser.parse_args()

    if args.k is None:
        args.k = 0
    if args.p is None:
        args.p = 1

    with open(os.path.join(RoothPath.get_root(), 'config.json'), 'r') as json_file:
        config = json.load(json_file)
    args.param = os.path.join(RoothPath.get_root(), os.path.join(config['input_path'], config['input_name']))
    args.output = os.path.join(RoothPath.get_root(), 'output.yaml')

    # Read from input file
    with open(args.param, 'r') as param_file:
        try:
            param = yaml.load(param_file, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)

    dimensions = param['map']['dimensions']
    obstacles = param['map']['obstacles']
    non_task_endpoints = param['map']['non_task_endpoints']
    agents = param['agents']
    if args.not_rand:
        # Old fixed tasks and delays
        tasks = param['tasks']
        delays = param['delays']
    else:
        # Generate random tasks and delays
        tasks, delays = gen_tasks_and_delays(agents, param['map']['start_locations'], param['map']['goal_locations'],
                                             param['n_tasks'], param['task_freq'], param['n_delays_per_agent'])
    param['tasks'] = tasks
    param['delays'] = delays

    logging.info("Generated tasks: %s", tasks)
    logging.info("Generated delays: %s", delays)

    with open(args.param + config['visual_postfix'], 'w') as param_file:
        yaml.safe_dump(param, param_file)

    # Simulate
    simulation = SimulationNewRecovery(tasks, agents, delays=delays)
    tp = TokenPassingRecovery(agents, dimensions, obstacles, non_task_endpoints, simulation,
                              a_star_max_iter=args.a_star_max_iter, k=args.k,
                              replan_every_k_delays=False, pd=args.pd, p_max=args.p, p_iter=args.p_iter,
                              new_recovery=True)
    while tp.get_completed_tasks() != len(tasks):
        simulation.time_forward(tp)
        logging.info("Simulation time step progressed")
    logging.info("Total number of replans: %d", tp.get_n_replans())
    logging.info("Completed tasks times: %s", tp.get_completed_tasks_times())

    cost = 0
    for path in simulation.actual_paths.values():
        cost = cost + len(path)
    output = {'schedule': simulation.actual_paths, 'cost': cost,
              'completed_tasks_times': tp.get_completed_tasks_times(),
              'n_replans': tp.get_n_replans()}
    with open(args.output, 'w') as output_yaml:
        yaml.safe_dump(output, output_yaml)

    create = [sys.executable, '-m', 'Utils.Visualization.visualize_jh', '-slow_factor', str(args.slow_factor)]
    subprocess.call(create)
