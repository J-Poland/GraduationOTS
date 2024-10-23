import sys
import io
import os
import pandas as pd
from datetime import datetime
from ema_workbench import (Model, MultiprocessingEvaluator, perform_experiments, Constant, RealParameter,
                           ScalarOutcome, SequentialEvaluator, Policy)
from ema_workbench.em_framework.samplers import LHSSampler
from ots_models.ots_vehicle_automation_model import VehicleAutomationModel

import random
import numpy as np


# class to redirect console output to capture and show it at the same time
class DualLogger:
    def __init__(self, file_path, file_name):
        self.terminal = sys.stdout  # save the current console output (stdout)
        os.makedirs(file_path, exist_ok=True)
        self.log = open(os.path.join(file_path, file_name), "w")  # open the file to write console output
        self.log_closed = False

    def write(self, message):
        self.terminal.write(message)  # write to the console
        self.log.write(message)       # write same message to the log file

    def flush(self):
        self.terminal.flush()  # ensure console output is flushed
        if not self.log_closed:
            self.log.flush()       # ensure file output is flushed

    def close_log(self):
        self.log.close()
        self.log_closed = True


# function to create valid policies
def define_policies():
    return [
        Policy('Policy 1', **{'level0_fraction': 0.25, 'level1_fraction': 0.25, 'level2_fraction': 0.25, 'level3_fraction': 0.25,
                              'in_vehicle_distraction': True, 'road_side_distraction': False}),
    ]


# function to add other levers to policies
def add_levers_to_policies(existing_policies, lhs_samples, num_samples):
    updated_policies = []
    for policy in existing_policies:
        for i in range(num_samples):
            sampled_values = {param: lhs_samples[param][i] for param in lhs_samples}
            combined_policy = {**policy.kwargs, **sampled_values}
            updated_policies.append(Policy(policy.name + f'_sample_{i}', **combined_policy))
    return updated_policies


# function to create model
def create_model(_seed):
    # create runnable OpenTrafficSim model
    _ots_model = VehicleAutomationModel(experiment_name=experiment_name, seed=_seed, user_feedback=True)

    # compile OTS project
    _ots_model.compile_project()

    # define model for EMA-Workbench
    _ema_model = Model('VehicleAutomationModel', function=_ots_model.run_model)

    # get defined model policies
    _policies = define_policies()

    # define model constants
    _ema_model.constants = [Constant('warm_up_time', 0),
                            Constant('sample_time', 2400)
                            ]  # 28800 = 8 hours of traffic simulation, 1800 = 30 min

    # define levers outside of policies
    _ema_model.levers = []

    # combine these levers with the defined policies
    if len(_ema_model.levers) > 0:
        lhs = LHSSampler()
        samples_per_parameter = 10  # 100 samples with LHS
        lhs_samples_dict = lhs.generate_samples(parameters=_ema_model.uncertainties, size=samples_per_parameter)
        _policies = add_levers_to_policies(_policies, lhs_samples_dict, samples_per_parameter)

    # define uncertain variables
    _ema_model.uncertainties = [RealParameter('main_demand', 2000, 6000),
                                RealParameter('ramp_demand', 400, 1000)]

    # list outcome variables of interest
    _ema_model.outcomes = [ScalarOutcome('mean_density'),
                           ScalarOutcome('mean_flow'),
                           ScalarOutcome('mean_speed'),
                           ScalarOutcome('mean_travel_time')]

    return _ots_model, _ema_model, _policies


# run EMA-Workbench experiment
if __name__ == '__main__':

    # experiment name and number
    experiment_string = 'warm_up_runs'
    experiment_number = 1
    experiment_name = f'{experiment_string}_{experiment_number}'

    # create folder for experiment results
    results_dir = fr'results'
    results_name = f'{experiment_string}_results_v{experiment_number}'
    results_folder = os.path.join(results_dir, f'{results_name}')

    # redirect sys.stdout to capture console output
    # while also printing messages to the original console
    console_output = io.StringIO()
    original_stdout = sys.stdout
    log_file_folder = results_folder
    log_file_name = 'console_log.txt'
    sys.stdout = DualLogger(log_file_folder, log_file_name)

    # show start of programme
    print(f'{datetime.now().time().strftime("%H:%M:%S")}: '
          f'Simulation of the {experiment_name} experiment has started.')

    # run experiments for multiple seeds
    seeds = [0, 1, 2, 3]
    # select scenarios per policy
    num_scenarios = 10
    for seed in seeds:
        print('\n'
              f'{datetime.now().time().strftime("%H:%M:%S")}: '
              f'Run experiment for seed {seed}:\n')

        # ensure reproducibility
        # otherwise the EMA workbench can provide different input values for the same seed
        random.seed = seed
        np.random.seed(seed)

        # create and compile Java model
        ots_model, ema_model, policies = create_model(seed)

        # run experiments
        # MultiprocessingEvaluator will run multiple simulations at the same time, but laptop memory cannot handle this
        with SequentialEvaluator(ema_model) as evaluator:
            results = evaluator.perform_experiments(policies=policies, scenarios=num_scenarios)

        # unpack results
        experiments, outcomes = results

        # convert outcomes to a DataFrame
        df_outcome = pd.DataFrame.from_dict(outcomes)

        # combine experiments and outcomes for analysis
        df_results: pd.DataFrame = experiments.join(df_outcome)

        # check/restore os directory before using os paths
        os.chdir(ots_model.original_os_dir)

        # save DataFrame
        results_path = os.path.join(results_folder, f'{results_name}_seed{seed}.csv')
        os.makedirs(os.path.dirname(results_path), exist_ok=True)
        df_results.to_csv(results_path)

        # show successful save of results
        print(f'{datetime.now().time().strftime("%H:%M:%S")}: '
              f'Runs for seed {seed} completed! Still {len(seeds) - 1} seeds to go...')
        print(f'\nThe experiment results are successfully saved to:')
        print(f'{results_path}')

    # show end of programme
    print('\n'
          f'{datetime.now().time().strftime("%H:%M:%S")}: '
          'Experiment has finished.')

    # end logging of console output
    sys.stdout.close_log()

