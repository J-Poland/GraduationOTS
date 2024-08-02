from ema_workbench import (Model, MultiprocessingEvaluator, perform_experiments, Constant, RealParameter,
                           ScalarOutcome, SequentialEvaluator, Policy)
from ema_workbench.em_framework.samplers import LHSSampler
from datetime import datetime
import pandas as pd
import os
from ots_models.ots_vehicle_automation_model import VehicleAutomationModel


# function to create valid policies
def define_policies():
    return [
        Policy('Policy 1', **{'level0_fraction': 1.0, 'level1_fraction': 0, 'level2_fraction': 0, 'level3_fraction': 0}),
        Policy('Policy 2', **{'level0_fraction': 0, 'level1_fraction': 1.0, 'level2_fraction': 0, 'level3_fraction': 0}),
        Policy('Policy 3', **{'level0_fraction': 0, 'level1_fraction': 0, 'level2_fraction': 1.0, 'level3_fraction': 0}),
        Policy('Policy 4', **{'level0_fraction': 0, 'level1_fraction': 0, 'level2_fraction': 0, 'level3_fraction': 1.0}),
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


# run EMA-Workbench experiment
if __name__ == '__main__':

    # experiment name and number
    experiment_string = 'policy_runs'
    experiment_number = 1
    experiment_name = f'{experiment_string}_{experiment_number}'

    # set seed
    seed = 0

    # show start of programme
    print(f'{datetime.now().time().strftime("%H:%M:%S")}: '
          f'Simulation of the {experiment_name} experiment has started:\n')

    # create runnable OpenTrafficSim model
    ots_model = VehicleAutomationModel(experiment_name=experiment_name, seed=seed, user_feedback=True)

    # compile OTS project
    ots_model.compile_project()

    # define model for EMA-Workbench
    ema_model = Model('VehicleAutomationModel', function=ots_model.run_model)

    # get defined model policies
    policies = define_policies()

    # define model constants
    ema_model.constants = [Constant('sim_time', 360)]  # 28800 8 hours of traffic simulation, 1800 is 30 min

    # define levers outside of policies
    ema_model.levers = []

    # combine these levers with the defined policies
    if len(ema_model.levers) > 0:
        lhs = LHSSampler()
        samples_per_parameter = 10  # 100 samples with LHS
        lhs_samples_dict = lhs.generate_samples(parameters=ema_model.uncertainties, size=samples_per_parameter)
        policies = add_levers_to_policies(policies, lhs_samples_dict, samples_per_parameter)

    # define uncertain variables
    ema_model.uncertainties = [RealParameter('main_demand', 1000, 6000),
                               RealParameter('ramp_demand', 800, 1000)]

    # list outcome variables of interest
    ema_model.outcomes = [ScalarOutcome('meanDensity'),
                          ScalarOutcome('meanFlow'),
                          ScalarOutcome('meanSpeed'),
                          ScalarOutcome('meanTravelTime'),
                          ScalarOutcome('laneChangesToRightBeforeRamp'),
                          ScalarOutcome('laneChangesToRightOnRamp'),
                          ScalarOutcome('laneChangesToRightAfterRamp'),
                          ScalarOutcome('laneChangesToLeftBeforeRamp'),
                          ScalarOutcome('laneChangesToLeftOnRamp'),
                          ScalarOutcome('laneChangesToLeftAfterRamp'),
                          ScalarOutcome('meanHeadwayTime'),
                          ScalarOutcome('meanHeadwayDistance')]

    # run experiments
    # MultiprocessingEvaluator will run multiple simulations at the same time, but laptop memory cannot handle this
    with SequentialEvaluator(ema_model) as evaluator:
        num_scenarios = 4
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
    results_name = f'{experiment_string}_results_v{experiment_number}'
    results_dir = fr'results'
    results_path = os.path.join(results_dir, f'{results_name}.csv')
    df_results.to_csv(results_path)

    # show successful save of results
    print(f'\nThe experiment results are successfully saved to:')
    print(f'{results_path}')

    # show end of programme
    print('\nExperiment has finished.')
