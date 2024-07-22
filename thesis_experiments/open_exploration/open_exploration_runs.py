from ema_workbench import Model, MultiprocessingEvaluator, perform_experiments, Constant, RealParameter, ScalarOutcome, SequentialEvaluator
from ema_workbench.em_framework.samplers import LHSSampler as lh_sampler
import pandas as pd
import os
import matplotlib.pyplot as plt
import seaborn as sns
from ots_models.ots_vehicle_automation_model import VehicleAutomationModel


# run EMA-Workbench experiment
if __name__ == '__main__':

    # experiment name and number
    experiment_string = 'open_exploration'
    experiment_number = 4
    experiment_name = f'{experiment_string}_{experiment_number}'

    # set seed
    seed = 0

    # show start of programme
    print(f'Simulation of the {experiment_name} experiment has started:\n')

    # create runnable OpenTrafficSim model
    ots_model = VehicleAutomationModel(experiment_name=experiment_name, seed=seed, user_feedback=True)

    # compile OTS project
    ots_model.compile_project()

    # define model for EMA-Workbench
    ema_model = Model('VehicleAutomationModel', function=ots_model.run_model)

    # define model constants
    ema_model.constants = [Constant('sim_time', 240)]  # 28800 8 hours of traffic simulation, 1800 is 30 min

    # define uncertain variables
    ema_model.uncertainties = [RealParameter('main_demand', 1000, 6000),
                               RealParameter('ramp_demand', 800, 1000)]

    # set levers
    ema_model.levers = [RealParameter('av_fraction', 0.0, 1.0)]

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
        n_scenarios = 30
        n_policies = 4
        results = evaluator.perform_experiments(scenarios=n_scenarios, policies=n_policies)

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
