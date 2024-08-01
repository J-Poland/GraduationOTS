from ema_workbench import Model, MultiprocessingEvaluator, perform_experiments, Constant, RealParameter, ScalarOutcome
import pandas as pd
import os
import matplotlib.pyplot as plt
import seaborn as sns
from ots_models.ots_vehicle_automation_model import VehicleAutomationModel


# run EMA-Workbench experiment
if __name__ == '__main__':

    # experiment name and number
    experiment_string = 'hdv_av'
    experiment_number = 1
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
    ema_model = Model('MyShortMergeModel', function=ots_model.run_model)

    # define model constants
    ema_model.constants = [Constant('sim_time', 240)]

    # define uncertain variables
    ema_model.uncertainties = [RealParameter('main_demand', 1000, 3000),
                               RealParameter('ramp_demand', 500, 1000)]

    # set levers
    ema_model.levers = [RealParameter('av_fraction', 0.1, 0.3)]

    # list outcome variables of interest
    ema_model.outcomes = [ScalarOutcome('LaneBasedMoveEvent')]

    # run experiments
    with MultiprocessingEvaluator(ema_model) as evaluator:
        n_scenarios = 5
        n_policies = 4
        results = evaluator.perform_experiments(scenarios=n_scenarios, policies=n_policies)

    # unpack results
    experiments, outcomes = results

    # convert outcomes to a DataFrame
    df_outcome = pd.DataFrame.from_dict(outcomes)

    # combine experiments and outcomes for analysis
    df_results: pd.DataFrame = experiments.join(df_outcome)

    # save DataFrame
    results_name = f'{experiment_string}_results_v{experiment_number}'
    results_path = os.path.join(r'/',
                                fr'experiment_hdv_av\results\{results_name}.csv')
    df_results.to_csv(results_path)

    # show successful save of results
    print(f'\nThe experiment results are successfully saved to:')
    print(f'{results_path}')

    # Plot the results to visualize relationships
    sns.pairplot(df_results, vars=['av_fraction', 'main_demand', 'ramp_demand', 'LaneBasedMoveEvent'])
    plt.show()

    # show end of programme
    print('\nExperiment has finished.')
