from java_manager import JavaManager
from ema_workbench import RealParameter, ScalarOutcome, Model, SequentialEvaluator, perform_experiments
import pandas as pd
import os
import matplotlib.pyplot as plt
import seaborn as sns


# OpenTrafficSim model class
class OpenTrafficSimModel:

    # initialize this object
    def __init__(self, java_file_path, java_project_folder, output_folder, output_file_name):
        self.java_file_path = java_file_path
        self.java_project_folder = java_project_folder
        self.output_folder = output_folder
        self.output_file_name = output_file_name

        self.java_file = None

    # function to compile project
    def compile_project(self, classpath_file_path):
        # create Java object
        self.java_file = JavaManager(self.java_file_path, self.java_project_folder, None)
        # compile project
        self.java_file.compile(classpath_file_path)

    # function to run simulation
    def run_simulation(self, seed, sim_time, av_fraction):
        # run simulation headless. With animation is still subject to errors, why?
        run_headless = 'true'

        # set input parameters
        input_parameters = [
            f'-headless={run_headless}',
            f'-seed={seed}',
            f'-simTime={sim_time}',
            f'-avFraction={av_fraction}'
        ]

        # compile and run Java file
        self.java_file.java_parameters = input_parameters
        self.java_file.run_java_file()

    @staticmethod
    def load_simulation_output(output_folder_path, output_file_name):
        # load output data
        df_output = pd.read_csv(os.path.join(output_folder_path, output_file_name))

        # create dictionary for output variables
        output_dict = {}

        # process all variables
        for col in df_output.columns.tolist():
            # get values for this variable
            values = df_output[col].tolist()
            # convert list to single value when only one value is present
            if len(values) == 1:
                values = values[0]
            # add to dictionary
            output_dict[col] = values

        # return output in dictionary format
        return output_dict

    def run_model(self, sim_time, av_fraction):
        # set constant variables
        seed = 0

        # run model
        self.run_simulation(seed, sim_time, av_fraction)

        # retrieve output data
        output = self.load_simulation_output(self.output_folder, self.output_file_name)

        # return output data
        return output


# run EMA-Workbench experiments
if __name__ == '__main__':
    # set Java file path
    file_path = r'C:\Users\jesse\Documents\Java\TrafficSimulation-workspace' \
                r'\traffic-sim\src\main\java\sim\demo\RunMyShortMerge.java'
    # set Java folder path (for compilation of required Java classes)
    project_folder = r'C:\Users\jesse\Documents\Java\TrafficSimulation-workspace\traffic-sim\src\main\java\sim\demo'
    # output data folder
    data_folder = r'C:\Users\jesse\Documents\Java\TrafficSimulation-workspace\traffic-sim\src\main\resources'
    # output file name
    output_file = r'outputData.csv'
    # classpath file
    class_path_file = r'C:\Users\jesse\Documents\EPA_TUDelft\MasterThesis\thesis_experiments\test\classpath.txt'

    # create runnable OpenTrafficSim model
    ots_model = OpenTrafficSimModel(file_path, project_folder, data_folder, output_file)

    # compile OTS project
    ots_model.compile_project(class_path_file)

    # define model for EMA-Workbench
    ema_model = Model('OpenTrafficSim', function=ots_model.run_model)

    # define uncertain variables
    ema_model.uncertainties = [RealParameter('sim_time', 120, 240),
                               RealParameter('av_fraction', 0.1, 0.3)]

    # list outcome variables of interest
    ema_model.outcomes = [ScalarOutcome('LaneBasedMoveEvent')]

    # run experiments
    with SequentialEvaluator(ema_model) as evaluator:
        scenarios = 5
        results = perform_experiments(ema_model, scenarios=scenarios)

    # unpack results
    experiments, outcomes = results

    # convert outcomes to a DataFrame
    df_outcome = pd.DataFrame.from_dict(outcomes)

    # combine experiments and outcomes for analysis
    df_results: pd.DataFrame = experiments.join(df_outcome)

    # save DataFrame
    df_results.to_csv(os.path.join(r'C:\Users\jesse\Documents\EPA_TUDelft\MasterThesis\thesis_experiments\test',
                                   r'ema_results.csv'))

    # Plot the results to visualize relationships
    sns.pairplot(df_results, vars=['sim_time', 'av_fraction', 'LaneBasedMoveEvent'])
    plt.show()
