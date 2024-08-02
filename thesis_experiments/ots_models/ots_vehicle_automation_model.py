from ots_models.java_manager import JavaManager
import pandas as pd
import os
import numpy as np


# OpenTrafficSim VehicleAutomationModel class.
# Class is designed to have a run_model function to work with the EMA Workbench.
# This allows the OTS Java model to run within the EMA Workbench package for extensive model exploration and analysis.
class VehicleAutomationModel:

    # initialize this object
    def __init__(self, experiment_name, seed=0, user_feedback=True):
        # input variables
        self.experiment_name = experiment_name
        self.seed = seed
        self.user_feedback = user_feedback

        # class variables
        self.java_file = None
        self.replication_count = None
        self.original_os_dir = None

        # set Java file path
        self.java_file_path = r'C:\Users\jesse\Documents\Java\TrafficSimulation-workspace' \
                              r'\traffic-sim\src\main\java\sim\demo\RunVehicleAutomationModel.java'

        # set Java folder path (for compilation of required Java classes)
        self.java_project_folder = r'C:\Users\jesse\Documents\Java\TrafficSimulation-workspace' \
                                   r'\traffic-sim\src\main\java\sim\demo'

        # set Java resources folder
        self.resources_folder = r'C:\Users\jesse\Documents\Java\TrafficSimulation-workspace' \
                                r'\traffic-sim\src\main\resources'

        # output data folder
        self.output_folder = None
        self.generate_output_folder()

        # output file name
        self.output_file_name = r'singleOutputData.csv'

        # classpath
        # get classpath from Maven project
        # run project as Maven project
        # set Goals field: dependency:build-classpath -DincludeScope=runtime
        # then run and paste resulting string into classpath.txt
        self.class_path_file = r'C:\Users\jesse\Documents\EPA_TUDelft\MasterThesis\thesis_experiments' \
                               r'\ots_models\classpath.txt'

    # function to create output folder path
    def generate_output_folder(self):
        output_folder = os.path.join(self.resources_folder, f'{self.experiment_name}',
                                     f'{self.experiment_name}_{self.replication_count}')
        self.output_folder = output_folder

    # function to compile project
    def compile_project(self):
        # create Java object
        self.java_file = JavaManager(java_file=self.java_file_path,
                                     compilation_folder=self.java_project_folder,
                                     java_parameters=None,
                                     user_feedback=self.user_feedback)
        # compile project
        self.java_file.compile(self.class_path_file)

        # get original os directory
        self.original_os_dir = self.java_file.original_os_dir

    # function to start the simulation's Java model
    def run_simulation(self, sim_time, level0_fraction, level1_fraction, level2_fraction, level3_fraction,
                       main_demand, ramp_demand):
        # get output folder for this run
        output_path = self.output_folder

        # run simulation headless (headless = 'true') or with animation window (headless = 'false')
        # TODO: with animation is still subject to errors, why?
        run_headless = 'true'

        # get current seed
        seed = self.seed

        # set input parameters
        input_parameters = [
            f'-headless={run_headless}',
            f'-seed={seed}',
            f'-simTime={sim_time}',
            f'-level0Fraction={level0_fraction}',
            f'-level1Fraction={level1_fraction}',
            f'-level2Fraction={level2_fraction}',
            f'-level3Fraction={level3_fraction}',
            f'-mainDemand={main_demand}',
            f'-rampDemand={ramp_demand}',
            rf'-singleOutputFilePath={output_path}\\singleOutputData.csv',
            rf'-intermediateMeanValuesFilePath={output_path}\\intermediateOutputData.csv',
            rf'-sequenceOutputFilePath={output_path}\\sequenceOutputData.csv',
            rf'-trajectoryOutputFilePath={output_path}\\trajectoryOutputData.csv'
        ]

        # compile and run Java file
        self.java_file.java_parameters = input_parameters
        self.java_file.run_java_file()

        # return corresponding output folder and file name
        return output_path

    # function to load simulation output
    def load_simulation_output(self, output_folder_path, output_file_name, detailed_output_file_name=None,
                               experiment_results_folder=None):
        # load single (non-sequential) output data
        df_output = pd.read_csv(os.path.join(output_folder_path, output_file_name))

        # only save detailed/sequential output when required paths are provided
        if (detailed_output_file_name is not None) and (experiment_results_folder is not None) and \
           (self.replication_count is not None):
            # load detailed output data
            df_detailed_output = pd.read_csv(os.path.join(output_folder_path, detailed_output_file_name))
            # save detailed output data
            detailed_csv_name = f'detailed_output_{self.experiment_name}_rep{self.replication_count}.csv'
            df_detailed_output.to_csv(os.path.join(experiment_results_folder, detailed_csv_name))

        # create dictionary for output variables
        output_dict = {}

        # process all variables
        for col in df_output.columns.tolist():
            # get values for this variable
            values = df_output[col].tolist()
            # convert list to single value when only one value is present
            if len(values) == 1:
                values = values[0]
            # check if single value is null, NaN or None
            invalid_values = [None, "None", "null", "Null", np.nan, "NaN", "nan", "NAN"]
            if values in invalid_values:
                values = 0
            # add to dictionary
            output_dict[col] = values

        # return output in dictionary format
        return output_dict

    # function to run the simulation model for the EMA Workbench and return model output
    def run_model(self, **kwargs):
        # count replications
        if self.replication_count is None:
            self.replication_count = 0
        else:
            self.replication_count += 1

        # create output folder for this replication
        self.generate_output_folder()

        # unpack experiment details
        sim_time = kwargs['sim_time']
        level0_fraction = kwargs['level0_fraction']
        level1_fraction = kwargs['level1_fraction']
        level2_fraction = kwargs['level2_fraction']
        level3_fraction = kwargs['level3_fraction']
        main_demand = kwargs['main_demand']
        ramp_demand = kwargs['ramp_demand']

        # run model (also receive fixed corresponding output path,
        #            self. reference could be changed by parallel running simulations)
        this_output_folder = self.run_simulation(sim_time, level0_fraction, level1_fraction,
                                                 level2_fraction, level3_fraction,
                                                 main_demand, ramp_demand)

        # retrieve output data
        output = self.load_simulation_output(this_output_folder, self.output_file_name)

        # return output data
        return output
