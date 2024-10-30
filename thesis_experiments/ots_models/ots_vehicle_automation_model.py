from ots_models.java_manager import JavaManager
import pandas as pd
import os
import numpy as np
import zipfile
import io


# OpenTrafficSim VehicleAutomationModel class.
# Class is designed to have a run_model function to work with the EMA Workbench.
# This allows the OTS Java model to run within the EMA Workbench package for extensive model exploration and analysis.
class VehicleAutomationModel:

    # initialize this object
    def __init__(self, experiment_name, seed=None, user_feedback=True):
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
        self.java_project_folder = r'C:\Users\jesse\Documents\Java\TrafficSimulation-new-175-workspace' \
                                   r'\traffic-sim\src\main\java\sim\demo'

        # output data folder
        self.output_parent_folder = r'C:\Users\jesse\Documents\Java\TrafficSimulation-new-175-workspace' \
                                    r'\traffic-sim\src\main\resources'
        self.output_folder = None
        self.generate_output_folder()

        # output file for EMA Workbench experiments
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
        output_folder = os.path.join(self.output_parent_folder,
                                     f'{self.experiment_name}',
                                     f'seed_{self.seed}',
                                     f'run_{self.replication_count}')
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
    def run_simulation(self, warm_up_time, sample_time,
                       level0_fraction, level1_fraction, level2_fraction, level3_fraction,
                       main_demand, ramp_demand, in_vehicle_distraction, road_side_distraction,
                       sensitivity_analysis_value):
        # only run simulation when seed is known
        if self.seed is not None:
            # get output folder for this run
            output_path = self.output_folder

            # run simulation headless (headless = 'true') or with animation window (headless = 'false')
            run_headless = 'true'

            # get current seed
            seed = self.seed

            print(f'\nInput: demand={main_demand} and ramp_demand={ramp_demand}')
            print(f'       level0={level0_fraction} level1={level1_fraction}, '
                  f'level2={level2_fraction} , level3={level3_fraction}')

            # set input parameters
            input_parameters = [
                f'-headless={run_headless}',
                f'-seed={seed}',
                f'-warmUpTime={warm_up_time}',
                f'-sampleTime={sample_time}',
                f'-level0Fraction={level0_fraction}',
                f'-level1Fraction={level1_fraction}',
                f'-level2Fraction={level2_fraction}',
                f'-level3Fraction={level3_fraction}',
                f'-mainDemand={main_demand}',
                f'-rampDemand={ramp_demand}',
                f'-inVehicleDistraction={in_vehicle_distraction}',
                f'-roadSideDistraction={road_side_distraction}',
                f'-sensitivityAnalysisValue={sensitivity_analysis_value}',

                rf'-outputFolderPath={output_path}',
                rf'-inputValuesFileName=inputValues.csv',
                rf'-singleOutputFileName=singleOutputData.csv',
                rf'-intermediateMeanValuesFileName=intermediateOutputData.csv',
                rf'-sequenceOutputFileName=sequenceOutputData.csv',
                rf'-laneChangeOutputFileName=laneChangeOutputData.csv'
            ]

            # compile and run Java file
            self.java_file.java_parameters = input_parameters
            self.java_file.run_java_file()

            # return corresponding output folder and file name
            return output_path

        # seed not provided? Provide user feedback
        else:
            print("Seed is None, so simulation cannot run.")

    # function to load simulation output
    def load_simulation_output(self, output_folder_path, output_file_name):
        # get zipped folder (remove .csv from output file name, it is saved as a .zip file)
        zip_path = os.path.join(output_folder_path, fr'{output_file_name[:-4]}.zip')
        # open zip file with output data
        with zipfile.ZipFile(zip_path, 'r') as zip_ref:
            # load single (non-sequential) output data
            with zip_ref.open(output_file_name) as data_file:
                df_output = pd.read_csv(data_file)

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
        warm_up_time = kwargs['warm_up_time']
        sample_time = kwargs['sample_time']
        level0_fraction = kwargs['level0_fraction']
        level1_fraction = kwargs['level1_fraction']
        level2_fraction = kwargs['level2_fraction']
        level3_fraction = kwargs['level3_fraction']
        main_demand = kwargs['main_demand']
        ramp_demand = kwargs['ramp_demand']
        in_vehicle_distraction = kwargs['in_vehicle_distraction']
        road_side_distraction = kwargs['road_side_distraction']
        sensitivity_analysis_value = kwargs['sensitivity_analysis_value']

        # run model (also receive fixed corresponding output path,
        #            self. reference could be changed by parallel running simulations)
        this_output_folder = self.run_simulation(warm_up_time, sample_time, level0_fraction, level1_fraction,
                                                 level2_fraction, level3_fraction,
                                                 main_demand, ramp_demand,
                                                 in_vehicle_distraction, road_side_distraction,
                                                 sensitivity_analysis_value)

        # retrieve output data
        output = self.load_simulation_output(this_output_folder, self.output_file_name)

        # return output data
        return output
