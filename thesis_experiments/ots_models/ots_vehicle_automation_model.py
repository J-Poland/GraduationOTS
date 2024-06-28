from ots_models.java_manager import JavaManager
import pandas as pd
import os


# OpenTrafficSim MyShortMerge model class
class MyShortMergeModel:

    # initialize this object
    def __init__(self, experiment_name, seed=0, user_feedback=True):
        # set Java file path
        self.java_file_path = r'C:\Users\jesse\Documents\Java\TrafficSimulation-workspace' \
                              r'\traffic-sim\src\main\java\sim\demo\RunMyShortMerge.java'

        # set Java folder path (for compilation of required Java classes)
        self.java_project_folder = r'C:\Users\jesse\Documents\Java\TrafficSimulation-workspace' \
                                   r'\traffic-sim\src\main\java\sim\demo'

        # output data folder
        self.output_folder = r'C:\Users\jesse\Documents\Java\TrafficSimulation-workspace' \
                             r'\traffic-sim\src\main\resources'

        # output file name
        self.output_file_name = r'singleOutputData.csv'

        # classpath
        self.class_path_file = r'C:\Users\jesse\Documents\EPA_TUDelft\MasterThesis\thesis_experiments' \
                               r'\ots_models\classpath.txt'

        # input variables
        self.experiment_name = experiment_name
        self.seed = seed
        self.user_feedback = user_feedback

        # class variables
        self.java_file = None
        self.replication_count = None

    # function to compile project
    def compile_project(self):
        # create Java object
        self.java_file = JavaManager(java_file=self.java_file_path,
                                     compilation_folder=self.java_project_folder,
                                     java_parameters=None,
                                     user_feedback=self.user_feedback)
        # compile project
        self.java_file.compile(self.class_path_file)

    # function to run simulation
    def run_simulation(self, sim_time, av_fraction, main_demand, ramp_demand):
        # run simulation headless. With animation is still subject to errors, why?
        run_headless = 'true'

        # get current seed
        seed = self.seed

        # set input parameters
        input_parameters = [
            f'-headless={run_headless}',
            f'-seed={seed}',
            f'-simTime={sim_time}',
            f'-avFraction={av_fraction}',
            f'-mainDemand={main_demand}',
            f'-rampDemand={ramp_demand}'
        ]

        # compile and run Java file
        self.java_file.java_parameters = input_parameters
        self.java_file.run_java_file()

    # function to automatically save detailed output data
    def save_detailed_output_data(self, results_folder):
        pass

    # function to load simulation output
    def load_simulation_output(self, output_folder_path, output_file_name, detailed_output_file_name=None,
                               experiment_results_folder=None):
        # load output data
        df_output = pd.read_csv(os.path.join(output_folder_path, output_file_name))

        # only save detailed output when required paths are provided
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
            # add to dictionary
            output_dict[col] = values

        # return output in dictionary format
        return output_dict

    # function to run the simulation model
    def run_model(self, **kwargs):

        # count replications
        if self.replication_count is None:
            self.replication_count = 0
        else:
            self.replication_count += 1

        # unpack experiment details
        sim_time = kwargs['sim_time']
        av_fraction = kwargs['av_fraction']
        main_demand = kwargs['main_demand']
        ramp_demand = kwargs['ramp_demand']

        # run model
        self.run_simulation(sim_time, av_fraction, main_demand, ramp_demand)

        # retrieve output data
        output = self.load_simulation_output(self.output_folder, self.output_file_name)

        # return output data
        return output
