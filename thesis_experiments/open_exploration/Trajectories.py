import pandas as pd
import os
import matplotlib.pyplot as plt


# class to manage trajectory data
class TrajectoryData:

    def __init__(self, trajectoriesFilePath):
        self.trajectoriesFilePath = trajectoriesFilePath
        self.df_trajectories = None

    def load_trajectory_data(self):
        # check file path
        if not os.path.exists(self.trajectoriesFilePath):
            print("Provided file path for trajectory data is not valid.")
            return

        # load data
        self.df_trajectories = pd.read_csv(self.trajectoriesFilePath)

    def show_trajectories(self, lanes=None):
        # get trajectory data
        df_traject = self.df_trajectories

        # select specific lane(s)
        lanes_string = ''
        if (lanes is not None) and (len(lanes) > 0):
            lanes_string = 'for lanes: '
            df_traject = df_traject[df_traject['lane'].isin(lanes)]
            for lane in lanes:
                lanes_string += f'{lane}, '
        lanes_string = lanes_string[:-2]

        # group by vehicle id
        df_grouped = df_traject.groupby('id')

        # create figure
        fig, ax = plt.subplots(1, 1, figsize=(16, 6))

        # plot for each id
        for veh_id, group in df_grouped:
            plt.plot(group['time'], group['distance'], linestyle='-', label=f'Vehicle {veh_id}', linewidth=0.8)

        # show plot
        ax.set_title(f'Vehicle trajectories {lanes_string}')
        ax.set(xlabel='Time', ylabel='Distance')

        plt.show()


if __name__ == '__main__':

    # trajectory data file path
    file_path = 'C:\\Users\\jesse\\Documents\\Java\\TrafficSimulation-workspace\\traffic-sim\\' \
                'src\\main\\resources\\trajectoryOutputData.csv'

    # create trajectory data
    trajectory_data = TrajectoryData(file_path)
    # load data
    trajectory_data.load_trajectory_data()
    # show data (possible lanes: FORWARD1, FORWARD2, FORWARD3)
    lanes = ['FORWARD1', 'FORWARD2']
    trajectory_data.show_trajectories(lanes=lanes)
