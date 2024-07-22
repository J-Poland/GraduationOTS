import os
import pandas as pd
import matplotlib.pyplot as plt
from ema_workbench.analysis import prim
from ema_workbench.util import ema_logging
ema_logging.log_to_stderr(ema_logging.INFO)

pd.set_option('display.max_rows', None)


if __name__ == '__main__':

    data_file = 'open_exploration_results_v1.csv'
    data_path = os.path.join('results', data_file)
    df_data = pd.read_csv(data_path)

    print(df_data.columns.tolist())

    # convert units
    df_data['meanSpeed'] = df_data['meanSpeed'] * 3.6       # m/s to km/h
    df_data['meanFlow'] = df_data['meanFlow'] * 3600        # veh/s to veh/h
    df_data['meanDensity'] = df_data['meanDensity'] * 1000  # veh/m to veh/km

    x = df_data[['main_demand', 'ramp_demand']].copy()
    y = df_data['meanFlow'] > 1500

    # print all values
    for i in range(len(y)):
        print(f'{y.iloc[i]} -> {df_data["meanFlow"].iloc[i]}')

    prim_alg = prim.Prim(x, y, threshold=0.8, peel_alpha=0.1)
    box1 = prim_alg.find_box()

    box1.show_tradeoff()
    plt.show()

    box1.inspect(12)
    box1.inspect(12, style="graph")
    plt.show()

    box1.show_pairs_scatter(12)
    plt.show()


