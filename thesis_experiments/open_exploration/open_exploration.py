import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np


# function to add a trend line
def add_trend_line(df_x, df_y, degree, line_type, name):
    # fit a polynomial trendline to data
    z = np.polyfit(df_x, df_y, degree)
    p = np.poly1d(z)
    # generate x and y values for the trend line
    x_fit = np.linspace(df_x.min(), df_x.max(), 100)
    y_fit = p(x_fit)
    # plot trend line
    ax.plot(x_fit, y_fit, line_type, label=f'{degree}-degree {name}')


if __name__ == '__main__':

    data_file = 'open_exploration_results_v4.csv'
    data_path = os.path.join('results', data_file)
    df_data = pd.read_csv(data_path)

    print(df_data.columns.tolist())

    # convert values for demand, flow, speed and travel time per distance to more convenient units
    df_data['meanSpeed'] = df_data['meanSpeed'] * 3.6       # m/s to km/h
    df_data['meanFlow'] = df_data['meanFlow'] * 3600        # veh/s to veh/h
    df_data['meanDensity'] = df_data['meanDensity'] * 1000      # veh/m to veh/km
    df_data['meanTravelTime'] = df_data['meanTravelTime'] * 60     # s to min

    # remove NaN and infinite values
    df_data = df_data.replace([np.inf, -np.inf], np.nan).dropna()

    # divide dataframe on meanDensity
    mean_density_value = df_data['meanDensity'].mean()
    df_data_till_mean = df_data[df_data['meanDensity'] <= mean_density_value]
    df_data_from_mean = df_data[df_data['meanDensity'] > mean_density_value]

    # x = df_data[['main_demand', 'ramp_demand']].copy()
    # y = df_data[['meanDensity', 'meanFlow']].copy()

    # plot scatter data
    fig, ax = plt.subplots(1, 1, figsize=(6, 6))
    ax.scatter(df_data['meanDensity'], df_data['meanFlow'], label='Data points')

    # add trend lines for first and second half of plot
    add_trend_line(df_x=df_data_till_mean['meanDensity'], df_y=df_data_till_mean['meanFlow'], degree=2,
                   line_type="g--", name='Free flow trend line')
    add_trend_line(df_x=df_data_from_mean['meanDensity'], df_y=df_data_from_mean['meanFlow'], degree=2,
                   line_type="r--", name='Congested trend line')

    # show plot
    ax.set_title('Density vs Flow')
    ax.set(xlabel='Density (veh/km)', ylabel='Flow (veh/h)')
    ax.legend()

    plt.show()

    fig, ax = plt.subplots(1, 1, figsize=(6, 6))
    ax.scatter(df_data['av_fraction'], df_data['meanHeadwayTime'])
    ax.set(xlabel='av_fraction', ylabel='meanHeadwayTime')
    ax.legend()
    plt.show()

