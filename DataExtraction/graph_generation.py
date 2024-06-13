import numpy
import matplotlib.pyplot as plt
import pandas as pd
import scipy

def show_graph(title, data: list, labels: list, ppg_filter_passthrough=False):
    # by just using plt, it now comes with auto zoom features which I somehow missed.


    # split the data into two parts
    # plot the data
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)



    for data_element in range(len(data)):
        row = len(data[data_element])
        real_x_data = numpy.arange(row)
        real_y_data = data[data_element]
        if ppg_filter_passthrough:
            Fs = 25  # sampling rate of PPG
            b = scipy.signal.firls(numtaps=33, bands=numpy.array([0, 0.2, 0.5, 2.5, 2.8, Fs / 2])
                                   , desired=numpy.array([0, 0, 1, 1, 0, 0]),
                                   weight=numpy.array([2000, 100, 1000]),
                                   fs=Fs)  # fit a filter
            real_y_data = scipy.signal.filtfilt(b, 1, real_y_data, axis=-1, padtype=None)
        ax.plot(real_x_data, real_y_data, label=labels[data_element])

    ax.legend()
    # set the limits
    # ax.set_xlim([0, 1])
    # ax.set_ylim([-1000, 1000])

    ax.set_title(title)

    # display the plot

    # this may cause issues because we are supposed to shutdown this process after data
    # collection is done, which will shutdown this graph even if block=False.
    plt.show(block=True)



def pd_graph_generation(title:str, data_set:pd.DataFrame):
    #data_set = pd.read_csv()
    labels = data_set.columns.values.tolist()
    data = []
    for label in labels:
        data.append(list(data_set[label]))
    #for label in labels:
    #    signals.append(list(data_set[label]))
    show_graph(title, data, labels, False)


if __name__ == "__main__":
    data = pd.read_csv("H:/ppg_signal/for_MATLAB_tool/BILATERAL_reading_564_on_2024-06-02.csv", delimiter="\t", header=1)

    ppg = data["PPG"]
    left_ppg = data.loc[data["Laterality"] == "LEFT_ARM"]["PPG"]
    right_ppg = data.loc[data["Laterality"] == "RIGHT_ARM"]["PPG"].reset_index(drop=True)
    left_time = data.loc[data["Laterality"] == "LEFT_ARM"]["Time"]
    right_time = data.loc[data["Laterality"] == "RIGHT_ARM"]["Time"].reset_index(drop=True)
    ppg_alone = {"Left PPG": left_ppg, "Left Time":left_time, "Right PPG": right_ppg, "Right Time":right_time }
    processed_df = pd.concat(ppg_alone, axis=1)
    pd_graph_generation("ppg", processed_df)
    print("done")
