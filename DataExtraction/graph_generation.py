import numpy
import matplotlib.pyplot as plt
import scipy

def show_graph(title, data: list, labels: list, ppg_filter_passthrough=False):
    # by just using plt, it now comes with auto zoom features which I somehow missed.

    # create random data
    xdata = numpy.random.random([2, 10])

    # split the data into two parts

    # plot the data
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)

    # ax.plot(xdata1, ydata1, color='tab:blue')
    # ax.plot(xdata2, ydata2, color='tab:orange')


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

