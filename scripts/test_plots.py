import rosbag
import numpy as np
import matplotlib.pyplot as pl
import os

from stat import S_ISREG, ST_CTIME, ST_MODE

class TestPlots():
    def __init__(self, bagfull, bagname, plotpath):
        self.error_time = np.empty([0,1])
        self.errors = np.empty([0,1])
        self.train_time = np.empty([0,1])
        self.njit = []
        self.wgen = []
        self.num_inducing = []
        self.degrees = []
        self.filename = bagfull
        self.bagname = bagname
        self.save_path = plotpath
        self.results = []
        self.mse = []
        self.rmse = []

    def extract_bag(self):
        bag = rosbag.Bag(self.filename)
        for msg in bag.read_messages(topics=['results']):
            self.results = msg.message
        bag.close()
        self.get_errors()
        self.get_params()
        self.get_updates()
        self.mse = np.mean(self.errors)
        self.rmse = np.sqrt(self.mse)

        
    def get_errors(self):
        for error in self.results.errors:
            self.errors = np.vstack((self.errors, error.error))
            self.error_time = np.vstack((self.error_time, error.header.stamp.to_sec()))

    def get_params(self):
        self.njit = self.results.params.njit
        self.num_inducing = self.results.params.inducing
        self.degrees = self.results.params.degrees
        self.wgen = self.results.params.wgen

    def get_updates(self):
        for upd in self.results.updates:
            self.train_time = np.vstack((self.train_time, upd.stamp.to_sec()))

    def make_plot(self):

        # shift = np.min([self.error_time[0],self.train_time[0]])
        shift = self.error_time[0]
        pl.plot(self.error_time - shift, self.errors)
        pl.vlines(self.train_time - shift, 0, np.max(self.errors), colors = 'g', linestyles = 'dotted' )
        pl.ylim(0, np.max(self.errors))
        pl.xlabel('Time [s]')
        pl.ylabel('squared error')
        # pl.text(15, 0.0004, 'rmse: ' + str(round(self.wgen, 4)))

        textstr = '\n'.join((
            r'$\mathrm{inducing\_points}=%u$' % (self.num_inducing, ),
            r'$\mathrm{wgen}=%.3f$' % (self.wgen, ),
            r'$\mathrm{RMSE}=%.4f$' % (self.rmse, )))

        # these are matplotlib.patch.Patch properties
        props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)

        # place a text box in upper left in axes coords
        pl.text(15, np.max(self.errors)*0.75, textstr, fontsize=12,
                verticalalignment='top', bbox=props)

        # pl.show()
        pl.savefig(self.save_path + self.bagname[:-4])
        pl.cla()

# rootdir = os.getcwd()
# imdir = os.path.join(rootdir, 'Figures')
dir_path = '/home/bpwilcox/catkin_ws/src/SOLAR_GP-ROS/bags/tests/'
plot_path = '/home/bpwilcox/catkin_ws/src/SOLAR_GP-ROS/plots/'
# dir_path = imdir   
data = (os.path.join(dir_path, fn) for fn in os.listdir(dir_path))
data = ((os.stat(path), path) for path in data)    
data = ((stat[ST_CTIME], path)
        for stat, path in data if S_ISREG(stat[ST_MODE]))

Results = dict{}
for cdate, path in sorted(data):       
    file = os.path.join(dir_path,path)
    split_path = os.path.split(path)
    # T = TestPlots(directory + 'test_20190314-212323.bag')
    T = TestPlots(file, split_path[1], plot_path)
    T.extract_bag()
    T.make_plot()
    Results[split_path[1]] = T.rmse