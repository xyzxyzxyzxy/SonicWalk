import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.style as mplstyle
import numpy as np

class Plotter():
    def terminate(self):
        print('plotter daemon terminated...')
        self.ani.event_source.stop()
        plt.close(self.fig)
    
    def animate(self, i):
        if self.data0[self.index0.value] == 1000:
            self.terminate()
            return

        pitch0 = np.array(self.data0)
        pitch1 = np.array(self.data1)

        self.ax.clear()
        l0, = self.ax.plot(self.data0, 'b')
        l1, = self.ax.plot(self.data1, 'c')
        return l0, l1
    
    def __call__(self, data0, data1, index0, index1):
        print('starting plotter daemon..')
        mplstyle.use('fast')
        self.data0 = data0
        self.data1 = data1
        self.index0 = index0 
        self.index1 = index1
        self.fig, self.ax = plt.subplots()
        self.ani = animation.FuncAnimation(self.fig, self.animate, interval=100, cache_frame_data=False, blit=True)
        print('...plotter daemon started')
        plt.show()