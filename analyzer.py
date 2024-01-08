import simpleaudio as sa
import time
import numpy as np

class Analyzer():
    def __init__(self) -> None:
        self.wave = sa.WaveObject.from_wave_file("audio_samples/beep.wav")
        self.winsize = 15 #window duration is 8.33ms * 15 ~ 125 ms
        self.peak = 0.0
        self.threshold = 3.5 #MAKE IT ADAPTIVE
        self.timeThreshold = 0.1 #seconds (100 ms)
        self.swingPhase = False
        self.active = False
        self.steps = 0
        
    def terminate(self):
        print("analyzer daemon {:d} terminated...".format(self.num))
        print("analyzer {:d} number of steps: {:d}".format(self.num, self.steps))
        #write total number of steps to shared memory
        self.data[self.index.value - 1] = self.steps
    
    def __detectStep(self):
        if self.data[self.index.value] == 1000:
            self.active = False
            self.terminate()
            return
        
        #Take last winsize samples from shared memory buffer
        if self.index.value > self.winsize:
            self.pitch = np.array(self.data[self.index.value-self.winsize:self.index.value])
        else:
            self.pitch = np.concatenate((np.array(self.data[-(self.winsize - self.index.value):]), np.array(self.data[:self.index.value])))

        # zero crossings count
        cross =  np.diff(np.signbit(self.pitch))
        if np.sum(cross) == 1: #If more than 1 zero crossing is found then it's noise
            crossPosition = np.where(cross)[0][0]
            # DETERMINE POLARITY OF ZC AFTER FINDING IT (use np.gradient at index of zero crossing + 1 (the value where zero is crossed))
            negativeZc = np.signbit(np.gradient(self.pitch)[crossPosition + 1])
            if negativeZc:
                if self.swingPhase == True:
                    elapsed_time = time.time() - self.timestamp
                    #print(elapsed_time)
                    if self.peak >= self.threshold and elapsed_time > self.timeThreshold:
                        self.swingPhase = False
                        self.timestamp = time.time()
                        playObject = self.wave.play()
                        self.steps += 1
                        self.peak = 0.0
                        print(self.stg) 
            else:
                self.swingPhase = True
                self.peak = np.max([self.peak, np.max(self.pitch)])

    def stepDetector(self):
        timestamp = time.time()
        while self.active:
            self.__detectStep()
        

    def __call__(self, data, index, num):
        print('starting analyzer daemon.. {:d}'.format(num))

        if num == 0:
            self.stg = "********************************STEP**********ZERO*********************************************"
        else:
            self.stg = "********************************STEP***********ONE*********************************************"

        self.num = num
        self.data = data
        self.index = index
        self.timestamp = time.time()
        self.active = True

        print('...analyzer daemon {:d} started'.format(num))
        self.stepDetector()
        
        