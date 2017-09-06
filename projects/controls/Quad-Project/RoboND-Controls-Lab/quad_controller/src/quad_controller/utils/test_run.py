import matplotlib.pyplot as plt

class TestRun:
    def __init__(self):
        self.sample_values_ = []
        self.sample_times_ = []

    def addSample(self, sample_value, timestamp):
        self.sample_times_.append(timestamp)
        self.sample_values_.append(sample_value)
        return True

    def reset(self):
        self.sample_values_ = []
        self.sample_times_ = []

    def plotRun(self, titles, axis_labels):
        fig = plt.figure(figsize=(12,3))
        plt.subplot(111)
        plt.title(titles[0])
        plt.xlabel(axis_labels[0])
        plt.ylabel(axis_labels[1])
        plt.plot(self.sample_times_, self.sample_values_)
        plt.show()

 
if __name__ == '__main__':
    # Generate a bunch of fake samples and times
    import random
    run = TestRun()
    for i,j in enumerate(range(0,100)):
        run.addSample(j+random.uniform(0.0, 5.0), i)
    
    run.plotRun(['Silly Plot of Nothing'], ['Time', 'Random Signal'])