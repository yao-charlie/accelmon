from datetime import datetime

class SampleSink:
    """The abstract base class for sinks to record streaming sample data"""

    def write(self, sample):
        """Write sample data to the sink.

        :param sample: The data sample as a iterable container (e.g. list)
        :type sample: list
        :returns: None
        """
        raise NotImplementedError("Abstract method")

class CsvSampleSink (SampleSink):
    """Write sample data to a text file in comma separated value (CSV) format"""

    def __init__(self, filename):
        self.filename = filename
        self.hf = None

    def open(self): 
        self.close()
        self.hf = open(self.filename, "w")

    def close(self):
        if self.hf is not None:
            self.hf.close()
        self.hf = None

    def write(self, sample):
        seps = [', ']*len(sample)
        seps[-1] = '\n'
        for v,s in zip(sample,seps):
            self.hf.write("{}{}".format(v,s).replace("[","").replace("]",""))

class ListSampleSink (SampleSink):
    """Write sample data to an array"""

    def __init__(self):
        self.data = []

    def write(self, sample):
        self.data.extend(sample)

        

