import numpy as np
from nptdms import TdmsFile
import time as time
import matplotlib.pyplot as plt
treshold=0.4
ChunksInRow=5
BoolCompareArray=np.array((ChunksInRow*True))
class Buffer:
    def __init__(self,DeltaT):
        self.DeltaT=DeltaT
        self.Samplerate=1/DeltaT
        self.Data=np.array([])
    def Flush(self):
        self.Data = np.array([])
        self.RFFT=np.array([])
        self.RFFTFreqs=np.array([])

    def PushData(self,Data):
        self.Data=np.append(self.Data, Data)

    def GetFFTMaxFreq(self):
        self.RFFT=np.fft.rfft(self.Data)
        self.RFFTFreqs=np.fft.rfftfreq(self.Data.size,self.DeltaT)
        return(self.RFFTFreqs[np.argmax(abs(self.RFFT))])

FoundFFTFreqs=[]
with TdmsFile.open(r"D:\data\191218_MPU_9250_X_Achse_150_Wdh\PXI-5922{0}.tdms") as tdms_file:
    group = tdms_file['18.12.2019 13:49:21 - All Data']
    channel=group['PXI-5922{0}']
    DataPoints=channel._length
    MeanChunkSize=channel.properties['wf_samples']
    NumOfCuncks=int(np.floor(DataPoints/MeanChunkSize))
    SubChunkingFactor=20
    SignalSTD=np.zeros(NumOfCuncks*SubChunkingFactor)
    Buf=Buffer(channel.properties['wf_increment'])
    LastChunckCountainsValide=False
    print("File opened")
    TruesInRowSoFar=0
    for i in range(NumOfCuncks):
        channel_chunk_data = channel[i*MeanChunkSize:MeanChunkSize*(i+1)]
        SubChunkedData=np.array_split(channel_chunk_data,SubChunkingFactor)
        if(i%100==0):
            print(i/NumOfCuncks*100)
        STD=np.std(SubChunkedData,axis=1)
        if(np.isnan(STD).any()):
            print("NAN FOUND at i="+str(i))
        SignalSTD[i*SubChunkingFactor:(i+1)*SubChunkingFactor]=STD
        i=i+1
        STDBiggerThanTreshold=STD > treshold
        for j in range(STDBiggerThanTreshold.size):
            if STDBiggerThanTreshold[j]==True:
                TruesInRowSoFar=TruesInRowSoFar+1
                if TruesInRowSoFar>ChunksInRow:
                    Buf.PushData(SubChunkedData[j])
            else:
                if TruesInRowSoFar>ChunksInRow:
                    print(i)
                    FFtFreq=Buf.GetFFTMaxFreq()
                    print(FFtFreq)
                    FoundFFTFreqs.append(FFtFreq)
                    if(FFtFreq>201):
                        plt.plot(Buf.Data)
                        plt.show()
                        stopIDX=int(1.2*np.argwhere(Buf.RFFTFreqs==FFtFreq))
                        plt.plot(Buf.RFFTFreqs[:stopIDX],abs(Buf.RFFT[:stopIDX]))
                        plt.show()
                        print("FFtFreq to large")
                    Buf.Flush()
                TruesInRowSoFar=0
        del channel_chunk_data
