import numpy as np
from nptdms import TdmsFile
import SineTools as ST
import json
REFtdms_file=TdmsFile.open(r"D:\data\191218_MPU_9250_X_Achse_150_Wdh\PXI-5922{0}.tdms")
SYNCtdms_file=TdmsFile.open(r"D:\data\191218_MPU_9250_X_Achse_150_Wdh\PXI-5922{1}.tdms")


REFgroup = REFtdms_file['18.12.2019 13:49:21 - All Data']
SYNCgroup = SYNCtdms_file['18.12.2019 13:49:21 - All Data']

REFchannel=REFgroup['PXI-5922{0}']
SYNCchannel=SYNCgroup['PXI-5922{1}']

treshold=0.4
ChunksInRow=5
SubChunkingFactor = 20
class Buffer:
    def __init__(self,DeltaT):
        self.DeltaT=DeltaT
        self.Samplerate=1/DeltaT
        self.Data=np.array([])
        self.Flags={'FFTDone':False}
        self.Params={}
    def Flush(self):
        self.Data = np.array([])
        self.RFFT=np.array([])
        self.RFFTFreqs=np.array([])
        self.Params = {}
        self.Flags = {'FFTDone': False}

    def PushData(self,Data):
        self.Data=np.append(self.Data, Data)


    def GetFFTMaxFreq(self):
        self.RFFT=np.fft.rfft(self.Data)
        self.RFFTFreqs=np.fft.rfftfreq(self.Data.size,self.DeltaT)
        self.Params["FFTFreq"]=self.RFFTFreqs[np.argmax(abs(self.RFFT))]
        self.Params["FFTAMP"]=abs(self.RFFT[np.argmax(abs(self.RFFT))])
        self.Params["FFTPhase"]=np.angle(self.RFFT[np.argmax(abs(self.RFFT))])
        self.Flags['FFTDone']=True
        return(self.RFFTFreqs[np.argmax(abs(self.RFFT))])

    def Finish(self):
        cutoutlen=3*int(self.Samplerate)
        self.Data=self.Data[cutoutlen:-cutoutlen]

    def GetSinFit(self):
        if self.Flags['FFTDone'] == False:
            self.GetFFTMaxFreq()
        self.times=np.arange(self.Data.size)*self.DeltaT
        self.FourParamsSineResultRaw=ST.seq_fourparsinefit(self.Data,self.times,self.Params["FFTFreq"],tol=5.0e-5,nmax=5000,periods=5)
        Complex = self.FourParamsSineResultRaw[:, 1] + 1j * self.FourParamsSineResultRaw[:, 0]
        DC = self.FourParamsSineResultRaw[:, 2]
        Freq = self.FourParamsSineResultRaw[:, 3]
        Angles=np.unwrap(np.angle(Complex))
        self.Params['FitResult']={
            'Amplitude':np.mean(abs(Complex)),
            'DC':np.mean(DC),
            'Frequency':np.mean(Freq),
            'Phase':np.mean(Angles)}
        self.Params['FitResultErr']={
            'Amplitude':np.std(abs(Complex))*2,
            'DC':np.std(DC)*2,
            'Frequency':np.std(Freq)*2,
            'Phase':np.std(Angles)*2,
            'N':self.FourParamsSineResultRaw.shape[0]}
        print("Fitting Done")
        return 0

if __name__ == '__main__':
    REFFoundFFTFreqs = []
    SYNCFoundFFTFreqs = []
    REFFitResults = []
    SYNCFitResults = []
    BoolCompareArray = np.array((ChunksInRow * True))
    REFDataPoints = REFchannel._length
    SYNCDataPoints = SYNCchannel._length
    if REFDataPoints != SYNCDataPoints:
        print("WARNING CHANNEL LENGTH MISMATCH !!!")
    MeanChunkSize = REFchannel.properties['wf_samples']
    NumOfCuncks = int(np.floor(REFDataPoints / MeanChunkSize))
    SignalSTD = np.zeros(NumOfCuncks * SubChunkingFactor)
    REFBuf = Buffer(REFchannel.properties['wf_increment'])
    SYNCBuf = Buffer(SYNCchannel.properties['wf_increment'])
    LastChunckCountainsValide = False
    TruesInRowSoFar = 0
    for i in range(NumOfCuncks):
        REFchannel_chunk_data = REFchannel[i * MeanChunkSize:MeanChunkSize * (i + 1)]
        REFSubChunkedData = np.array_split(REFchannel_chunk_data, SubChunkingFactor)

        SYNCchannel_chunk_data = SYNCchannel[i * MeanChunkSize:MeanChunkSize * (i + 1)]
        SYNCSubChunkedData = np.array_split(SYNCchannel_chunk_data, SubChunkingFactor)
        if (i % 100 == 0):
            print(i / NumOfCuncks * 100)
        STD = np.std(REFSubChunkedData, axis=1)
        if (np.isnan(STD).any()):
            print("NAN FOUND at i=" + str(i))
        SignalSTD[i * SubChunkingFactor:(i + 1) * SubChunkingFactor] = STD
        i = i + 1
        STDBiggerThanTreshold = STD > treshold
        for j in range(STDBiggerThanTreshold.size):
            if STDBiggerThanTreshold[j] == True:
                TruesInRowSoFar = TruesInRowSoFar + 1
                if TruesInRowSoFar > ChunksInRow:
                    REFBuf.PushData(REFSubChunkedData[j])
                    SYNCBuf.PushData(SYNCSubChunkedData[j])
            else:
                if TruesInRowSoFar > ChunksInRow:
                    #todo add multi treading here
                    REFBuf.Finish()
                    SYNCBuf.Finish()
                    REFBuf.GetSinFit()
                    SYNCBuf.GetSinFit()
                    print(str(i) + "FFT MAX AT " + str(REFBuf.Params["FFTFreq"]) + ' ' + str(SYNCBuf.Params["FFTFreq"]))
                    REFFitResults.append(REFBuf.Params)
                    SYNCFitResults.append(SYNCBuf.Params)
                REFBuf.Flush()
                SYNCBuf.Flush()
                TruesInRowSoFar = 0
        del REFchannel_chunk_data
        del SYNCchannel_chunk_data

    T = np.zeros([len(SYNCFitResults),6])
    loop = 0
    lastFreq = 0
    for i in range(len(SYNCFitResults)):
        if REFFitResults[i]['FitResult']['Frequency'] < lastFreq:
            loop = loop + 1
        T[i,0]=loop
        T[i,2]=AMP = REFFitResults[i]['FitResult']['Amplitude']
        T[i,3]=AMPErr = REFFitResults[i]['FitResultErr']['Amplitude']
        T[i,4]=Phase = REFFitResults[i]['FitResult']['Phase'] - SYNCFitResults[i]['FitResult']['Phase']
        T[i,5]=PhaseErr = np.sqrt(np.power(REFFitResults[i]['FitResult']['Phase'], 2) + np.power(SYNCFitResults[i]['FitResult']['Phase'], 2))
        T[i,1]=intFreq = np.rint(REFFitResults[i]['FitResult']['Frequency'])
        lastFreq = REFFitResults[i]['FitResult']['Frequency']
