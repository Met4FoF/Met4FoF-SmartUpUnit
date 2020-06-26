import numpy as np
from nptdms import TdmsFile
import SineTools as ST

testFreqRelOffset=[]
def GetNearestFreq(freq,TestFreqs):
    frqIDX = abs(TestFreqs - freq).argmin()
    Freqoffset=freq/TestFreqs[frqIDX]
    testFreqRelOffset.append(Freqoffset)
    return TestFreqs[frqIDX]

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
        cutoutlenStart=int(2.5*self.Samplerate)
        cutoutlenStop =int(1 * self.Samplerate)
        self.Data=self.Data[cutoutlenStart:-cutoutlenStop]

    def GetSinFit(self):
        if self.Flags['FFTDone'] == False:
            self.GetFFTMaxFreq()
        self.times=np.arange(self.Data.size)*self.DeltaT
        tmpperiods=int(np.rint(self.Params["FFTFreq"]))
        self.Sine4ParamResultRaw=ST.fourparsinefit(self.Data, self.times, self.Params["FFTFreq"], tol=1.0e-7, nmax=8000)
        self.SineResultRaw = ST.seq_threeparsinefit(self.Data, self.times,self.Sine4ParamResultRaw[3],periods=tmpperiods)
        Complex = self.SineResultRaw[:, 1] + 1j * self.SineResultRaw[:, 0]
        DC = self.SineResultRaw[:, 2]
        Freq = self.Sine4ParamResultRaw[3]
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
            'N':self.SineResultRaw.shape[0]}
        self.Params['AngleRaw']=Angles
        print("Fitting Done")
        return 0

def generateTFCSV(folder,tdmsChannel,outputfile):
    REFtdms_file = TdmsFile.open(folder+r"\vibrometer.tdms")
    SYNCtdms_file = TdmsFile.open(folder+r"\SYNC.tdms")

    REFgroup = REFtdms_file[tdmsChannel]
    SYNCgroup = SYNCtdms_file[tdmsChannel]

    REFchannel = REFgroup['vibrometer']
    SYNCchannel = SYNCgroup['SYNC']

    treshold = 0.35
    ChunksInRow = 7
    SubChunkingFactor = 20
    SensDegPerSecPerVolt=100

    TestFreqs=[4.0,5.0,6.3,8.0,10.0,12.5,16.0,20.0,25.0,31.5,40.0,50.0,63.0,80.0,100.0,125.0,160.0,200.0,250.0]
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
    SignalValide = np.zeros(NumOfCuncks * SubChunkingFactor)
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
                    SignalValide[(i-1)*SubChunkingFactor+j]=1
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
        if REFFitResults[i]['FitResult']['Frequency']*1.05 < lastFreq:
            loop = loop + 1
            #todo add unwrap here
        T[i,0]=loop
        T[i,2]=AMP = REFFitResults[i]['FitResult']['Amplitude']
        T[i,3]=AMPErr = REFFitResults[i]['FitResultErr']['Amplitude']
        T[i,4]=Phase = (REFFitResults[i]['FitResult']['Phase'] - SYNCFitResults[i]['FitResult']['Phase'])/np.pi*180
        T[i,5]=PhaseErr = (np.sqrt(np.power(REFFitResults[i]['FitResultErr']['Phase'], 2) + np.power(SYNCFitResults[i]['FitResultErr']['Phase'], 2)))/np.pi*180
        #T[i,6]=PhaseErr = REFFitResults[i]['FitResultErr']['Phase']/np.pi*180
        #T[i,7] = PhaseErr = SYNCFitResults[i]['FitResultErr']['Phase']/np.pi*180
        #T[i, 8] = AMPTF= REFFitResults[i]['FitResult']['Amplitude']/SYNCFitResults[i]['FitResult']['Amplitude']
        T[i,1]=intFreq = GetNearestFreq(REFFitResults[i]['FitResult']['Frequency'],TestFreqs)
        lastFreq = REFFitResults[i]['FitResult']['Frequency']

    T[:, 2] = SensDegPerSecPerVolt * T[:, 2]
    T[:, 3] = SensDegPerSecPerVolt * T[:, 3]


    for i in range(len(SYNCFitResults)):
        if T[i,4]>180:
            T[i, 4]=T[i,4]-360
        if T[i,4]<-180:
            T[i, 4] = T[i, 4] + 360
    headerstr="""loop;frequency;ex_amp;ex_amp_std;phase;phase_std;
    #Experiment Count;Excitation frequency;Excitation amplitude;Excitation amplitude uncertainty (2s 95%);Phase against reference signal;Phase against reference signal uncertainty (2s 95%)
    #Number;Hz;deg/s;deg/s;deg;deg"""
    np.savetxt(folder+outputfile+r'_TDMS_TF.csv', T, delimiter=';', header=headerstr,comments='')







