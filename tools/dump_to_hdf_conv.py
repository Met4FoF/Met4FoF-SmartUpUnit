import json
import h5py
import csv
import numpy as np
import pandas
from collections import Counter
import itertools

def convertdumpfile(dumpfile):
    with open(dumpfile) as f:
        jsonstring = f.readline()
        dscp=json.loads(jsonstring)
    print(dscp)
    df=pandas.read_csv(dumpfile,sep=';',header=1)
    hdf5filename=dumpfile.replace('csv','hdf5')
    #hdffile = h5py.File(hdf5filename, 'a')
    #group=hdffile.create_group(hex(dscp[ID]))
    return dscp,df



if __name__ == "__main__":
    dscp,df=convertdumpfile(r"D:\data\200907_mpu9250_BMA280_cal\2020-09-01 Messungen MPU9250_SN20_Zweikanalig\WDH1\20200901114132_MPU_9250_0x1fe40000_platine_sensor_2_kanal_SN20_WDH1.dump")
    f = h5py.File(r"fsizeTest2.hdf5",'a')
    group=group=f.create_group(dscp['Name'].replace(' ','_')+hex(dscp['ID']))
    Acc=group.create_dataset("Acceleration",([3,np.max(df.shape)]),maxshape=(3,None),compression="gzip",shuffle=True,
                          dtype='float32')
    Gyro=group.create_dataset("Angular velocity", ([3,np.max(df.shape)]), maxshape=(3, None), compression="gzip",
                         shuffle=True,dtype='float32')
    Mag=group.create_dataset("Magnetic flux density", ([3,np.max(df.shape)]), maxshape=(3, None), compression="gzip",
                         shuffle=True,dtype='float32')
    Temp=group.create_dataset("Temperature", ([1, np.max(df.shape)]), maxshape=(3, None), compression="gzip",
                         shuffle=True,dtype='float32')
    Time=group.create_dataset("Time", ([1, np.max(df.shape)]), maxshape=(3, None), compression="gzip",
                         shuffle=True,dtype='i8')
    Acc[0]= df['Data_01']
    Acc[1] = df['Data_02']
    Acc[2] = df['Data_03']

    Gyro[0]= df['Data_04']
    Gyro[1] = df['Data_05']
    Gyro[2] = df['Data_06']

    Mag[0]= df['Data_07']
    Mag[1] = df['Data_08']
    Mag[2] = df['Data_09']

    Temp = df['Data_10']
    time=np.int64(df['unix_time']*1e9+df['unix_time_nsecs'])
    f.flush()

    #for the atributes and group selection we find channels with same unit und correleatet channel names

    strs = [str(x) for x in range(16)]# maximum 16 channels ;convert to str since json parsed keys are string
    aktivechannels=list(set(dscp.keys()).intersection(strs))#find matching keys
    aktivechannels.sort(key=float)
    units={}
    for channel in aktivechannels:
        if dscp[channel]["UNIT"] not in units.keys():
            units[dscp[channel]["UNIT"]]=[channel]
        else:
            units[dscp[channel]["UNIT"]].append(channel)
    for unit in units:
        unitnamelist = []
        if len(units[unit])>1:
            for channel in units[unit]:
                unitnamelist.append(dscp[channel]['PHYSICAL_QUANTITY'].split(' '))
            unitnamelist = list(itertools.chain(*unitnamelist))
            c = Counter(unitnamelist)
            quantname=Counter(el for el in c.elements() if c[el] >= len(units[unit])-1)
            most_common_words = [word for word, word_count in quantname.most_common()]
            basephysicalquant = " ".join(most_common_words)
            print(basephysicalquant)
        else:
            basephysicalquant = dscp[channel]['PHYSICAL_QUANTITY']
            print(basephysicalquant)