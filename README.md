# Software for the Met4FoF Smartup Unit V2

[See this publication as well](https://doi.org/10.1051/metrology/201922003)
## Setup
1. Download this repo
1. install Python dependencies [add needed deps]

    ```shell
    pip install protobuf
    ```
1. Flash the µC with the newest release [see this link] (doc/flash_stm32_fimware.md)
1. Connect µC and PC directly via Lan (or over an switch with DHCP anabled to 192.168.2.x Subnet)
1. In case you are working behind a firewall, set an exception for incoming UDP packets from the IP address of the Smartup Unit on port 7654.
1. Set ip Adress of pc to 192.168.0.200
1. Power on µC Board
1. Observe Display and LEDS see Picture
1. Start Python Interpreter and run /tools/MET4FOFDataReceiver.py
1. Start data Receiver node with:

    ```python
    DR=DataReceiver("192.168.0.200",7654)
    ```
If every thing works correctly you will see that a new sensor is found

```
Data receiver now running wating for packets 
FOUND NEW SENSOR WITH ID=hex0x37300000==>dec:925892608
```
Attach your callback function to Sensor
    
```python
DR.AllSensors[925892608].SetCallback(DumpDataMPU9250)
```
    
# Misc
### Build System.
The software was and is created with [SW4STM32](http://www.openstm32.org/HomePage) (gcc).
[SEGGER Systemview](https://www.segger.com/products/development-tools/systemview/) can be used for debugging.

## Dependencies and used libraries

| Used for                   | Library Name | Version    | Link                                                                                                                                                                                           |
|:---------------------------|:-------------|:-----------|:-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| real-time operating system | FreeRTOS     | 9.0.0      | https://www.freertos.org/FreeRTOS-V9.html                                                                                                                                                      |
| Ethernet                   | LWIP         | 2.0.3      | [ST DM00103685]( https://www.st.com/content/ccc/resource/technical/document/user_manual/65/e8/20/db/16/36/45/f7/DM00103685.pdf/files/DM00103685.pdf/jcr:content/translations/en.DM00103685.pdf)|
| Data serialization         | nanopb       | a2db482... | https://github.com/nanopb/nanopb                                                                                                                                                               |

## using nanopb to generate Code
### C
see also https://jpa.kapsi.fi/nanopb/docs/
and https://github.com/nanopb/nanopb/blob/master/docs/concepts.rst

![nanopb_picture](https://jpa.kapsi.fi/nanopb/docs/generator_flow.png "from https://jpa.kapsi.fi/nanopb/docs/")

```
cd nanopb/
mkdir messages
cp ../protobuff_deps/messages.proto messages/
cp generator/proto/nanopb.proto ./
protoc -o message.pb messages/messages.proto
python generator/nanopb_generator.py message.pb
cp message.* ../protobuff_deps/
cd ./generator/proto
make
```
### python

https://developers.google.com/protocol-buffers/docs/pythontutorial
```
cd protobuff_deps
mkdir python
protoc --python_out=python messages.proto
```

## Converting St-Link to Segger J-link
https://www.segger.com/products/debug-probes/j-link/models/other-j-links/st-link-on-board/
Download software extract and run STLinkReflash.exe
1. Accept Segger license
1. Accept ST license
1. Choose Upgrade to J-Link [1] The device is flashed and reconnected
   Output looks like
```
Preparing for FW update (can take up to 10 seconds)...O.K.
Identifying ST-LINK variant...O.K.: ST-LINK/V2-1
Performing firmware update...O.K.
```
now you have an working J-Link at your STM32-Board.

## Debug Output

![RTT_pic](https://www.segger.com/fileadmin/_processed_/b/6/csm_J-Link-RTT_800x_21198b3c21.png "from https://www.segger.com")

https://www.segger.com/products/debug-probes/j-link/technology/about-real-time-transfer/

https://mcuoneclipse.com/2015/07/07/using-segger-real-time-terminal-rtt-with-eclipse/


##### Telnet Config for RTT dumping

| Param  | Value     |
|:-------|:----------|
| Host   | 127.0.0.1 |
| Timeout| 20 sec    |
| Port   | 19021     |
