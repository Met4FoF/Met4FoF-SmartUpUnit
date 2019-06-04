# Software for the Met4FoF Smartup Unit V2
# Build System.
The software was and is created with [SW4STM32](http://www.openstm32.org/HomePage) (gcc).
[SEGGER Systemview](https://www.segger.com/products/development-tools/systemview/) can be used for debugging.

# Dependencies and used Libarys

| Used for   | Libary Name   | Version   | Link   |
|:-------------|:-------------|:-----| :---|
|real-time operatingsystem|FreeRTOS|9.0.0|https://www.freertos.org/FreeRTOS-V9.html|
|Ethernet|LWIP|2.0.3|[ST DM00103685]( https://www.st.com/content/ccc/resource/technical/document/user_manual/65/e8/20/db/16/36/45/f7/DM00103685.pdf/files/DM00103685.pdf/jcr:content/translations/en.DM00103685.pdf)|
|Data serialization|nanopb|a2db482...|https://github.com/nanopb/nanopb|

# using nanopb to genreate Code
see also https://jpa.kapsi.fi/nanopb/docs/
and https://github.com/nanopb/nanopb/blob/master/docs/concepts.rst

![nanopb_picture](https://jpa.kapsi.fi/nanopb/docs/generator_flow.png "from https://jpa.kapsi.fi/nanopb/docs/")

```
cd nanopb/
mkdir messages
cp ../protobuff_deps/messages.proto messages/
cp generator/proto/nanopb.proto ./
protoc -omessage.pb messages/messages.proto
python generator/nanopb_generator.py message.pb
cp message.* ../protobuff_deps/```

# Debug Output

![RTT_pic](https://www.segger.com/fileadmin/_processed_/b/6/csm_J-Link-RTT_800x_21198b3c21.png "from https://www.segger.com")

https://www.segger.com/products/debug-probes/j-link/technology/about-real-time-transfer/

https://mcuoneclipse.com/2015/07/07/using-segger-real-time-terminal-rtt-with-eclipse/
