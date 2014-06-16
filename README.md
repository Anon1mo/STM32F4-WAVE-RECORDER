STM32F4-WAVE-RECORDER
=====================

  ------ STM32F4 Discovery WAVE(.wav) recorder ------
 	    	by Piotr Jakubowski and Filip Skurniak

  We based on ST example:
  http://www.st.com/st-web-ui/static/active/en/resource/technical/document/application_note/DM00040802.pdf
  and STM32DiscoveryVCP project by Xenovacivus ( https://github.com/xenovacivus/STM32DiscoveryVCP )

  You can record and save files on your computer using STM32F4 Discovery.

  What you need:
  --------------
  CooCox CoIDE (http://www.coocox.org/Downloads.htm)
  [ST-LINK/V2 in-circuit debugger/programmer](http://www.st.com/web/catalog/tools/FM146/CL1984/SC724/SS1677/PF251168).
  [Virtual Com Port driver](http://www.st.com/web/en/catalog/tools/PF257938) so you can talk to your STM32F4 once you      load this project.
  The C# program, which you can download from the site of our project in GitHub ( https://github.com/Anon1mo/STM32F4        -WAVE-RECORDER )
 
 What it does:
 ------------
 The built-in MP45DT02 Microphone captures PDM 1-bit audio samples, which are later converted to PCM 16-bit samples and
 aggregated in uint16_t arrays. When data from one array is being sent via COM port, the second array is being fullfiled
 with samples. We can to this thanks to standard C pointers, which switch arrays depending on the previously described    method. We get fluent sound without any delays. The standard sampling frequency is 16 kHz, but you can change it         depending on your needs.
 
 How to use it
 -------------
 Connect STM32F4 Discovery to your computer via USB and use micro-USB->USB cable to connect it to the computer from the  other side(it will create COM port). Download the program from CooCox to your board. If everything is initialized successfully, the two diodes will start shining. Then you need to press the USER BUTTON on your board. If you do it, you can move to C# application, open COM port(it will automatically detect the settings), name a file and click 'Record'. The recording has started and the green diode will shine. If you want to end the recording then press the USER BUTTON. All diodes will shine for a second and then they will be turned off. You'll get a piece of information in the application that the recording has ended. Now you can play the recorded sound.
 Information
 -----------
 If you have any questions, feel free to ask and send me an email lotropj@gmail.com
