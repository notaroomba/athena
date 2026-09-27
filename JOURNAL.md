<!--
  Exported from Hack Club Blueprint on 2026-09-27.
  Source: https://blueprint.hackclub.com/projects/1124
-->

## 11/2/2025 - Fixed USB Hub Spec / Misc Problems / LED Problems  

_Time spent: 31.0h_  

![IMG-20251028-WA0024](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6NjMxMiwicHVyIjoiYmxvYl9pZCJ9fQ==--41850544da3487ad6cffc63088dd5dafacfb3717/IMG-20251028-WA0024.jpg)

So my board finally came in and there were a few problems...

First of all was that when making the USB Hub I had some 15K pulldown resistors as I followed the schematic. Then when I was revising it a month later before I ordered it, I marked them as DNP and removed the pads... (subtle foreshadowing)

When It finally arrived, I connected it to USB-C and nothing happened. I assume it was because I havent configured USB-PD yet. Then I decided to connect a 1s LiPo battery and it finally connected to my laptop. The only problem was that it didn't recognize the 3 STM32's that I had placed. It was only a few hours later after debugging that I noticed why... (BECAUSE OF THE DNP RESISTORS)

Finally when I pressed the boot button, the entire board "reset" and it disconnected and reconnected to my computer... Revising the PCB...
![image](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6NjMxMywicHVyIjoiYmxvYl9pZCJ9fQ==--a394c606041831721cd5fff236df3f00ca126ffe/image.png)
...the definition of fucking around and finding out

Now that I have succesfully identified both problems with my board, I set out to fix them. The only problem is, I didn't know how to solder so I tore open a wifi router that was unused and practiced soldering and desoldering 0603 and 0402 components for a few days. This because I was planning on buying some resistors online and soldering it myself. 

![WhatsApp Image 2025-10-21 at 21.35.53_7902855d](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6NjMxNCwicHVyIjoiYmxvYl9pZCJ9fQ==--40bf96547da776257d9d2657fb423c06f8e9078d/WhatsApp%20Image%202025-10-21%20at%2021.35.53_7902855d.jpg)

After running out of allowance on customs and tariffs, I couldn't order SMD resistors and then had to go downtown and find some, but I couldn't find any so I settled for normal resistors (Through Hole). After receiving the resistors I now was faced with the horror of soldering so I asked a friend if they could lend me their fine tipped soldering iron/magnifying glass and got to work.

The result after a little help and a bunch of youtube tutorials...
![WhatsApp Image 2025-10-28 at 12.38.26_463903b8](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6NjMxNSwicHVyIjoiYmxvYl9pZCJ9fQ==--332aa4cff429b4ce37845b0d1577ebfb464733d7/WhatsApp%20Image%202025-10-28%20at%2012.38.26_463903b8.jpg)

I still have another PCB to do but I am going to wait to order the SMD resistors so it looks prettier. I then had to desolder the GND pin of the button, raise it up a little, and then tried to solder a thin copper connecting the resistor to the button correctly, after a few failed attempts I finally got something that didn't cause a short. I put a little piece of sandpaper under so that the button wouldn't fall down again when pressed and cause another short lol.

![WhatsApp Image 2025-10-28 at 14.22.05_a0f7b6d9](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6NjMxNiwicHVyIjoiYmxvYl9pZCJ9fQ==--6c36e47432f1c4227fcaf3273a4047f7c0fe5284/WhatsApp%20Image%202025-10-28%20at%2014.22.05_a0f7b6d9.jpg)
(its the second button that looks a little raised)

Now I could finally start to program! I plugged in the battery and finally my computer detected it as 3 STM32's, also the SPU's status light turned green because I'm assuming for the UART conenction:

![image](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6NzgzMywicHVyIjoiYmxvYl9pZCJ9fQ==--df3b4c9ba10221e50d526b62edc2c313ca2eb5aa/image.png)

![GreenLight](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6NzgzNCwicHVyIjoiYmxvYl9pZCJ9fQ==--00b8e5c9ec5d2202604a0ad8cb1eba56d9e17b39/GreenLight.jpg)


I started working on turning on the LED's but couldn't get them to change so I went to sleep. The next day when I tried to turn on the board, I connected the battery and no lights came on. I pulled out my multimeter and after a few hours of debugging I realized that the 3.3V buck converter wasn't turning on so I assumed it was the battery, after putting 2 battery cells in series the board turned on. So then I decided to take 2 old batter cells from old walkie talkies and I charged them and then soldered them in series and connected them to the board.

![WhatsApp Image 2025-11-02 at 13.14.12_a31d992a](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6Nzg0MiwicHVyIjoiYmxvYl9pZCJ9fQ==--13f56d1ade32cf467055f8af64f87af2cf658b8c/WhatsApp%20Image%202025-11-02%20at%2013.14.12_a31d992a.jpg)

After that I started to program and got the MCU to blink a white light. I'm still having issues changing the lights so I'm going to work on that.

![WhatsApp Image 2025-11-02 at 13.18.15_8c6d99f5](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6Nzg0MywicHVyIjoiYmxvYl9pZCJ9fQ==--fb0f8330188f88bce283f3872c4a798f03100ce4/WhatsApp%20Image%202025-11-02%20at%2013.18.15_8c6d99f5.jpg)
  

## 11/14/2025 - Lights and Power Issue  

_Time spent: 13.0h_  

Finally got a light/status driver that I wrote for all 3 MCU's working and made a general makefile/flash script to automate flashing all 3 at the same time with STM32CubeMX_CLI. Although I encountered a bug where the MPU doesn't want to light up at all. 

After that the board suddenly turned off because the batteries died, so I am now recharging the batteries (1 by 1 with an old flight controller with a tp4506) while I write firmware to get USB-C PD up and running.

![WhatsApp Image 2025-11-13 at 14.15.59_84d74b2a](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6MTA4NzcsInB1ciI6ImJsb2JfaWQifX0=--46fd5ab178ccf53ea91f86c455a29e37e6115f28/WhatsApp%20Image%202025-11-13%20at%2014.15.59_84d74b2a.jpg)


![WhatsApp Image 2025-11-13 at 12.58.19_5f13501e](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6MTA4NjUsInB1ciI6ImJsb2JfaWQifX0=--b6a151ae86cda06782c4959890664811b8b7876f/WhatsApp%20Image%202025-11-13%20at%2012.58.19_5f13501e.jpg)

After a while I encountered a problem and looking through the schematic, I made another oppsie. I FORGOT PULLUPS ON THE I2C LINES. IT WAS THE ONLY TIME I HAD EVER USED I2C ACROSS THE ENTIRE BOARD AND I FORGOT THE PULLUPS. So now looking through the PCB, thankfully the I2C lines are close to VCC on the STM so I can connect resistors there:

![image](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6MTA4NzgsInB1ciI6ImJsb2JfaWQifX0=--759546e0178f0b4a4157655e79abab0cfa47834f/image.png)

For the I2C lines connecting the battery charger and the USB-C PD, I will have to scrape a bit over here and jump the resistor over to VCC.

![image](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6MTA4ODAsInB1ciI6ImJsb2JfaWQifX0=--3c89bc8375bbc3397ff9915283e7fc0499a95444/image.png).

For now I am going to try and get the BMP388 up and running. Debugging a bit more I found the cause as to why the MPU wasn't turning on. I had connected the RESET pin to the SPU to be able to reset it just in case and I forgot to pull it high by default in STM32CubeMX so it was causing an issue there.

After a few hours of debugging I finally got it working:
![image](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6MTA5NTksInB1ciI6ImJsb2JfaWQifX0=--7ab879d15bfe9298d8fd933d46ee8ba5edb43ae1/image.png)

I used this driver from [libdriver](https://github.com/libdriver/bmp388) and then added in the functions to make it work and now I finally got pressure and temperature data. I had a problem with floating points but then just had to multiply and use modulus to get the values.

After debugging a bit more, I was wondering why when I plugged in the USB-C the board wasn't turning on. After looking through it, I found out this wonderful connection in the USB-C PD IC:

![image](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6MTA5NjgsInB1ciI6ImJsb2JfaWQifX0=--d7c26d05f73c76854e9cb45ffb7ea0f6cbd724cb/image.png)


I'm guessing that my laptop detected this short and disabled power to the device (wooo), which also means that to use it/program it, I will have to use a battery (like I have been doing). I am tempted to buy a hot air gun and try and fix it myself. 

![image](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6MTA5NzIsInB1ciI6ImJsb2JfaWQifX0=--5aa547b54cab2fed78242b82d8af70d024cb344b/image.png)


All I'd have to do is scrape away around the pin and then using a thin wire, connect it to the GND of the capacitor across from the stitching vias.  

## 12/3/2025 - Barometers Done and Start IMUs  

_Time spent: 10.0h_  

So it's been a while since I've touched this project but it's time to come back. Since my last journal I've been working on it intermittently and finally got one of the barometers working. This wasn't without it's hardships as the first problem that I encountered was with the ICP not wanting to be read/wrote. After a few hours of debugging I found out that the data size in the SPI settings of STM32CubeMX was incorrect:

![image](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6MTc0NDMsInB1ciI6ImJsb2JfaWQifX0=--ac72cb50713be57a1f427552c359ae36386dd939/image.png)

I used to be working with 4 bits but after READING THE DATASHEET I found out that it needed to be 8 bits. After I got that working I then started work on the BMP388, again with more hardships, at first the sensor didn't want to be read at all both in it's simple mode and FIFO (first in first out) so then I had to debug a bit and found out that I was using the wrong baud rate for the SPI line (20 MBit/s instead of 10MBit/s) and that caused errors that I didn't know how to resolve UNTIL I READ THE DATASHEET. (theres a pattern lol)

![image](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6MTc0NDIsInB1ciI6ImJsb2JfaWQifX0=--bdb1a969136400131263db8d5a2c873da7b3a941/image.png)

Now after tinkering around I found out that FIFO is not what I am looking for as it bunches up the data and is not good for real-time applications. Still I used the interrupt handler together with the one-shot configuration to only get data when it is ready to be gotten. Now after that's done it was time to move on to the IMUs...

Before that though I also found out that one of the Li-ion batteries I had was completely dead and I was reusing it so I found another one and am using that one for now.

There is also a bit of an issue as I am trying to use both the BMP and the ICM and the BMP is taking a while to read its registers. This was because the driver waits 5000 MS to get/check the registers if the data isn't ready:

![image](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6MTc1NDYsInB1ciI6ImJsb2JfaWQifX0=--f86ee3918b701237980458cfdb5b08b7777433d1/image.png)

I have tried using interrupts but the problem continues, when I enable the interrupt for the BMP it halts the other barometer for a few seconds as if it was measuring data. I am running a clean build to see if there is anything left over but I don't think so. And nada, I'll come back to the BMP as I at least got one pressure sensor working.

Moving on to the IMUs and looking at the schematic, I added in an FSYNC timer pin thing:

![image](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6MTc1NjEsInB1ciI6ImJsb2JfaWQifX0=--381e08315d2bcd21178e544de77745ea1f4b4103/image.png)

LOOKING AT THE DATASHEET (wooo) the clock supports up to 40kHz:

![image](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6MTc1NjQsInB1ciI6ImJsb2JfaWQifX0=--891af4da0e0762a37c910f1a6477dd4e11a554cc/image.png)

and with no other mention in the datasheet I'll look into the drivers.

![image](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6MTc2MzMsInB1ciI6ImJsb2JfaWQifX0=--f75ba5e0c53ee948298ccc8861f944c89948583a/image.png)

I had another problem that when setting the default to HIGH in STM32CubeMX, the SET/RESET GPIO functions were inverted so that threw everything off. I'll update the other sensors and keep this in mind when working with STM32 in the future. I have tried implementing the basic code in the repository but it kept not recognizing the IMUs.

I am going to further debug it but later because the batteries have died and I have to recharge them :sob:  

