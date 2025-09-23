import can
import usb


import sys
import time


dev = usb.core.find(idVendor=0x1D50, idProduct=0x606F)

bus = can.Bus(interface="gs_usb", channel=dev.product, index=0, bitrate=250000)



# Receive a CAN message
while 1:
    try:
        
        received_message = bus.recv(100)
        if received_message:
            print((received_message.data[0]<<8)+received_message.data[1],
                  (received_message.data[2]),received_message.data[3],
                  (received_message.data[4]),received_message.data[5],
                  (received_message.data[6]<<8)+received_message.data[7])

           
            #time.sleep(0.2)
    except Exception as e:
        print(f"Fehler beim Empfang der Nachricht: {e}")
        time.sleep(1) # Längere Pause nach einem Fehler
