import argparse
import crtx as crtx
import time


if __name__ == "__main__":
    radio = crtx.Crazyradio()
    radio.set_channel(7) 

    while True:    
        radio.send_packet([1,0,3])
        time.sleep(1)      
        radio.send_packet([2,1,4])
        time.sleep(1)
        radio.send_packet([3,2,5])
        time.sleep(1)
        radio.send_packet([4,3,6])
        time.sleep(1)      
        radio.send_packet([4,4,4])
        time.sleep(1)
        radio.send_packet([6,5,5])
        time.sleep(1)
        radio.send_packet([7,6,5])
        time.sleep(1)
        radio.send_packet([0,7,5])
        time.sleep(1)

    radio.close
