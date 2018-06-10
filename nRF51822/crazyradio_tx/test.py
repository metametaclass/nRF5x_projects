import argparse
import crtx as crtx
import time


if __name__ == "__main__":
    radio = crtx.Crazyradio()

    while True:    
        radio.send_packet([1])
        time.sleep(1)
        radio.send_packet([2])
        time.sleep(1)
        radio.send_packet([3])
        time.sleep(1)
        radio.send_packet([4])

    radio.close
