from wifi_rssi_collection.utils import *

def retriveAPinfo(interface):
    packets = sniff(interface, count = 100)
    for pkt in packets:
        print(pkt)
