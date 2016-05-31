#!/usr/bin/python3.4
#Adapted from Yannick Jestin's program "rejeu"

from ivy import ivy
import os.path
import re
import collections
import logging
import time
import argparse
import sys
import signal
import gpxpy
import gpxpy.gpx
import random

DATA_PATH = './'
DEFAULTDATA = os.path.join(DATA_PATH, 'human_positions.gpx')
REGEX = "^([0-9.]+) (.*)"
# DEFAULTBUS = "224.5.6.7:8910" # on MacOSX
DEFAULTBUS = "127.255.255.255:2010"  # on Linux
speed = 100	# speed in cm/s. Determinated randomly.

class Rejeu():
    """une classe pour rejouer le trafic d'un log paparazzi sur le bus ivy"""

    def __init__(self, bus, filename):
        """Initialisation du rejeu sur un bus Ivy"""
        self.time = 0
        self.exit = False
        self.is_paused = False
        self.ivy = ivy.IvyServer('GPX_Replay', 'GPX Replay READY')
        ivy.ivylogger.setLevel(logging.ERROR)
        self.data = collections.OrderedDict(sorted(self.do_load(filename).items()))   #############
        for timestamp, text in self.data.items():
            self.time = timestamp
            break;
        self.ivy.bind_msg(self.do_pause, '^(Start|Stop)')
        self.ivy.start(bus)
        print("ready to send messages")

    def sigstop(self, signal, frame):
        """gestionnaire de signaux :si on interromp le programme par ^C"""
        self.exit = True

    def start(self):
        """lance la lecture de l'ensemble des événements"""
        self.do_play()
        self.ivy.stop()

    def do_pause(self, client, *p):
        """Ivy slot: exécuté quand on reçoit Start ou stop sur le bus"""
        self.is_paused = (p[0] == 'Stop')

    def do_load(self, filename):
        """Charge le contenu d'un fichier, et produit un dictionnaire indexé
        par les temps"""
        global speed
        print("loading " + filename) 
        gpx_file = open(filename, 'r')
        gpx = gpxpy.parse(gpx_file)
        gpx_file.close()
        out = {}
        for track in gpx.tracks:
            for segment in track.segments:
                for point in segment.points:
                    lat = int(point.latitude * 10**7)	# convert coordinates to int32
                    lon = int(point.longitude * 10**7)
                    alt = int(point.elevation * 10**3)
                    time = int(point.time.timestamp())
                    speed = int(0.7*speed + 0.3*random.randint(80,500))		# try to do something with speed
                    out[time] = "replay HUMAN_GPS {} {} {} {}".format(lat, lon, alt, speed)
        self.time = out.get
        print("end processing " + filename)
        return out

    def do_play(self):
        """envoie les messages sur le bus, en attendant si on est en pause"""
        print("playing messages")
        for timestamp, text in self.data.items():
            if self.exit: return
            while self.is_paused:
                time.sleep(0.1)
            time.sleep(float(timestamp-self.time))
            self.time = timestamp
            self.ivy.send_msg(text)
        print("end playing messages")


def args(argv):
    """analyse les arguments en ligne de commande"""
    parser = argparse.ArgumentParser(description='rejeu de données paparazzi')
    parser.add_argument('-b', type=str, help='Ivy bus domain')
    parser.add_argument('-f', type=str, help='file name of data')
    args = parser.parse_args()
    bus = args.b if args.b else DEFAULTBUS
    f = args.f if args.f else DEFAULTDATA
    return (bus, f)

if __name__ == "__main__":
    """programme principal"""
    (bus, f) = args(sys.argv)
    r = Rejeu(bus, f)
    signal.signal(signal.SIGINT, r.sigstop)
    r.start()
