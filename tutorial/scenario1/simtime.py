import math
import monotonic
import socket
import struct
import sys
import time
import threading

simtime_socket = None
current_timestamp = None

# NOTE: we cannot use threading.main_thread() bacause it is Python 3 only
main_thread = threading.current_thread()

orig_sleep_fn = time.sleep
orig_time_fn = time.time

# These "observers" are notified when phase 0 begins / ends
phase0_callbacks = []
phase0_ongoing = False

def wait_for_begintick():
    global current_timestamp

    current_timestamp = struct.unpack('<d', simtime_socket.recv(8, socket.MSG_WAITALL))[0]
    #print('RECEIVED begintick', current_timestamp)

    phase0_ongoing = True
    for fn in phase0_callbacks:
        fn(phase0_ongoing)

def signal_endtick():
    phase0_ongoing = False
    for fn in phase0_callbacks:
        fn(phase0_ongoing)

    simtime_socket.send(b'!')
    #print('SENT endtick', current_timestamp)

def new_sleep(n):
    # we only synchronize the main thread to avoid having to deal with
    # mutexes and condition variables
    if threading.current_thread() == main_thread:
        expiration_timestamp = new_time() + n
        while new_time() < expiration_timestamp:
            if current_timestamp is not None:
                signal_endtick()
            wait_for_begintick()
    else:
        orig_sleep_fn(n)

def new_time():
    #print('TIME', current_timestamp)

    if current_timestamp is not None:
        return current_timestamp
    else:
        return 0

def connect(port):
    global simtime_socket

    simtime_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    simtime_socket.connect(("127.0.0.1", port))

    simtime_socket.send(b'\0') # Subscribe to the first phase

    # replace some standard python functions
    time.sleep = new_sleep
    time.time = new_time
    monotonic.monotonic = new_time

def register_phase0_callback(fn):
    phase0_callbacks.append(fn)
    fn(phase0_ongoing)
