import simtime
import socket
import struct
import time
import threading

import os

BROADCAST = -1

ns3conn = None
phase0_ongoing = False
ack_sem = threading.Semaphore(0)
uav_id = None
incoming_messages = [] # (payload, sender_id)

def _conn_thread(ns3_address):
    global ns3conn

    # Retry until we manage to connect
    print('Waiting for ns3 to be ready to accept our connection...')
    while True:
        try:
            ns3conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            ns3conn.settimeout(3)
            ns3conn.connect(ns3_address)
            ns3conn.settimeout(None)
            print('Connected to ns3')
            return
        except Exception as e:
            print('Still waiting, it seems that ns3 is not ready yet: {}'.format(str(e)))
            time.sleep(2)

def _recv_loop():
    while True:
        # Wait until new data arrives (without consuming it)
        w = ns3conn.recv(1, socket.MSG_PEEK)

        if phase0_ongoing:
            # We're currently in Phase 0, it must be an ACK
            v = ns3conn.recv(1)
            assert v == b'!'
            ack_sem.release()
        else:
            # We're currently in Phase 1, it must be an incoming message
            length = struct.unpack("<I", ns3conn.recv(4, socket.MSG_WAITALL))[0]
            payload = ns3conn.recv(length, socket.MSG_WAITALL)

            # Split sender field and payload
            sender_id = struct.unpack("<I", payload[:4])[0]
            payload = payload[4:]

            # Add to received queue
            incoming_messages.insert(0, (payload, sender_id))
            #print('MSG', sender_id, payload)

            # Send ACK
            ns3conn.send(b'!')

def _phase0_cb(phase0_ongoing_flag):
    global phase0_ongoing
    phase0_ongoing = phase0_ongoing_flag

def connect(ns3_address, local_id):
    global uav_id

    # Run initalisation on a separate thread to prevent socket timeouts from
    # being synchronised to the simulation clock, because the simulation stays
    # stalled until we connect
    t = threading.Thread(target=_conn_thread, args=((ns3_address, 9998),))
    t.start()
    t.join()

    uav_id = local_id
    print('Registering ns3 node #{}...'.format(uav_id))
    ns3conn.send(struct.pack("<I", uav_id))

    # Register _phase0_cb() to be called at the beginning and at the end of Phase 0
    simtime.register_phase0_callback(_phase0_cb)

    t = threading.Thread(target=_recv_loop)
    t.daemon = True
    t.start()

def local_id():
    return uav_id

def sendto(message, dest_id): # dest_id can also be BROADCAST
    assert phase0_ongoing == True # messages can only be sent during Phase 0

    payload = struct.pack("<i", dest_id) + message
    packet = struct.pack("<I", len(payload)) + payload
    ns3conn.sendall(packet)

    ack_sem.acquire() # wait for ack

def message_available():
    return len(incoming_messages) != 0

# to be called only if message_available() == True
def recvfrom(): # (payload, sender_id)
    return incoming_messages.pop()
