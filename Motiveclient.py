
import time
import socket
import struct 
from threading import Thread
import logging

# create structs for reading various object types.
# format: f is float, i is integer, d is double in C type
# e.g., the format 'fff' stands for 'float float float'
# '<' indicates little-endian byte-order
Vector3 = struct.Struct('<fff') 
Quternion = struct.Struct('<ffff')
DoubleValue = struct.Struct('<d')

# use your client IP address
myIP = "192.168.1.72"
# use your server IP address
serverIP = "192.168.0.93"
# note the multicast IP address and 
# data and command socket port numbers from Motive SW.
# if they don't match, there won't be a connection.
multiCastAddress = "239.255.42.99"

# Client/server message id for each NatNet message (as in NatNetTypes.h)
NAT_FRAMEOFDATA = 7


class MotiveClient:    
    # constructor 
    def __init__(self):
        self.serverIPAddress = serverIP
        self.multicastAddress = multiCastAddress
        # specifies the port to be used for streaming data from the server to the client
        self.dataPort = 1511
        # Callbacks
        self.rigidBodyClient = None

    # main
    def run(self): 
        # socket and thread
        self.dataSocket = self.createSocket(self.dataPort)
        if self.dataSocket is None:
            raise RuntimeError("Could not start the data streaming")
    
        dataThread = Thread(target=self.threadFunction, args=(self.dataSocket,))

        dataThread.start()

    # sockets 
    def createSocket(self, port):
        # create a new UDP socket
        newSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        newSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        # Specify myIP as the interface to subscribe to multicast through
        newSocket.setsockopt(socket.SOL_IP, socket.IP_ADD_MEMBERSHIP,socket.inet_aton(multiCastAddress) + socket.inet_aton(myIP))
        newSocket.bind((multiCastAddress,port))
        return newSocket
    
    # threads 
    def threadFunction(self, socket): 
        # receive data from the socket. 
        # the return value is a pair (bytes, address) where bytes is 
        # a bytes object representing the data received
        # and address is the address of the socket sending the data
        data, address = socket.recvfrom(32768) # 32k byte buffer size

        if len(data) > 0:
            self.parseMessage(data)
    
       
    # initial console messages and message parsing
    def parseMessage(self, data):
        # Motive SW packs the data mainly into two different formats: 
        # 1.) data descriptions and 2.) frame-specific tracking data.
        # Utilizing this format, the client application can discover which data are streamed out from the server application
        # in advance to accessing the actual tracking data.
        # In this function we only access the frame-specific tracking data using messageID = NAT_FRAMEOFDATA.
        
        offset = 0
        
        logging.info("\n------------\nBegin Packet")

        # message ID (e.g. NAT_FRAMEOFDATA) ---uint16_t
        messageID = int.from_bytes(data[offset:offset+2], byteorder ='little')
        offset =+2

        # Num bytes in payload (packet size) ---uint16_t
        offset +=2

        if messageID == NAT_FRAMEOFDATA:
            self.unpackMotiveData(data[offset:])
        else:
            logging.info("ERROR: Unrecognized packet type")

        logging.info("End Packet\n----------")


    # depacketizing Motive data packets directly          
    def unpackMotiveData(self, data):
        # as in NatNetTypes.h
        data = memoryview(data)
        offset = 0
        
        # Frame number 4 bytes--- int32_t
        frameNumber = int.from_bytes(data[offset:offset + 4], byteorder='little')
        offset += 4
        logging.info("Frame: {}".format(frameNumber))
        
        # Just skip the following 
        # Marker sets 4 bytes--- int32_t
        offset += 4
        # Unlabeled markers 4 bytes--- int32_t
        #unlabeledMarkersCount = int.from_bytes(data[offset:offset + 4],byteorder='little')
        offset += 4
        # offset += 12 * unlabeledMarkersCount
        # Rigid bodies  4 bytes--- int32_t
        rigidBodyCount = int.from_bytes(data[offset:offset + 4], byteorder='little')
        offset += 4
        logging.info("Rigid Body Count: {}".format(rigidBodyCount))
        for i in range(rigidBodyCount):
            offset += self.unpackRigidBodyData(data[offset:])
        
        # Just skip the following
        # labeled Marker Count 4 bytes--- int32_t
        offset += 4
        # force Plate Count  4 bytes--- int32_t
        offset += 4
        # device Count 4 bytes--- int32_t
        offset += 4
        # Timecode --- uint32_t 
        offset += 4
        # Sub-timecode
        offset += 4

        # Timestamp  --- double
        timestamp, = DoubleValue.unpack(data[offset:offset + 8])
        offset += 8
        logging.info("Timestamp: {}".format(timestamp))

    def unpackRigidBodyData(self,data):
        # as in NatNetTypes.h
        offset = 0

        #  rigidBody identifier 4 bytes--- int32_t
        rigidBodyID = int.from_bytes(data[offset:offset + 4], byteorder = 'little')
        offset += 4

        # logs a message with level INFO on this logger
        # str.format() allows multiple substitutions and value formatting
        logging.info("\tID: {}".format(rigidBodyID))

        # position of the rigid body
        # each postion value occupies 4 bytes---float
        rigidBodyPositon = Vector3.unpack(data[offset:offset + 12])
        offset += 12
        logging.info("\t\tPosition; {}".format(rigidBodyPositon))

        # orientation of the rigid body---float
        rigidBodyOrientation = Quternion.unpack(data[offset: offset + 16])
        offset += 16
        logging.info("\t\tOrientation: {}".format(rigidBodyOrientation))

        # send  the information to any client in the network
        if self.rigidBodyClient is not None:
            self.rigidBodyClient(rigidBodyID,rigidBodyPositon,rigidBodyOrientation)

   
if __name__ == "__main__":
    # IRIDIA client
    newClient = MotiveClient()
    newClient.run()
