import time
import socket
import struct
from threading import Thread
import coloredlogs, logging


# Create structs for reading various object types to speed up parsing.
Vector3 = struct.Struct('<fff')
Quaternion = struct.Struct('<ffff')
IntValue = struct.Struct('<i')
FloatValue = struct.Struct('<f')
DoubleValue = struct.Struct('<d')


# use your client IP address
myIP = "192.168.0.96"
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
    # Logging
    coloredlogs.install(level='INFO', fmt='MotiveDATA: %(message)s',
                          level_styles={'info': {'color': 'red'}})

  def run(self):
    # Data socket and thread
    self.dataSocket = self.createSocket(self.dataPort)
    if self.dataSocket is None:
      raise RuntimeError("Could not open data channel")
    dataThread = Thread(target=self.threadFunction, args=(self.dataSocket,))

    dataThread.start()

  # sockets 
  def createSocket(self, port):
    # create a new UDP socket
    newSocket = socket.socket(socket.AF_INET,  # Internet
                           socket.SOCK_DGRAM,
                           socket.IPPROTO_UDP)  # UDP
    newSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    # Specify my_ip as the interface to subscribe to multicast through
    newSocket.setsockopt(socket.SOL_IP, socket.IP_ADD_MEMBERSHIP,
                      socket.inet_aton(self.multicastAddress)
                      + socket.inet_aton(myIP))


    newSocket.bind((self.multicastAddress, port))
    return newSocket

  
  def threadFunction(self, socket):
    # receive data from the socket. 
    # the return value is a pair (bytes, address) where bytes is 
    # a bytes object representing the data received
    # and address is the address of the socket sending the data
    while True:
      # Block for input
      data, addr = socket.recvfrom(32768)  # 32k byte buffer size
      if len(data) > 0:
        self.parseMessage(data)

  # initial console messages and message parsing
  def parseMessage(self, data):
    # Motive SW packs the data mainly into two different formats: 
    # 1.) data descriptions and 2.) frame-specific tracking data.
    # Utilizing this format, the client application can discover which data are streamed out from the server application
    # in advance to accessing the actual tracking data.
    # In this function we only access the frame-specific tracking data using messageID = NAT_FRAMEOFDATA.
        
    logging.info("\n------------\nBegin Packet")

    # message ID (e.g. NAT_FRAMEOFDATA) ---uint16_t
    messageID = int.from_bytes(data[0:2], byteorder='little')
    logging.info("Message ID: {}".format(messageID))
    # Skip the getting Num bytes in payload (packet size) ---uint16_t
    offset = 4
    if messageID == NAT_FRAMEOFDATA:
      self.unpackMotiveData(data[offset:])
    else:
      logging.info("ERROR: Unrecognized packet type")

    logging.info("End Packet\n----------")

  # depacketizing Motive data packets directly as in NatNetTypes.h  
  def unpackMotiveData(self, data):
    data = memoryview(data)
    offset = 0

    # Frame number (4 bytes)
    frameNumber = int.from_bytes(data[offset:offset + 4], byteorder='little')
    offset += 4
    logging.info("Frame: {}".format(frameNumber))
    
    # Marker sets
    markerSetCount = int.from_bytes(data[offset:offset + 4], byteorder='little')
    offset += 4
    for i in range(markerSetCount):
      offset += self.unpackMarkerSet(data[offset:])

    # Unlabeled markers int32_t
    unlabeledMarkersCount = int.from_bytes(data[offset:offset + 4],
                                           byteorder='little')
    offset += 4
    # Just skip them
    offset += 12 * unlabeledMarkersCount

    # Rigid bodies int32_t
    rigidBodyCount = int.from_bytes(data[offset:offset + 4], byteorder='little')
    offset += 4
    logging.info("Rigid Body Count: {}".format(rigidBodyCount))
    for i in range(rigidBodyCount):
      offset += self.unpackRigidBodyData(data[offset:])

    # Skeletons int32_t
    # Just skip them
    offset += 4

    #  Labeled markers int32_t
    # Just skip them
    labeledMarkerCount = 0
    labeledMarkerCount = int.from_bytes(data[offset:offset + 4],byteorder='little')
    offset += 4
    offset += 26*labeledMarkerCount

    # Force Plate data int32_t
    # Just skip them
    offset += 4

    # Device data int32_t
    # Just skip them
    offset += 4
    
    # Timecode uint32_t 
    # Just skip them
    offset += 4
    # timecode Sub uint32_t 
    # Just skip them
    offset += 4

    # Timestamp double
    timestamp, = DoubleValue.unpack(data[offset:offset + 8])
    offset += 8
    logging.info("Timestamp: {}".format(timestamp))

    # skip mid cam exposure time stamp double
    offset += 8
    # skip Camera data received timestamp double
    offset += 8
    # skip Transmit timestamp double
    offset += 8

    # Frame parameters int16_t
    # Just skip them
    offset += 2

  def unpackMarkerSet(self, data):
    # as in NatNetTypes.h 
    offset = 0
    # Model name
    modelName, separator, remainder = bytes(data[offset:]).partition(b'\0')
    offset += len(modelName) + 1
    # Marker count
    markerCount = int.from_bytes(data[offset:offset + 4], byteorder='little')
    offset += 4
    # Markers
    offset += 12*markerCount
    return offset

  def unpackRigidBodyData(self, data):
    # as in NatNetTypes.h
    offset = 0
    # rigidBody identifier 4 bytes--- int32_t
    rigidBodyID = int.from_bytes(data[offset:offset + 4], byteorder='little')
    offset += 4
    logging.info("\tID: {}".format(rigidBodyID))

    # position of the rigid body
    # each postion value occupies 4 bytes---float
    rigidBodyPositon = Vector3.unpack(data[offset:offset + 12])
    offset += 12
    logging.info("\t\tPosition: {}".format(rigidBodyPositon))

    # orientation of the rigid body---float
    rigidBodyOrientation = Quaternion.unpack(data[offset:offset + 16])
    offset += 16
    logging.info("\t\tOrientation: {}".format(rigidBodyOrientation))

    # skip the Mean marker error
    offset += 4

    # skip the tracking status info
    offset += 2

    return offset


if __name__ == "__main__":
  # Optitrack client
  streamingClient = MotiveClient()
  streamingClient.run()
