import serial
import time
import os
import glob
import serial.tools.list_ports
import math
import sys
from tqdm import tqdm
import crcmod.predefined
import io

#CONSTANTS

FORMAT = 'ASCII'
CACHE_SIZE = 256 #(bytes) Size of the Serial Cache on the LoRa Moduel
PKT_SIZE = 68 #(bytes) Total size of the packet
# Note: pkt_number could be reduced to 1 in future builds, unless packet loss detection is wanted
PKT_CHECKSUM_SIZE = 2 #(bytes) Header (first 2 bytes of packet)
PKT_NUMBER = 2 #(bytes) (next 2 bytes of packet)
PKT_PAYLOAD = PKT_SIZE - PKT_CHECKSUM_SIZE - PKT_NUMBER #(bytes) Actual data of the packet
PKT_NO_CHECKSUM = PKT_SIZE - PKT_CHECKSUM_SIZE #(bytes) once checksum is removed and processed
SERIAL_ACK = "PacketReceived\n"
ACK_SIZE = 15 #(bytes) Size of the serial acknowledgement statement
TIMEOUT = 10 #(seconds) Timeout of serial port
# TEST_FILEPATH = "Cmprs_Segments" #Filepath for transmitted files
TEST_FILEPATH = "Compressed_Segments" #Filepath for transmitted files
RX_FILEPATH = "RawData" #Filepath for transmission data, time to receive, data received, etc.
INIT_PACKET_SENDS = 25 #How many times to send the initial header packet of a file

def Sanitize_Serial(serial_data):
    try:
        sanitized_data = serial_data.decode(FORMAT)
        return sanitized_data
    except UnicodeDecodeError:
        print("Error, received pkt: ", serial_data)

def FindPortName():
    ports = serial.tools.list_ports.comports()
    if sys.platform.startswith('win'):
        for port in ports:
            port_descrip = port.description.split("(")[0]
            if(port_descrip == "Silicon Labs CP210x USB to UART Bridge "):
                return port.device
        return
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/ttyUSB[0-9]*')
        if(len(ports) == 1):
            return ports[0]
        return


def SerialConnect():
    """
    SerialConnect attempts to connect to a LoRa Module connected via USB, handling for no connection

    :param port: string of the port that LoRa USB should connect to.
    :return: Serial Object related to LoRa USB Connection
    """ 

    port = FindPortName()

    if not port:
        print("Failed to find port...")
        quit()

    connection_count_timeout = 0
    while(True):
        #Establish connection to Serial Port
        try:
            ser = serial.Serial(port, 115200, timeout=TIMEOUT)
        #If connection fail wait 3 seconds and try again
        #If fails 10 times then cancel connect
        except:
            print("No Serial Connection Detected on " + port + " - Reattempting...")
            time.sleep(3)
            connection_count_timeout += 1
            if(connection_count_timeout == 10):
                print("No connection after 10 attempts, exiting")
                sys.exit()
            continue
        #Serial Connection Established
        else:
            print("Connection Established to port: " + port + '\n')
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            return ser
        
def ClearSerialInput(ser: serial.Serial):
    bytes_to_clear = ser.in_waiting
    junk_bytes = ser.read(bytes_to_clear)
    # junk_bytes = len(junk_bytes)
    # print("Cleared ", junk_bytes, " bytes from serial")
    return

def readRSSI(ser: serial.Serial, discard_check):
    rssi = ser.read_until(expected= b':RSSI:')
    if(discard_check == False):
        rssi = Sanitize_Serial(rssi)
        rssi = rssi.rstrip(':RSSI:')
        rssi = int(rssi)
        return rssi
    else:
        return 

def WaitForInitialPkt(ser: serial.Serial):
    """
    WaitForInitialPkt waits for a packet from the Tx LoRa module that notifies details about the file about to be sent over LoRa and through Serial

    :param ser: Serial Object related to LoRa USB Connection
    :return: PKT_SIZE number of bytes
    """ 
    while True:
        init_packet = ReadPacketSerial(ser)
        if(init_packet):
            if(len(init_packet) >= PKT_NO_CHECKSUM):
                try:
                    received_string = Sanitize_Serial(init_packet)
                    init_packet_split = received_string.split(":", 2)

                    filename = init_packet_split[0]
                    filetype = filename.split(".")
                    filetype = filetype[1]
                    print("Receiving file: ", filename)
                    
                    filesize = int(init_packet_split[1])
                except:
                    pass
                else:
                    return init_packet

def ParseHeaderPacket(ser: serial.Serial, init_packet, spreadfactor, bandwidth, distance):
    """
    ParseHeaderPacket takes the initial header packet sent by the Tx LoRa and decodes it to know what filetype,filename, and filesize to expect

    :param ser: Serial Object related to LoRa USB Connection
    :param init_packet: initial string packet of a file that contains file name and size
    :param spreadfactor: manually entered in sf, for data collection purposes
    :param bandwidth: manually entered in bw, for data collection purposes
    :return: N/A
    """ 

    now = time.ctime()
    print("Transmission start... ", now)
    original_init_pkt = init_packet
    init_packet = Sanitize_Serial(init_packet)
    init_packet_split = init_packet.split(":", 2)

    filename = init_packet_split[0]

    filename = filename[PKT_NUMBER:]
    
    filesize = int(init_packet_split[1])

    #Sends rest of data to be read and save
    FileSaveSerial(ser,filename,filesize, spreadfactor, bandwidth, original_init_pkt, distance)
    now = time.ctime()
    print("File Transmit complete... ", now, '\n')

    return filename

def ReadPacketSerial(ser: serial.Serial):
    """
    ReadPacketSerial reads PKT_SIZE amount of bytes from the Serial Connection, this funciton will block until this number of bytes has been read.

    :param ser: Serial Object related to LoRa USB Connection
    :return: PKT_SIZE number of bytes from Serial Connection
    """ 

    rec_packet = ser.read(PKT_SIZE) #Read PKT_SIZE
    packet = VerifyPacketChecksum(rec_packet, ser)
    if(packet):
        return packet
    else:
        return

def SerialReadyAck(ser: serial.Serial):
    """
    SerialReadyAck waits for a serial acknowledgement from the Tx LoRa Module that it's ready to send another packet, this also blocks until this acknowledgement arrives. This function is necessary otherwise the serial would flood the Tx LoRa faster than it can send packets over LoRa

    :param ser: Serial Object related to LoRa USB Connection
    :return: Nothing if Ack received, otherwise send what it got over Serial(Error-case)
    """ 
    while True:
        rec_packet = ser.read_until(expected= b'\n' , size = ACK_SIZE)
        rec_packet = Sanitize_Serial(rec_packet)
        if(rec_packet == SERIAL_ACK):
            break
    return 

def AddPacketChecksum(packet):

    payload_bytes = packet[PKT_NUMBER:]

    # Create a CRC-16 function with the "crc-16" preset
    crc16_func = crcmod.predefined.mkCrcFun('crc-16')

    # Calculate CRC-16 checksum for the packet
    crc_value = crc16_func(payload_bytes)

    # Convert the 16-bit CRC value to a 2-byte representation
    checksum_bytes = crc_value.to_bytes(2, byteorder='big')

    packet = checksum_bytes + packet
    return packet

#Adds a 2 byte checksum to the front of 
def VerifyPacketChecksum(packet, ser: serial.Serial):
    crc = packet[0:PKT_CHECKSUM_SIZE]
    payload_bytes = packet[PKT_CHECKSUM_SIZE+PKT_NUMBER:]

    # Create a CRC-16 function with the "crc-16" preset
    crc16_func = crcmod.predefined.mkCrcFun('crc-16')


    # Calculate CRC-16 checksum for the packet
    crc_value = crc16_func(payload_bytes)

    # Convert the 16-bit CRC value to a 2-byte representation
    calculated_crc = crc_value.to_bytes(2, byteorder='big')
    if(crc == calculated_crc):
        return packet[PKT_CHECKSUM_SIZE:]
    else:
        #If not a proper packet then discard, also need to discard RSSI data that would follow it
        try:
            if(len(packet) > 0):
                print("Checksum Error, pkt : ", packet.decode(FORMAT)) 
            #Discards RSSI data
            if(len(packet) > 0):
                readRSSI(ser, False)
        except:
            if(len(packet) > 0):
                print("Checksum Error, pkt : ", packet)
            #Discards RSSI data
            if(len(packet) > 0):
                readRSSI(ser, False)
        return False

def WritePacketSerial(ser: serial.Serial, packet):
    """
    WritePacketSerial sends the packet byte data over the Serial connection to the LoRa Module
    This function also calls the checksum function in order to 

    :param ser: Serial Object related to LoRa USB Connection
    :param packet: string or byte data
    :return: N/A
    """ 


    # Encode packet into bytes if it's a string
    if isinstance(packet, str):
        packet = packet.encode(FORMAT)
        packet = AddPacketChecksum(packet)
        ser.write(packet)
    if isinstance(packet, bytearray):
        packet = AddPacketChecksum(packet)
        ser.write(packet)
    return

# def PadPacket(data, type, limit):
#     padding = limit - data
#     if(type == "str"):
#         for i in range(numNullChars):
#             request_packet += "\0"
#     else:

def RetransmitPackets(ser, missed_pkts, data_stream):
    """
    RetransmitPackets will retransmit the packets that the Rx LoRa never received

    :param ser: Serial Object related to LoRa USB Connection
    :param missed_pkts: list of packet numbers that were never received
    :param data_stream: file object/file data
    :return: N/A
    """ 
    first_packet = True
    pkt_ID = 0
    for pkt in missed_pkts:
        pkt_front = ("P" + str(pkt_ID)).encode(FORMAT)
        if(first_packet == True):
            print("\nRetransmitting Packet: ", pkt)
            first_packet = False
        else:
            print("Retransmitting Packet: ", pkt)
        pkt_data = bytearray(pkt_front + data_stream[pkt*PKT_PAYLOAD:PKT_PAYLOAD+pkt*PKT_PAYLOAD])
        #Send Packet Data and wait for ack
        WritePacketSerial(ser, pkt_data)
        pkt_ID = (pkt_ID + 1) % 10
        SerialReadyAck(ser)
    return

def RequestMissingPackets(ser, missed_pkts, packets):
    """
    RequestMissingPackets will send packets via the Rx Lora to the Tx Lora in order to get retransmission of any lost packets, it will repeatedly do this as long as there is still missing packets.

    :param ser: Serial Object related to LoRa USB Connection
    :param missed_pkts: list, first index is amount of bytes read, second index has rssi total
    :param packets: 2d list of all received packets, each row being a received packet of PKT_SIZE
    :return: missing_pkt_info
    """ 

    missing_pkt_info = [0,0]
    time.sleep(TIMEOUT)

    #While Rx doesn't have all missed pkts
    while missed_pkts:
        expected_PKT_ID = 0
        request_packet = ""
        request_packet = ",".join(map(str, missed_pkts))
        if len(request_packet) > PKT_NO_CHECKSUM:
            request_packet = request_packet[:PKT_NO_CHECKSUM-1].rsplit(',', 1)[0]
        numNullChars = PKT_NO_CHECKSUM - len(request_packet)
        request_packet += "\0" * numNullChars
        print("Requesting Packets")
        print(missed_pkts)
        WritePacketSerial(ser, request_packet)
        SerialReadyAck(ser)

        #Wait for all packets
        print("Waiting for Packets...")
        # timeout_check = time.time()
        while True:
            timeout_check = time.time()
            read_in = ReadPacketSerial(ser)
            if not missed_pkts:
                print("Finished Transmission")
                break
            #Timeout
            if((time.time() - timeout_check) >= TIMEOUT):
                print("Requesting more packets")
                break
            if(read_in):
                #Extract PKT_HEADER
                packet_ID = int(read_in[1:2])
                read_in = bytearray(read_in[2:])
                if expected_PKT_ID != packet_ID:
                    #print("PACKET MISSED!!!")
                    missed_pkts.append(missed_pkts[0])
                    missed_pkts = missed_pkts[1:]
                    expected_PKT_ID = (expected_PKT_ID + 1) % 10
                missing_pkt_info[0] += len(read_in)
                packets[missed_pkts[0]] = read_in
                missed_pkts = missed_pkts[1:]
                #RSSI INFO EXTRACTION
                rssi = ser.read_until(expected= b':RSSI:')
                rssi = Sanitize_Serial(rssi)
                rssi = rssi.rstrip(':RSSI:')
                rssi = int(rssi)
                missing_pkt_info[1] += rssi
                expected_PKT_ID = (expected_PKT_ID + 1) % 10

    return missing_pkt_info




#Description:

def FileSendSerial(ser: serial.Serial, filename: str):
    """
    FileSendSerial Sends a file over serial to the connected LoRa Module, File can be binary(image, etc.) or string(txt), File information is sent in an initial header packet, All Packets are sent in PKT_SIZE chunks

    :param ser: Serial Object related to LoRa USB Connection
    :param filename: name of file to send
    :return: N/A
    """ 

    #This block is here in order to make sure that no junk_data is sitting idle on the serial
    #This avoids the root problem but should hopefully provide a long-term solution
    ser.reset_input_buffer()
    #This function might be unnecessary but I don't trust reset_input_buffer() enough atm
    ClearSerialInput(ser)

    # Extract filetype (txt, jpg, etc.)
    filetype = filename.split(".")
    filetype = filetype[1]
    # Collect information to send in 
    file_path = os.path.join(TEST_FILEPATH, filename)
    file_size = str(os.path.getsize(file_path))
    packet_header: str = filename + ":" + str(file_size) + ":"

    # missing packet error vars
    pkts_sent = 0
    pkt_ID = 0
    data_sent = 0

    f = open(file_path, mode = 'rb')
    file_data = f.read()
    #convert to bytearray
    file_byte_array = bytearray(file_data)
    file_bytes = len(file_byte_array)
    packet_header: str = filename + ":" + str(file_bytes) + ":"
    
    #find ceiling of number of packets
    packets: int = math.ceil(file_bytes / PKT_PAYLOAD)
    

    #send header packet with buffer, easier to not place overflow in initial pkt
    if(len(packet_header) < PKT_NO_CHECKSUM):
        # Create packet number buffer between checksum and filename/details
        packet_header = "\0" * PKT_NUMBER + packet_header
        # Calculate the number of null characters to add
        numNullChars = PKT_NO_CHECKSUM - len(packet_header)
        # Add null characters to the end of the string
        for i in range(numNullChars):
            packet_header += "\0"
            
    #Send initial packet header a certain amount of times to assure it arrives
    for i in range(INIT_PACKET_SENDS):
        
        WritePacketSerial(ser, packet_header)
        SerialReadyAck(ser)

        
    with tqdm(total=packets, desc="Sending: ", unit="packet", unit_scale=False) as progress_bar:
        for i in range(packets):
            pkt_front = ("P" + str(pkt_ID)).encode(FORMAT)
            #Last packet, pad if necessary
            if(i == packets - 1):
                #PADDING TO PKT_SIZE
                
                pkt = pkt_front + file_byte_array[i*PKT_PAYLOAD:file_bytes]
                pkt += b'\0' * (PKT_NO_CHECKSUM - len(pkt))
                pkt = bytearray(pkt)
                WritePacketSerial(ser, pkt)
                pkts_sent += 1
                SerialReadyAck(ser)

                #MISSING_PACKETS
                timeout_count = 0

                #Wait for our transmission success ack
                while True:
                    success_wait_timeout = time.time()
                    read_input = ReadPacketSerial(ser)
                    if(read_input):
                        read_input = Sanitize_Serial(read_input)
                        read_input = read_input.replace("\0","")
                        rssi = ser.read_until(expected= b':RSSI:')
                        #Success ACK
                        if(read_input == "success!"):
                            break
                        missed_pkts = [int(x) for x in read_input.split(",")]
                        RetransmitPackets(ser, missed_pkts, file_byte_array)
                    #If transmitter doesn't get missing packets or a success then finish transmission and move on
                    if((time.time() - success_wait_timeout) >= TIMEOUT):
                        timeout_count += 1
                    if(timeout_count == 5):
                        break
                progress_bar.update(1)
                break
            #Take packet data
            pkt = bytearray(pkt_front + file_byte_array[i*PKT_PAYLOAD:PKT_PAYLOAD+i*PKT_PAYLOAD])
            #Send Packet Data and wait for ack
            data_sent += len(pkt)
            WritePacketSerial(ser, pkt)
            pkt_ID = (pkt_ID + 1) % 10
            pkts_sent += 1
            SerialReadyAck(ser)
            #Loop pkt number counter back to 0, this allows only 1 byte to be used for pkt number

            #Update Progress Bar
            progress_bar.update(1)



def FileSendSerialGUI(ser: serial.Serial, filename: str, progress_label):
    """
    FileSendSerial Sends a file over serial to the connected LoRa Module, File can be binary(image, etc.) or string(txt), File information is sent in an initial header packet, All Packets are sent in PKT_SIZE chunks

    :param ser: Serial Object related to LoRa USB Connection
    :param filename: name of file to send
    :return: N/A
    """

    #This block is here in order to make sure that no junk_data is sitting idle on the serial
    #This avoids the root problem but should hopefully provide a long-term solution
    ser.reset_input_buffer()
    #This function might be unnecessary but I don't trust reset_input_buffer() enough atm
    ClearSerialInput(ser)

    # Extract filetype (txt, jpg, etc.)
    filetype = filename.split(".")
    filetype = filetype[1]
    # Collect information to send in 
    file_path = os.path.join(TEST_FILEPATH, filename)
    file_size = str(os.path.getsize(file_path))
    packet_header: str = filename + ":" + str(file_size) + ":"

    # missing packet error vars
    pkts_sent = 0
    pkt_ID = 0
    data_sent = 0

    f = open(file_path, mode = 'rb')
    file_data = f.read()
    #convert to bytearray
    file_byte_array = bytearray(file_data)
    file_bytes = len(file_byte_array)
    packet_header: str = filename + ":" + str(file_bytes) + ":"
    
    #find ceiling of number of packets
    packets: int = math.ceil(file_bytes / PKT_PAYLOAD)
    

    #send header packet with buffer, easier to not place overflow in initial pkt
    if(len(packet_header) < PKT_NO_CHECKSUM):
        # Create packet number buffer between checksum and filename/details
        packet_header = "\0" * PKT_NUMBER + packet_header
        # Calculate the number of null characters to add
        numNullChars = PKT_NO_CHECKSUM - len(packet_header)
        # Add null characters to the end of the string
        for i in range(numNullChars):
            packet_header += "\0"
            
    #Send initial packet header a certain amount of times to assure it arrives
    for i in range(INIT_PACKET_SENDS):
        
        WritePacketSerial(ser, packet_header)
        SerialReadyAck(ser)

        
    with tqdm(total=packets, desc="Sending: ", unit="packet", unit_scale=False) as progress_bar:
        for i in range(packets):
            pkt_front = ("P" + str(pkt_ID)).encode(FORMAT)
            #Last packet, pad if necessary
            if(i == packets - 1):
                #PADDING TO PKT_SIZE
                
                pkt = pkt_front + file_byte_array[i*PKT_PAYLOAD:file_bytes]
                pkt += b'\0' * (PKT_NO_CHECKSUM - len(pkt))
                pkt = bytearray(pkt)
                WritePacketSerial(ser, pkt)
                pkts_sent += 1
                progress_string = "Tx %%: %.1f" % (pkts_sent / packets * 100)
                progress_label.setText(progress_string)
                SerialReadyAck(ser)

                #MISSING_PACKETS
                timeout_count = 0

                #Wait for our transmission success ack
                while True:
                    success_wait_timeout = time.time()
                    read_input = ReadPacketSerial(ser)
                    if(read_input):
                        read_input = Sanitize_Serial(read_input)
                        read_input = read_input.replace("\0","")
                        rssi = ser.read_until(expected= b':RSSI:')
                        #Success ACK
                        if(read_input == "success!"):
                            break
                        missed_pkts = [int(x) for x in read_input.split(",")]
                        progress_label.setText("Retransmit...")
                        RetransmitPackets(ser, missed_pkts, file_byte_array)
                    #If transmitter doesn't get missing packets or a success then finish transmission and move on
                    if((time.time() - success_wait_timeout) >= TIMEOUT):
                        timeout_count += 1
                    if(timeout_count == 5):
                        break
                progress_bar.update(1)
                break
            #Take packet data
            pkt = bytearray(pkt_front + file_byte_array[i*PKT_PAYLOAD:PKT_PAYLOAD+i*PKT_PAYLOAD])
            #Send Packet Data and wait for ack
            data_sent += len(pkt)
            WritePacketSerial(ser, pkt)
            pkt_ID = (pkt_ID + 1) % 10
            pkts_sent += 1
            SerialReadyAck(ser)
            #Loop pkt number counter back to 0, this allows only 1 byte to be used for pkt number

            #Update Progress Bar
            progress_bar.update(1)


def BytesSendSerial(ser: serial.Serial, filename: str, buffer: io.BytesIO):
    """
    BytesSendSerial Sends a file over serial to the connected LoRa Module, File can be binary(image, etc.) or string(txt), File information is sent in an initial header packet, All Packets are sent in PKT_SIZE chunks

    :param ser: Serial Object related to LoRa USB Connection
    :param filename: name of file to send
    :return: N/A
    """ 

    #This block is here in order to make sure that no junk_data is sitting idle on the serial
    #This avoids the root problem but should hopefully provide a long-term solution
    ser.reset_input_buffer()
    #This function might be unnecessary but I don't trust reset_input_buffer() enough atm
    ClearSerialInput(ser)

    # Extract filetype (txt, jpg, etc.)
    filetype = filename.split(".")
    filetype = filetype[1]
    # Collect information to send in 
    file_size = str(buffer.tell())
    packet_header: str = filename + ":" + str(file_size) + ":"

    # missing packet error vars
    pkts_sent = 0
    pkt_ID = 0
    data_sent = 0

    buffer_contents = buffer.getvalue()
    #convert to bytearray
    file_byte_array = bytearray(buffer_contents)
    file_bytes = len(file_byte_array)
    packet_header: str = filename + ":" + str(file_bytes) + ":"
    
    #find ceiling of number of packets
    packets: int = math.ceil(file_bytes / PKT_PAYLOAD)
    

    #send header packet with buffer, easier to not place overflow in initial pkt
    if(len(packet_header) < PKT_NO_CHECKSUM):
        # Create packet number buffer between checksum and filename/details
        packet_header = "\0" * PKT_NUMBER + packet_header
        # Calculate the number of null characters to add
        numNullChars = PKT_NO_CHECKSUM - len(packet_header)
        # Add null characters to the end of the string
        for i in range(numNullChars):
            packet_header += "\0"
            
    #Send initial packet header a certain amount of times to assure it arrives
    for i in range(INIT_PACKET_SENDS):
        
        WritePacketSerial(ser, packet_header)
        SerialReadyAck(ser)

        
    with tqdm(total=packets, desc="Sending: ", unit="packet", unit_scale=False) as progress_bar:
        for i in range(packets):
            pkt_front = ("P" + str(pkt_ID)).encode(FORMAT)
            #Last packet, pad if necessary
            if(i == packets - 1):
                #PADDING TO PKT_SIZE
                
                pkt = pkt_front + file_byte_array[i*PKT_PAYLOAD:file_bytes]
                pkt += b'\0' * (PKT_NO_CHECKSUM - len(pkt))
                pkt = bytearray(pkt)
                WritePacketSerial(ser, pkt)
                pkts_sent += 1
                SerialReadyAck(ser)

                #MISSING_PACKETS
                timeout_count = 0

                #Wait for our transmission success ack
                while True:
                    success_wait_timeout = time.time()
                    read_input = ReadPacketSerial(ser)
                    if(read_input):
                        read_input = Sanitize_Serial(read_input)
                        read_input = read_input.replace("\0","")
                        rssi = ser.read_until(expected= b':RSSI:')
                        #Success ACK
                        if(read_input == "success!"):
                            break
                        missed_pkts = [int(x) for x in read_input.split(",")]
                        RetransmitPackets(ser, missed_pkts, file_byte_array)
                    #If transmitter doesn't get missing packets or a success then finish transmission and move on
                    if((time.time() - success_wait_timeout) >= TIMEOUT):
                        timeout_count += 1
                    if(timeout_count == 5):
                        break
                progress_bar.update(1)
                break
            #Take packet data
            pkt = bytearray(pkt_front + file_byte_array[i*PKT_PAYLOAD:PKT_PAYLOAD+i*PKT_PAYLOAD])
            #Send Packet Data and wait for ack
            data_sent += len(pkt)
            WritePacketSerial(ser, pkt)
            pkt_ID = (pkt_ID + 1) % 10
            pkts_sent += 1
            SerialReadyAck(ser)
            #Loop pkt number counter back to 0, this allows only 1 byte to be used for pkt number

            #Update Progress Bar
            progress_bar.update(1)

def FileSaveSerial(ser: serial.Serial, filename: str, filesize: int, spreadfactor, bandwidth, init_pkt, distance):
    """
    FileSaveSerial will attempt to receive and save an entire file over LoRa via a Serial connection to a LoRa module

    :param ser: Serial Object related to LoRa USB Connection
    :param filename: name of expected file
    :param filesize: size of expected file
    :param spreadfactor: sf for data recording
    :param bandwidth: bw for data recording
    :param init_pkt: the intial header packet
    :param distance: distance between LoRa devices
    :return: N/A
    """ 

    #Missing packet vars
    expected_PKT_ID = 0
    pkt_rotations = 0
    missed_pkts = []
    #Used to hold each packet until full transmission
    packets = []
    #Kinda messy but whatever, used to strip final packet of padding
    final_packet = math.ceil(filesize / PKT_PAYLOAD)
    #File transmission stats
    missing_data = 0
    pkt_receive_time = 0
    final_pkt_time = 0
    start_time = 0
    rssiTotal = 0
    original_filesize = filesize
    no_progress_bar = False
    failed_retransmit = False

    start_time = time.time()
    with tqdm(total = filesize, unit="bytes") as progress_bar:
        while final_packet:
            t = time.time()
            #Get RSSI Data
            rssi = ser.read_until(expected= b':RSSI:')
            rssi = Sanitize_Serial(rssi)
            rssi = rssi.rstrip(':RSSI:')
            rssi = int(rssi)
            progress_bar.set_postfix({"RSSI": rssi})
            rssiTotal += rssi
            #Read in data from serial
            read_in = ReadPacketSerial(ser)
            if(read_in == init_pkt):
                start_time = time.time()
                #Skip to next iteration, got a duplicate initial packet
                continue
            #timeout handler
            if((time.time() - t) >= TIMEOUT):
                #If the final packet is never received
                no_progress_bar = True
                progress_bar.close()
                missing_data = PKT_PAYLOAD * len(missed_pkts)
                print("Missing Data at Timeout: ", missing_data , " bytes")
                missing_percentage = (missing_data / original_filesize) * 100
                print("Missing Data Percentage: ", round(missing_percentage,2), "%")
                if not final_pkt_time:
                    print("Final packet never received, adding to missed pkts")
                    packets.append([])
                    missed_pkts.append(final_packet - 1)
                if(missed_pkts):
                    missing_data = len(missed_pkts) * PKT_PAYLOAD
                    final_pkt_time = pkt_receive_time - start_time
                    try:
                        missing_pkt_info = RequestMissingPackets(ser, missed_pkts, packets)
                        new_data_read = missing_pkt_info[0]
                        rssiTotal += missing_pkt_info[1]
                        filesize -= new_data_read
                        expected_packets = 0
                        break
                    except:
                        print("Couldn't handle missing packet requests, saving anyway...")
                        failed_retransmit = True
                        break
                else:
                    print("Something went wrong...")


            #If packet isn't empty then add it to binary data
            #Technically this if statement isn't needed
            if(read_in):
                pkt_receive_time = time.time()
                

                #Extract PKT HEADER and PAYLOAD
                packet_ID = Sanitize_Serial(read_in[1:PKT_NUMBER])
                packet_ID = int(packet_ID)
                pkt_number = packet_ID + 10 * pkt_rotations
                #If last packet remove padding
                if(pkt_number == final_packet - 1):
                    missing_data = filesize
                    final_pkt_time = pkt_receive_time - start_time
                    #print("Stripping padding from final packet...")
                    read_in = read_in.rstrip(b'\x00')
                    if not missed_pkts:
                        read_in = bytearray(read_in[PKT_NUMBER:])
                        pkt_len = len(read_in)
                        packets.append(read_in)
                        filesize -= pkt_len
                        progress_bar.update(pkt_len)
                        missing_data = filesize
                        break


                read_in = bytearray(read_in[PKT_NUMBER:])
                pkt_len = len(read_in)

                #Missing packet handling
                while expected_PKT_ID != packet_ID:
                    #print("PACKET MISSED!!!")
                    packets.append([])
                    pkt_number = expected_PKT_ID + 10 * pkt_rotations
                    missed_pkts.append(pkt_number)
                    expected_PKT_ID = (expected_PKT_ID + 1) % 10
                    pkt_rotations += expected_PKT_ID == 0

                packets.append(read_in)
                filesize -= pkt_len
                if(no_progress_bar == False):
                    progress_bar.update(pkt_len)
                

                expected_PKT_ID = (expected_PKT_ID + 1) % 10
                pkt_rotations += expected_PKT_ID == 0
    
    rssiTotal = rssiTotal / final_packet
    combined_bytearray = bytearray()
    for pkt in packets:
        combined_bytearray.extend(pkt)
    combined_bytearray = bytearray(combined_bytearray)            
    data_decoded = combined_bytearray
    file_path = os.path.join(TEST_FILEPATH, filename)  
    f = open(file_path, "wb")
    f.write(data_decoded)
    f.close()

    data_filename = "FileTransmission.txt"
    data_fullpath = os.path.join(RX_FILEPATH,data_filename)
    file_size_time = time.time() - start_time
    data_file = open(data_fullpath, "a")
    Data = "Missed Bytes: " + str(missing_data) + ", First Time: " + str(final_pkt_time) + ", Final Time: " + str(file_size_time) + ", RSSI Avg: " + str(rssiTotal) + ", SF: " + str(spreadfactor)+ ", BW: " + str(bandwidth) + ", Distance: " + str(distance) + ", Size: " + str(filesize) + ", File: " + filename + "\n"
    data_file.write(Data)
    data_file.close()

    if not failed_retransmit:
        #Send receive ack
        rssi = ser.read_until(expected= b':RSSI:')
        rssi = Sanitize_Serial(rssi)
        rssi = rssi.rstrip(':RSSI:')
    else:
        ser.reset_input_buffer()
    #
    success_ack = "success!"
    success_ack += "\0" * (PKT_NO_CHECKSUM - 8)
    WritePacketSerial(ser, success_ack)
    SerialReadyAck(ser)
    
    print("Average RSSI: ", rssiTotal)
    
    return
