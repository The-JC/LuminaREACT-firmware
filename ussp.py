import serial
import time
import struct
from dataclasses import dataclass
from enum import Enum
from threading import Thread
from util import *
import time

START_BYTE = 0x7E
END_BYTE = 0x7F
ESCAPE_BYTE = 0x7D

MAX_PAYLOAD_SIZE = 255
USSP_PACKAGE_OVERHEAD = 5
MAX_PACKAGAGE_SIZE = USSP_PACKAGE_OVERHEAD + MAX_PAYLOAD_SIZE

class USSP_PayloadType(Enum):
    STR_ERROR   = 0x01
    STR_INFO    = 0x02
    STR_DEBUG   = 0x03
    MESSAGE		= 0x80

@dataclass
class Packet:
    def __init__(self, payloadType, payload: bytes):
        self.length = len(payload)
        self.payloadType = payloadType
        self.payload = payload
    
    def encode(self) -> bytes:

        # Pack data 
        if type(self.payloadType) == USSP_PayloadType:
            payloadType = self.payloadType.value
        else:
            payloadType = self.payloadType

        encoded_data = bytearray()
        encoded_data.append(START_BYTE)

        # Check if length byte needs to be escaped
        if self.length in (START_BYTE, END_BYTE, ESCAPE_BYTE):
            encoded_data += struct.pack('BB', ESCAPE_BYTE, len(self.payload) ^ 0x20) 
        else:
            encoded_data.append(len(self.payload))

        # Check if payload type byte needs to be escaped
        if payloadType in (START_BYTE, END_BYTE, ESCAPE_BYTE):
            encoded_data += struct.pack('BB', ESCAPE_BYTE, payloadType ^ 0x20)
        else:
            encoded_data.append(payloadType)

        # If the payloads instance is a string encode it for further processing
        if isinstance(self.payload, str):
            payload = self.payload.encode()
        else:
            payload = self.payload

        # Go trough the whole payload to append it to the data package and possible escape bytes
        for byte in payload:
            if byte in (START_BYTE, END_BYTE, ESCAPE_BYTE):
                encoded_data += struct.pack('BB', ESCAPE_BYTE, byte ^ 0x20)
            else:
                encoded_data += struct.pack('B', byte)

        # Calculate checksum
        empty_bytes = (4 - (len(payload)+1) % 4) % 4
        payloadCRC = bytearray(payloadType.to_bytes() + payload) + bytearray(empty_bytes)
        crc32 = process_queue(payloadCRC) & 0xFF
        if crc32 in (START_BYTE, END_BYTE, ESCAPE_BYTE):
            encoded_data += struct.pack('BB', ESCAPE_BYTE, crc32 ^ 0x20)
        else:
            encoded_data.append(crc32)

        encoded_data.append(END_BYTE)
        return encoded_data
    
    def __str__(self):
        str = "Datatype: "
        if(self.payloadType == USSP_PayloadType.MESSAGE.value):
            str += "Message\n"
            str += self.payload.decode()
        elif (self.payloadType & 0xF0 ) == 0xF0:
            frame = self.payloadType & 0x0F
            str += "FFT Frame No. " + frame + " "
        else:
            str += hexToString([self.payloadType])
            str += "\n"
            str += hexToString(self.payload)

        return str

class USSP:
    def __init__(self, serialPort = '/dev/ttyUSB0', serialBaud = 38400):
        self.port = serialPort
        self.baud = serialBaud

        self.thread         = None
        self.isReceiving    = False
        self.isRun          = True

        # Connect to serial port
        print('Trying to connect to: ' + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')
        try:
            self.serialConnection = serial.Serial(serialPort, serialBaud, timeout=4)
            print('Connected to ' + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')
        except:
            print("Failed to connect with " + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')
            exit()

        # self.serialConnection.write(100)

    def sendMessage(self, str):
        try:
            packet = Packet(USSP_PayloadType.MESSAGE, str.encode())

            # Encode the packet
            encoded_packet = packet.encode()
            print("Encoded Packet:")
            printHex(encoded_packet)

            # Send the packet over serial
            self.serialConnection.write(encoded_packet)

        except ValueError as e:
            print("Error:", e)

    def readSerialStart(self):
        if self.thread == None:
            self.thread = Thread(target=self.backgroundThread)
            self.thread.start()
            # Block till we start receiving values
            i=0
            while self.isReceiving != True:
                time.sleep(0.1)
                # str = "Hello world %d\n" % (i)
                # bin = Packet(USSP_PayloadType.MESSAGE, str).encode()
                # packet = self.decode_packet(bin)
                # print("Decoded Packet:", packet)
                i+=1

    def backgroundThread(self):    # retrieve data
        time.sleep(1)  # give some buffer time for retrieving data

        buffer = bytearray()
        self.serialConnection.reset_input_buffer()
        while (self.isRun):
            # self.serialConnection.readinto(self.rawData)
            # self.isReceiving = True
            #print(self.rawData)

            if self.serialConnection.in_waiting > USSP_PACKAGE_OVERHEAD +1:
                # Read available data from the serial port
                data = self.serialConnection.read(self.serialConnection.in_waiting)
                buffer.extend(data)

                # Process buffer to extract packets
                while True:
                    try:
                        # Try to decode the packet
                        start = time.time()
                        packet = self.decode_packet(buffer)
                        print("Decoded Packet:", packet)
                        end = time.time()
                        print(end - start)

                        # Remove processed packet from buffer
                        end_index = buffer.index(END_BYTE) + 1
                        buffer = buffer[end_index:]
                        print("Buffer len ", len(buffer))
                    except ValueError:
                        # Not enough data or invalid packet, wait for more
                        break

    def find_start_byte(self, data: bytes) -> int:
        """Finds the first occurance of the start byte in the data and returns the indice otherwise -1
        :rtype: int
        :param: data: datapackage to find start byte
        :return: start byte indice or -1 if data doesn't contain start byte
        """
        try:
            return data.index(START_BYTE)
        except ValueError:
            return -1
        
    def find_end_byte(self, data: bytes) -> int:
        """Finds the first occurance of the end byte in the data and returns the indice otherwise -1
        :rtype: int
        :param: data: datapackage to find end byte
        :return: end byte indice or -1 if data doesn't contain end byte
        """
        try:
            return data.index(END_BYTE)
        except ValueError:
            return -1

    def close(self):
        self.isRun = False
        self.thread.join()
        self.serialConnection.close()
        print('Disconnected...')
        # df = pd.DataFrame(self.csvData)
        # df.to_csv('/home/rikisenia/Desktop/data.csv')
        
    
    def decode_packet(self, encoded_data: bytes) -> Packet:
        # Unpack data
        if len(encoded_data) < 4:
            raise ValueError("Encoded data is too short.")
        
        # Try to find the start byte otherwise throw an error
        start_index = self.find_start_byte(encoded_data)
        if start_index == -1:
            raise ValueError("Start byte not found in the data.")
        
        # Try to find the start byte otherwise throw an error
        end_index = self.find_end_byte(encoded_data)
        if end_index == -1:
            raise ValueError("End byte not found in the data.")
    
        i = start_index

        # Extract length (1 byte) using struct
        if len(encoded_data) < i + 2:
            raise ValueError("Data length too short after start byte.")
        
        # Validate start byte
        start_byte = encoded_data[i]
        if start_byte != START_BYTE:
            raise ValueError("Invalid start byte.")
        i += 1

        # Decode the length byte
        length = encoded_data[i]
        if length == ESCAPE_BYTE:
            i+=1
            length = encoded_data[i] ^ 0x20
        i+=1
        
        # Decode the payload type byte
        payloadType = encoded_data[i]
        if payloadType == ESCAPE_BYTE:
            i+=1
            payloadType = encoded_data[i] ^ 0x20
        i+=1

        escape = False
        decoded_data = bytearray()

        # Decode the payload
        while i < end_index - 1:
            byte = encoded_data[i]
            if escape:
                decoded_data.append(byte ^ 0x20)
                escape = False
            elif byte == ESCAPE_BYTE:
                escape = True
            else:
                decoded_data.append(byte)
            i += 1

        # Validate decoded data length matches advertised packet length
        if len(decoded_data) != length:
            raise ValueError("Decoded length doesn't match packet length.")

        checksum = encoded_data[i]
        if escape == True:
            checksum = checksum ^ 0x20
        i+=1

        end_byte = encoded_data[i]

        if end_byte != END_BYTE:
            raise ValueError("Invalid end byte.")

        # Calculate CRC32 and compare to packet CRC
        empty_bytes = (4 - (length+1) % 4) % 4 # Byte padding for crc
        payloadCRC = bytearray(payloadType.to_bytes() + decoded_data) + bytearray(empty_bytes)
        crc32 = process_queue(payloadCRC)
        if checksum != (crc32) & 0xFF:
            raise ValueError("Checksum does not match.")

        return Packet(payloadType, decoded_data)

def main():
    ussp = USSP('COM3', 115200)
    ussp.readSerialStart()


if __name__ == '__main__':
    main()