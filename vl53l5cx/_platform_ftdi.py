import time

from typing import List


DEBUG_IO = False
PRINT_SIZE_MAX = 1024

VL53L5CX_COMMS_CHUNK_SIZE = 4096  # Mark's original value was 1024, but 4096 works as well
    
class I2CAPI:
    """
    Implementation to read/write via PYFTDI
    """
    def __init__(self, i2c_bus, bus_id, frequency=1000000):
        if i2c_bus is None:
            raise ValueError("You need to specify the FTDI URL as i2c_bus argument")
        import pyftdi as pyftdi
        import pyftdi.i2c
        
        #from _pyftdi import i2c as _i2c
        self.i2c_bus = pyftdi.i2c.I2cController()
        self.i2c_bus.configure(i2c_bus, frequency=frequency)
        self.frequency = frequency
        self.set_latency(10)
        
        #VL53L5CX interface setup:
        self.i2cport = self.i2c_bus.get_port(bus_id)  
        self.i2cport.configure_register(bigendian=True, width=2) #uses two bytes as register index, little endian
        
        #do a quick check if device is connected:
        #try:
        #    self.i2cport.read(0)
        #except pyftdi.i2c.I2cNackError as e:
        #    raise pyftdi.i2c.I2cNackError(f"No device is responding at bus id {bus_id}!")

    def set_latency(self, ms):
        """
        set how many ms to wait before flushing the usb buffer
        """
        self.i2c_bus.ftdi.set_latency_timer(ms)
    

    def rd_multi(self, addr: int, buffer: List[int], size: int) -> None:
        bytes = self.i2cport.read_from(addr, readlen=size)
        for i, b in enumerate(bytes):
            buffer[i] = b
        if DEBUG_IO:
            print(f"read_from addr={addr:#0{6}x}, size={size}, data={buffer[:size]}", end="\n")

    def wr_multi(self, addr: int, buffer: List[int], size: int) -> None:
             
        PAYLOAD_MAX = VL53L5CX_COMMS_CHUNK_SIZE - 2
        if size > PAYLOAD_MAX: #send the FW in (large) chunks
            for startindex in range(0, size, PAYLOAD_MAX):
                totalremaining  = size - startindex
                if totalremaining > PAYLOAD_MAX:
                    chunksize = PAYLOAD_MAX 
                else:
                    chunksize = totalremaining
                if DEBUG_IO: print(f"{startindex}, {chunksize}, {totalremaining}")
                self.wr_multi(addr+startindex, buffer[startindex:startindex+chunksize], chunksize)
        else: 
            data = bytearray(buffer[:size])
            b = self.i2cport.write_to(addr, data)

        if DEBUG_IO:
            print(f"write_to addr={addr:#0{6}x}, data={data}", end="\n")

    def rd_byte(self, addr: int) -> int:
        b = self.i2cport.read_from(addr, readlen=1)
        value = int.from_bytes(b, 'big')
        if DEBUG_IO:
            print(f"read_from addr={addr:#0{6}x}, byte={value:#0{4}x}", end="\n")
        return value

    def wr_byte(self, addr: int, value: int) -> None:
        b = self.i2cport.write_to(addr, value.to_bytes(1, byteorder='big'))
        if DEBUG_IO:
            print(f"write_to addr={addr:#0{6}x}, byte={value:#0{4}x}", end="\n")
    
