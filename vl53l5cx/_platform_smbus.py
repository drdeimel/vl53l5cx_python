import time

from typing import List


DEBUG_IO = False
PRINT_SIZE_MAX = 1024

VL53L5CX_COMMS_CHUNK_SIZE = 4096  # Mark's original value was 1024, but 4096 works as well



class I2CAPI:
    """
    implementation to read/write via SMBUS2
    """
    def __init__(self, i2c_bus, bus_id):
        if i2c_bus is None:
            from smbus2 import SMBus, i2c_msg
            self.i2c_msg = i2c_msg
            i2c_bus = SMBus(bus_id)
            self.i2c_bus = i2cbus
    
    
    def rd_multi(self, addr: int, buffer: List[int], size: int) -> None:
        write_addr = self.i2c_msg.write(self.i2c_address, [addr >> 8 & 0xff, addr & 0xff])
        read_data = self.i2c_msg.read(self.i2c_address, size)

        self.i2c_bus.i2c_rdwr(write_addr, read_data)

        read_size = len(read_data)

        if read_size > 0:
            read_buffer = list(read_data)
            buffer[:read_size] = read_buffer[:read_size]
            if DEBUG_IO:
                print(f"rd_multi addr={addr:#0{6}x}, len={{}}, size={size}. read_size={read_size}, buf_len= read=[", end="")
                print_size = read_size
                if print_size > PRINT_SIZE_MAX:
                    print_size = PRINT_SIZE_MAX
                for i in range(print_size):
                    if i > 0:
                        print(", ", end="")
                    print(f"{read_buffer[i]:#0{2}x}", end="")
                if print_size != read_size:
                    print(", ...", end="")
                print("]")
        else:
            raise Exception("Couldn't read any bytes")

    def wr_multi(self, addr: int, buffer: List[int], size: int) -> None:
        position = 0
        while position < size:
            data_size = VL53L5CX_COMMS_CHUNK_SIZE - 2 if size - position > VL53L5CX_COMMS_CHUNK_SIZE - 2 else size - position

            buf = [0] * (data_size + 2)
            buf[0] = addr >> 8
            buf[1] = addr & 0xff
            buf[2:] = buffer[position:position + data_size]
            write = self.i2c_msg.write(self.i2c_address, buf)
            self.i2c_bus.i2c_rdwr(write)

            if DEBUG_IO:
                print(f"wr_multi addr={addr:#0{6}x}, len={{}}, size={size}. write_size={data_size} [", end="")
                print_size = data_size
                if print_size > PRINT_SIZE_MAX:
                    print_size = PRINT_SIZE_MAX
                for i in range(print_size):
                    if i > 0:
                        print(", ", end="")
                    print(f"{buffer[position + i]:#0{2}x}", end="")
                if print_size != data_size:
                    print(", ...", end="")
                print("]")
            addr += data_size
            position += data_size

    def rd_byte(self, addr: int) -> int:
        write_addr = self.i2c_msg.write(self.i2c_address, [addr >> 8 & 0xff, addr & 0xff])
        read_data = self.i2c_msg.read(self.i2c_address, 1)
        self.i2c_bus.i2c_rdwr(write_addr, read_data)
        b = read_data.buf[0][0]
        if DEBUG_IO:
            print(f"rd_byte addr={addr:#0{6}x}, byte={b:#0{4}x}")
        return b

    def wr_byte(self, addr: int, value: int) -> None:
        write_addr_and_value = self.i2c_msg.write(self.i2c_address, [addr >> 8 & 0xff, addr & 0xff, value])
        self.i2c_bus.i2c_rdwr(write_addr_and_value)
        if DEBUG_IO:
            print(f"wr_byte addr={addr:#0{6}x}, byte={value:#0{4}x}")
    
    
class _I2CAPI_FTDI:
    """
    Implementation to read/write via PYFTDI
    """
    def __init__(self, i2c_bus, bus_id):
        if i2c_bus is None:
            raise ValueError("You need to specify the FTDI URL as i2c_bus argument")
        import pyftdi as pyftdi
        import pyftdi.i2c
        #from _pyftdi import i2c as _i2c
        self.i2c_bus = pyftdi.i2c.I2cController()
        self.i2c_bus.configure(i2c_bus, frequency=250000)
        self.i2c_bus.ftdi.set_latency_timer(10)

        #VL53L5CX interface setup:
        self.i2cport = self.i2c_bus.get_port(bus_id)  
        self.i2cport.configure_register(bigendian=True, width=2) #uses two bytes as register index, little endian
        
        #do a quick check if device is connected:
        #try:
        #    self.i2cport.read(0)
        #except pyftdi.i2c.I2cNackError as e:
        #    raise pyftdi.i2c.I2cNackError(f"No device is responding at bus id {bus_id}!")


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
                print(f"{startindex}, {chunksize}, {totalremaining}")
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
    
