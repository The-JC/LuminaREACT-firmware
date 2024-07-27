from crccheck import crc

def hexToString(array):
    return ", ".join(hex(b) for b in array)

def printHex(array):
    print(hexToString(array))

_debug_mode = False

def _debug_msg(a, b) -> None:
    if _debug_mode:
        print(a, b)

def msb(val: int, bits: int = 8) -> bool:
    """returns whether the Most Significant Bit (msb) of the provided value is 1
    :rtype: bool
    :param val: numeric value to check the most msb
    :param bits: bit length of the value
    :return: true if the msb is a 1, false otherwise
    """
    return bool(val & (1 << (bits - 1)))


def embedded_crc(input_data: int, poly: int = 0x04C11DB7, initial_crc: int = 0xFFFFFFFF, sizeof_input: int = 32) -> int:
    """Calculates the crc for one given integer. This function represents one iteration on the embedded crc peripheral
    in the STM controller family. (The parameter default values equal the ones embedded in the STM hardware modules.)
    :rtype: int
    :param input_data: one integer value with the specified bit length
    :param poly: crc polynomial to use in the algorithm
    :param initial_crc: crc value to start with (e.g. value from the previous iteration)
    :param sizeof_input: bit length of the desired crc value
    :return: crc value of the given integer
    """
    # create the proper mask
    msk = 0
    for i in range(sizeof_input):
        msk = (msk << 1) | 1

    # start of the algorithm described in the stm32 crc application manual
    _debug_msg("initial_crc", bin(initial_crc))
    _debug_msg("input_data", bin(input_data))

    crc = initial_crc ^ input_data
    _debug_msg("crc = initial ^ input", bin(crc))

    b_index = 0
    while b_index < sizeof_input:

        _debug_msg("crc", bin(crc))

        if msb(crc, sizeof_input):
            crc = ((crc << 1) ^ poly) & msk
            _debug_msg("crc ^ poly", bin(crc))
        else:
            crc = (crc << 1) & msk

        b_index += 1

    return crc

def prepare_data(data: [int]) -> [int]:
    """Convert a string into a list of integers. (Uses utf-8 encoding on most machines.)
    :rtype: [int]
    :param data: string to convert
    :return: list of the int representations of the  chars from the string
    """
    reduced_data = []
    for i in range(0, int(len(data)/4)):
        tmp = data[i*4]
        tmp |= data[i*4+1] << 8
        tmp |= data[i*4+2] << 16
        tmp |= data[i*4+3] << 24
        reduced_data.append(tmp)
    return reduced_data


def process_queue(data, poly: int = 0x04C11DB7, initial_crc: int = 0xFFFFFFFF, sizeof_input: int = 32) -> int:
    """Calculates the crc for a string. (The parameter default values equal the ones embedded in the STM hardware
    modules.)
    :rtype: int
    :param data: the crc value is calculated for this string
    :param poly: crc polynomial to use in the algorithm
    :param initial_crc: crc value to start with
    :param sizeof_input: bit length of the desired crc value
    :return: crc value of the provided string
    """
    crc = initial_crc
    data = prepare_data(data)
    for val in data:
        crc = embedded_crc(val, poly, crc, sizeof_input)
    return crc