import time
import can
import logging

# 设置日志级别为INFO，获取日志记录器实例
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

# # 创建一个文件处理器，用于将日志写入文件
# file_handler = logging.FileHandler('can_bus_operator_log.txt')
# file_handler.setLevel(logging.DEBUG)

# # 创建一个日志格式
# log_format = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
# file_handler.setFormatter(log_format)

# # 将文件处理器添加到日志记录器
# logger.addHandler(file_handler)

console_handler = logging.StreamHandler()

# 设置处理程序的日志级别为 INFO
console_handler.setLevel(logging.INFO)
logger.addHandler(console_handler)

# 定义指令类型常量
READ_REGISTER = 0b00
WRITE_REGISTER = 0b01
RESERVED = 0b10
MANAGEMENT_COMMAND = 0b11

# 定义寄存器数量常量
INVALID_COUNT = 0b00
ONE_REGISTER = 0b01
TWO_REGISTERS = 0b10
THREE_REGISTERS = 0b11

WAIT_TIME = 2 # 延迟打印，方便查看

def setup_can_bus(interface='pcan', channel='PCAN_USBBUS1', bitrate=1000000):
    """
    初始化 CAN 总线连接
    :param interface: CAN 接口类型
    :param channel: CAN 通道
    :param bitrate: CAN 总线波特率
    :return: CAN 总线对象，如果连接失败则返回 None
    """
    try:
        bus = can.interface.Bus(
            interface=interface,
            channel=channel,
            bitrate=bitrate
        )
        logger.info(f"Successfully connected to CAN bus on {interface}:{channel} with bitrate {bitrate}\n")
        return bus
    except can.CanError as e:
        logger.error(f"\nError connecting to CAN bus: {e}\n")
        return None

def close_can_bus(bus):
    """
    关闭 CAN 总线连接
    :param bus: CAN 总线对象
    """
    if bus is not None:
        try:
            bus.shutdown()
            logger.info(f"\nCAN bus connection closed.\n")
        except can.CanError as e:
            logger.error(f"\nError closing CAN bus connection: {e}\n")

def build_request_frame(command_type, register_count, start_address, data=None):
    """
    构建主机请求帧
    :param command_type: 指令类型
    :param register_count: 寄存器数量
    :param start_address: 起始寄存器地址
    :param data: 写入的数据（写操作时使用）
    :return: 请求帧字节列表
    """
    byte0 = (command_type << 6) | (register_count << 4) | ((start_address >> 8) & 0x0F)
    byte1 = start_address & 0xFF
    frame = [byte0, byte1]
    if command_type == WRITE_REGISTER and data:
        frame.extend(data)
    return frame

def send_can_message(bus, arbitration_id, data):
    """
    发送 CAN 消息
    :param bus: CAN 总线对象
    :param arbitration_id: 仲裁 ID
    :param data: 消息数据
    :return: 发送是否成功
    """
    try:
        msg = can.Message(
            arbitration_id=arbitration_id,
            data=data,
            is_extended_id=False
        )
        bus.send(msg)
        logger.info("CAN message sent successfully.")
        return True
    except can.CanError as e:
        logger.error(f"Error sending CAN message: {e}")
        return False

def receive_can_message(bus, timeout=1.0):
    """
    接收 CAN 消息
    :param bus: CAN 总线对象
    :param timeout: 超时时间
    :return: 接收到的消息，如果超时则返回 None
    """
    try:
        msg = bus.recv(timeout=timeout)
        if msg is not None:
            logger.info("CAN message received successfully.")
        return msg
    except can.CanError as e:
        logger.error(f"Error receiving CAN message: {e}")
        return None

def read_registers(bus, arbitration_id=2, start_address=None, register_count=ONE_REGISTER):
    """
    读取寄存器
    :param bus: CAN 总线对象
    :param arbitration_id: 仲裁 ID
    :param start_address: 起始寄存器地址
    :param register_count: 寄存器数量
    :return: 成功时返回寄存器的值列表，失败时返回错误码元组 (错误代码, 子错误代码)
    """
    if start_address is None:
        raise ValueError("起始地址不能为 None")
    request_frame = build_request_frame(READ_REGISTER, register_count, start_address)
    if send_can_message(bus, arbitration_id, request_frame):
        response_msg = receive_can_message(bus)
        if response_msg:
            response_data = response_msg.data
            byte0 = response_data[0]
            bit7_6 = (byte0 & 0xC0) >> 6
            bit5_4 = (byte0 & 0x30) >> 4
            received_start_address = ((byte0 & 0x0F) << 8) | response_data[1]
            logger.info(f"bit7_6: {bit7_6}, bit5_4: {bit5_4}, received_start_address: {int(received_start_address)}")
            for i, byte in enumerate(response_data):
                logger.info(f"Byte {i}: {bin(byte)}")
            if bit7_6 == READ_REGISTER:
                if bit5_4 == INVALID_COUNT:  # 错误应答
                    error_code = byte0 & 0x0F
                    sub_error_code = response_data[1]
                    logger.error(f'error_code={error_code}, sub_error_code={sub_error_code}')
                    return None
                else:  # 正常应答
                    if received_start_address == start_address:
                        start_index = 2
                        end_index = start_index + bit5_4*2
                        # 检查切片范围是否在 response_data 的有效范围内
                        values = response_data[start_index:end_index]
                        combined_value = 0
                        # 按字节组合成整数，后面的字节为高位
                        num_bytes = len(values)
                        for i, byte in enumerate(reversed(values)):
                            try:
                                # 将每个字节左移相应的位数并累加
                                shift_amount = (num_bytes - i - 1) * 8
                                combined_value |= int(byte) << shift_amount
                            except ValueError:
                                print("字节数据无法转换为整数。")
                                return None
                        logger.info(f'读取寄存器{start_address}成功，读取的值为：{combined_value}')
                        return combined_value
                    else:
                        logger.warning(f"接收到的起始地址 {int(received_start_address)} 与请求的起始地址 {int(start_address)} 不匹配")
            else:
                logger.warning(f"未知应答格式: bit7_6={bit7_6}, bit5_4={bit5_4}")
    return None


def write_registers(bus, arbitration_id=2, start_address=None, register_count=1, data=[]):
    """
    写入寄存器
    :param bus: CAN 总线对象
    :param arbitration_id: 仲裁 ID
    :param start_address: 起始寄存器地址
    :param register_count: 寄存器数量
    :param data: 要写入的数据列表，每个寄存器对应两个字节
    :return: 成功时返回 True，失败时返回错误码元组 (错误代码, 子错误代码)
    """
    if start_address is None:
        raise ValueError("起始地址不能为 None")
    if len(data) != register_count * 2:
        raise ValueError("数据长度与寄存器数量不匹配")
    request_frame = build_request_frame(WRITE_REGISTER, register_count, start_address, data)
    if send_can_message(bus, arbitration_id, request_frame):
        time.sleep(WAIT_TIME)
        response_msg = receive_can_message(bus)
        if response_msg:
            response_data = response_msg.data
            byte0 = response_data[0]
            bit7_6 = (byte0 & 0xC0) >> 6
            bit5_4 = (byte0 & 0x30) >> 4
            received_start_address = ((byte0 & 0x0F) << 8) | response_data[1]
            logger.info(f"bit7_6: {bit7_6}, bit5_4: {bit5_4}, received_start_address: {int(received_start_address)}")
            for i, byte in enumerate(response_data):
                logger.info(f"Byte {i}: {bin(byte)}")
            if bit7_6 == WRITE_REGISTER:
                if bit5_4 == INVALID_COUNT:  # 错误应答
                    error_code = byte0 & 0x0F
                    sub_error_code = response_data[1]
                    logger.error(f'error_code={error_code}, sub_error_code={sub_error_code}')
                    return False
                else:  # 正常应答
                    if received_start_address == start_address:
                        logger.info(f"写入寄存器{start_address}成功")
                        return True
                    else:
                        logger.warning(f"接收到的起始地址 {int(received_start_address)} 与请求的起始地址 {int(start_address)} 不匹配")
            else:
                logger.warning(f"未知应答格式: bit7_6={bit7_6}, bit5_4={bit5_4}")
    return False
    
def get_version(response,byte_num = 1):
    swapped_response = ((response & 0xFF) << 8) | ((response >> 8) & 0xFF)
    if byte_num ==1:
        return (swapped_response >> 8) & 0xFF
    else:#系统修订版本号
        return swapped_response

def send_broadcast(bus):
    """
    发送广播消息，请求从机返回设备版本信息
    :param bus: CAN 总线对象
    """
    # 构建广播请求帧
    byte0 = (MANAGEMENT_COMMAND << 6) | (MANAGEMENT_COMMAND << 4) | 0x00
    byte1 = 0x00
    request_frame = [byte0, byte1] + [0x00] * 6

    # 发送广播消息
    arbitration_id = 0x7FF
    if send_can_message(bus, arbitration_id, request_frame):
        time.sleep(WAIT_TIME)
        # 接收从机响应
        response_msg = receive_can_message(bus)
        if response_msg:
            response_data = response_msg.data
            byte0 = response_data[0]
            bit7_6 = (byte0 & 0xC0) >> 6
            bit5_4 = (byte0 & 0x30) >> 4
            hand_id = response_data[1]
            if bit7_6 == MANAGEMENT_COMMAND and bit5_4 == MANAGEMENT_COMMAND:
                protocol_sub_version = response_data[2]
                protocol_main_version = response_data[3]
                system_sub_version = response_data[4]
                system_main_version = response_data[5]
                system_revision_version = (response_data[6] << 8) | response_data[7]
                logger.info(f"从机响应 - HandID: {hand_id}")
                logger.info(f"协议子版本: {get_version(protocol_sub_version,1)}")
                logger.info(f"协议主版本: {get_version(protocol_main_version,1)}")
                logger.info(f"系统子版本: {get_version(system_sub_version,1)}")
                logger.info(f"系统主版本: {get_version(system_main_version,1)}")
                logger.info(f"系统修订版本号: {get_version(system_revision_version,2)}")
                return True
            else:
                logger.warning(f"未知应答格式: bit7_6={bit7_6}, bit5_4={bit5_4}")
                return False
        else:
            logger.error('收到的信息为空')
            return False
    else:
        return False


if __name__ == "__main__":
    # 初始化 CAN 总线
    can_bus = setup_can_bus()

    if can_bus:
        arbitration_id = 0x2  # 示例仲裁 ID

        # # 写入寄存器
        # write_data = [0xFF,0xFF]  # 2-247  F7
        # write_response = write_registers(can_bus, arbitration_id=2, start_address=1155, register_count=1, data=write_data)
        # if write_response is True:
        #     logger.info("Write registers success\n")
        # elif isinstance(write_response, tuple):
        #     logger.info(f"Write registers failed, error code: {write_response}")
            
        #  # 读取寄存器
        # read_response = read_registers(can_bus, arbitration_id=2, start_address=1155, register_count=1)
        # if read_response is not None:
        #     logger.info(f"Read registers success, values: {read_response}")
        # elif isinstance(read_response, tuple):
        #     logger.info(f"Read registers failed, error code: {read_response}")
        send_broadcast(can_bus)
        

        # 关闭 CAN 总线连接
        close_can_bus(can_bus)
