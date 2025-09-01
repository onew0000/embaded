import serial
import time
import struct
import threading
from typing import Callable, Optional

class EMGSerialManager:
    """근전도 센서 데이터를 관리하는 시리얼 매니저"""
    
    def __init__(self, port: str = '/dev/ttyUSB0', baudrate: int = 9600):
        self.port = port
        self.baudrate = baudrate
        self.serial_connection: Optional[serial.Serial] = None
        self.is_connected = False
        self.is_running = False
        self.data_callback: Optional[Callable[[float], None]] = None
        self.thread: Optional[threading.Thread] = None
        
    def connect(self) -> bool:
        """시리얼 포트에 연결"""
        try:
            self.serial_connection = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1
            )
            self.is_connected = True
            print(f"시리얼 포트 {self.port}에 연결되었습니다.")
            return True
        except serial.SerialException as e:
            print(f"시리얼 포트 연결 실패: {e}")
            return False
    
    def disconnect(self):
        """시리얼 포트 연결 해제"""
        self.is_connected = False
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
        print("시리얼 포트 연결이 해제되었습니다.")
    
    def set_data_callback(self, callback: Callable[[float], None]):
        """데이터 수신 시 호출될 콜백 함수 설정"""
        self.data_callback = callback
    
    def start_reading(self):
        """데이터 읽기 시작"""
        if not self.is_connected:
            print("먼저 시리얼 포트에 연결해주세요.")
            return
        
        self.is_running = True
        self.thread = threading.Thread(target=self._read_loop)
        self.thread.daemon = True
        self.thread.start()
        print("데이터 읽기가 시작되었습니다.")
    
    def stop_reading(self):
        """데이터 읽기 중지"""
        self.is_running = False
        if self.thread:
            self.thread.join()
        print("데이터 읽기가 중지되었습니다.")
    
    def _read_loop(self):
        """데이터 읽기 루프"""
        while self.is_running and self.is_connected:
            try:
                if self.serial_connection.in_waiting > 0:
                    # 데이터 읽기 (4바이트 float 값)
                    data = self.serial_connection.read(4)
                    if len(data) == 4:
                        # float 값으로 변환
                        emg_value = struct.unpack('f', data)[0]
                        
                        # 콜백 함수 호출
                        if self.data_callback:
                            self.data_callback(emg_value)
                
                time.sleep(0.01)  # 10ms 대기
                
            except serial.SerialException as e:
                print(f"데이터 읽기 오류: {e}")
                break
            except Exception as e:
                print(f"예상치 못한 오류: {e}")
                break
    
    def send_command(self, command: str) -> bool:
        """센서에 명령 전송"""
        if not self.is_connected:
            return False
        
        try:
            self.serial_connection.write(command.encode())
            return True
        except Exception as e:
            print(f"명령 전송 실패: {e}")
            return False

class BluetoothManager:
    """블루투스 통신을 관리하는 클래스"""
    
    def __init__(self, port: str = '/dev/rfcomm0', baudrate: int = 9600):
        self.port = port
        self.baudrate = baudrate
        self.bluetooth_connection: Optional[serial.Serial] = None
        self.is_connected = False
    
    def connect(self) -> bool:
        """블루투스 모듈에 연결"""
        try:
            self.bluetooth_connection = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1
            )
            self.is_connected = True
            print(f"블루투스 모듈 {self.port}에 연결되었습니다.")
            return True
        except serial.SerialException as e:
            print(f"블루투스 연결 실패: {e}")
            return False
    
    def disconnect(self):
        """블루투스 연결 해제"""
        self.is_connected = False
        if self.bluetooth_connection and self.bluetooth_connection.is_open:
            self.bluetooth_connection.close()
        print("블루투스 연결이 해제되었습니다.")
    
    def send_data(self, data: float) -> bool:
        """데이터를 블루투스로 전송"""
        if not self.is_connected:
            return False
        
        try:
            # float 값을 4바이트로 변환하여 전송
            data_bytes = struct.pack('f', data)
            self.bluetooth_connection.write(data_bytes)
            return True
        except Exception as e:
            print(f"데이터 전송 실패: {e}")
            return False
    
    def receive_data(self) -> Optional[bytes]:
        """블루투스에서 데이터 수신"""
        if not self.is_connected:
            return None
        
        try:
            if self.bluetooth_connection.in_waiting > 0:
                return self.bluetooth_connection.read(self.bluetooth_connection.in_waiting)
        except Exception as e:
            print(f"데이터 수신 실패: {e}")
        
        return None

# 사용 예시
if __name__ == "__main__":
    # EMG 센서 매니저 생성
    emg_manager = EMGSerialManager(port='/dev/ttyUSB0')
    
    # 블루투스 매니저 생성
    bt_manager = BluetoothManager(port='/dev/rfcomm0')
    
    # 데이터 수신 콜백 함수
    def on_emg_data(data: float):
        print(f"EMG 데이터 수신: {data:.2f}")
        # 블루투스로 데이터 전송
        bt_manager.send_data(data)
    
    # 콜백 설정
    emg_manager.set_data_callback(on_emg_data)
    
    try:
        # 연결
        if emg_manager.connect() and bt_manager.connect():
            # 데이터 읽기 시작
            emg_manager.start_reading()
            
            # 메인 루프
            while True:
                time.sleep(1)
                
    except KeyboardInterrupt:
        print("\n프로그램을 종료합니다...")
    finally:
        emg_manager.stop_reading()
        emg_manager.disconnect()
        bt_manager.disconnect()
