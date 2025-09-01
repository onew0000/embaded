import numpy as np
import time
import threading
from typing import List, Optional, Callable
from serial import EMGSerialManager, BluetoothManager
import cv2
import pygame
from dataclasses import dataclass
from enum import Enum

class TensionLevel(Enum):
    """긴장도 수준"""
    LOW = "low"
    NORMAL = "normal"
    HIGH = "high"

@dataclass
class TensionResult:
    """긴장도 판정 결과"""
    level: TensionLevel
    value: float
    timestamp: float
    duration: float

class EMGFilter:
    """EMG 신호 필터링 클래스"""
    
    def __init__(self, window_size: int = 10):
        self.window_size = window_size
        self.data_buffer: List[float] = []
        self.kalman_gain = 0.1
        self.estimated_value = 0.0
        self.estimated_error = 1.0
        self.measurement_error = 0.1
        
    def kalman_filter(self, measurement: float) -> float:
        """칼만 필터 적용"""

        predicted_error = self.estimated_error + 0.01
       
        self.kalman_gain = predicted_error / (predicted_error + self.measurement_error)
        self.estimated_value = self.estimated_value + self.kalman_gain * (measurement - self.estimated_value)
        self.estimated_error = (1 - self.kalman_gain) * predicted_error
        
        return self.estimated_value
    
    def low_pass_filter(self, value: float, alpha: float = 0.1) -> float:
        """저역통과 필터"""
        if not hasattr(self, '_low_pass_value'):
            self._low_pass_value = value
        
        self._low_pass_value = alpha * value + (1 - alpha) * self._low_pass_value
        return self._low_pass_value
    
    def moving_average_filter(self, value: float) -> float:
        """이동평균 필터"""
        self.data_buffer.append(value)
        
        if len(self.data_buffer) > self.window_size:
            self.data_buffer.pop(0)
        
        return np.mean(self.data_buffer)
    
    def apply_filters(self, raw_value: float) -> float:
        """모든 필터를 순차적으로 적용"""
        # 칼만 필터
        kalman_filtered = self.kalman_filter(raw_value)
        
        # 저역통과 필터
        low_pass_filtered = self.low_pass_filter(kalman_filtered)
        
        # 이동평균 필터
        final_filtered = self.moving_average_filter(low_pass_filtered)
        
        return final_filtered

class TensionAnalyzer:
    """근육 긴장도 분석기"""
    
    def __init__(self, threshold: float = 100.0, duration_threshold: float = 2.0):
        self.threshold = threshold
        self.duration_threshold = duration_threshold
        self.positive_start_time: Optional[float] = None
        self.negative_start_time: Optional[float] = None
        self.current_result: Optional[TensionResult] = None
        
    def analyze_tension(self, filtered_value: float) -> Optional[TensionResult]:
        """긴장도 분석 및 판정"""
        current_time = time.time()
        
        # 긍정 판정 (기준값 초과)
        if filtered_value > self.threshold:
            if self.positive_start_time is None:
                self.positive_start_time = current_time
                self.negative_start_time = None
            
            duration = current_time - self.positive_start_time
            
            if duration >= self.duration_threshold:
                self.current_result = TensionResult(
                    level=TensionLevel.HIGH,
                    value=filtered_value,
                    timestamp=current_time,
                    duration=duration
                )
                return self.current_result
        
        # 부정 판정 (기준값 미달)
        elif filtered_value <= self.threshold:
            if self.negative_start_time is None:
                self.negative_start_time = current_time
                self.positive_start_time = None
            
            duration = current_time - self.negative_start_time
            
            if duration >= self.duration_threshold:
                self.current_result = TensionResult(
                    level=TensionLevel.LOW,
                    value=filtered_value,
                    timestamp=current_time,
                    duration=duration
                )
                return self.current_result
        
        return None

class FeedbackProvider:
    """피드백 제공 클래스"""
    
    def __init__(self):
        self.pygame_initialized = False
        self.feedback_videos = {
            TensionLevel.HIGH: "feedback_high_tension.mp4",
            TensionLevel.LOW: "feedback_low_tension.mp4"
        }
        
    def initialize_pygame(self):
        """Pygame 초기화"""
        try:
            pygame.init()
            self.pygame_initialized = True
            print("Pygame이 초기화되었습니다.")
        except Exception as e:
            print(f"Pygame 초기화 실패: {e}")
    
    def provide_visual_feedback(self, tension_result: TensionResult):
        """시각적 피드백 제공"""
        if not self.pygame_initialized:
            self.initialize_pygame()
        
        # 화면 설정
        screen_width, screen_height = 800, 600
        screen = pygame.display.set_mode((screen_width, screen_height))
        pygame.display.set_caption("근육 긴장도 피드백")
        
        # 폰트 설정
        font = pygame.font.Font(None, 48)
        small_font = pygame.font.Font(None, 32)
        
        # 색상 설정
        if tension_result.level == TensionLevel.HIGH:
            bg_color = (255, 100, 100)  # 빨간색 (높은 긴장도)
            text_color = (255, 255, 255)
            message = "근육 긴장도가 높습니다!"
            advice = "긴장을 풀고 이완하세요"
        else:
            bg_color = (100, 100, 255)  # 파란색 (낮은 긴장도)
            text_color = (255, 255, 255)
            message = "근육 긴장도가 낮습니다!"
            advice = "적절한 힘을 주세요"
        
        # 메인 루프
        running = True
        start_time = time.time()
        display_duration = 5.0  # 5초간 표시
        
        while running and (time.time() - start_time) < display_duration:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
            
            # 화면 그리기
            screen.fill(bg_color)
            
            # 메인 메시지
            main_text = font.render(message, True, text_color)
            main_rect = main_text.get_rect(center=(screen_width//2, screen_height//2 - 50))
            screen.blit(main_text, main_rect)
            
            # 조언
            advice_text = small_font.render(advice, True, text_color)
            advice_rect = advice_text.get_rect(center=(screen_width//2, screen_height//2 + 20))
            screen.blit(advice_text, advice_rect)
            
            # 수치 정보
            value_text = small_font.render(f"긴장도: {tension_result.value:.1f}", True, text_color)
            value_rect = value_text.get_rect(center=(screen_width//2, screen_height//2 + 60))
            screen.blit(value_text, value_rect)
            
            # 지속 시간
            duration_text = small_font.render(f"지속 시간: {tension_result.duration:.1f}초", True, text_color)
            duration_rect = duration_text.get_rect(center=(screen_width//2, screen_height//2 + 100))
            screen.blit(duration_text, duration_rect)
            
            # 남은 시간
            remaining = display_duration - (time.time() - start_time)
            if remaining > 0:
                remaining_text = small_font.render(f"남은 시간: {remaining:.1f}초", True, text_color)
                remaining_rect = remaining_text.get_rect(center=(screen_width//2, screen_height//2 + 140))
                screen.blit(remaining_text, remaining_rect)
            
            pygame.display.flip()
            pygame.time.wait(50)  # 50ms 대기
        
        pygame.quit()

class EMGSystem:
    """EMG 시스템 메인 클래스"""
    
    def __init__(self, serial_port: str = '/dev/ttyUSB0', bt_port: str = '/dev/rfcomm0'):
        self.emg_manager = EMGSerialManager(port=serial_port)
        self.bt_manager = BluetoothManager(port=bt_port)
        self.filter = EMGFilter()
        self.analyzer = TensionAnalyzer()
        self.feedback_provider = FeedbackProvider()
        self.is_running = False
        self.data_callback: Optional[Callable[[float], None]] = None
        
    def set_data_callback(self, callback: Callable[[float], None]):
        """데이터 콜백 설정"""
        self.data_callback = callback
    
    def start_system(self) -> bool:
        """시스템 시작"""
        # 연결
        if not self.emg_manager.connect():
            print("EMG 센서 연결 실패")
            return False
        
        if not self.bt_manager.connect():
            print("블루투스 연결 실패")
            return False
        
        # 데이터 콜백 설정
        self.emg_manager.set_data_callback(self._process_emg_data)
        
        # 데이터 읽기 시작
        self.emg_manager.start_reading()
        self.is_running = True
        
        print("EMG 시스템이 시작되었습니다.")
        return True
    
    def stop_system(self):
        """시스템 중지"""
        self.is_running = False
        self.emg_manager.stop_reading()
        self.emg_manager.disconnect()
        self.bt_manager.disconnect()
        print("EMG 시스템이 중지되었습니다.")
    
    def _process_emg_data(self, raw_data: float):
        """EMG 데이터 처리"""
        # 블루투스로 전송
        self.bt_manager.send_data(raw_data)
        
        # 필터링
        filtered_data = self.filter.apply_filters(raw_data)
        
        # 긴장도 분석
        tension_result = self.analyzer.analyze_tension(filtered_data)
        
        # 피드백 제공
        if tension_result:
            print(f"긴장도 판정: {tension_result.level.value}, 값: {tension_result.value:.1f}")
            
            # 별도 스레드에서 피드백 제공
            feedback_thread = threading.Thread(
                target=self.feedback_provider.provide_visual_feedback,
                args=(tension_result,)
            )
            feedback_thread.daemon = True
            feedback_thread.start()
        
        # 외부 콜백 호출
        if self.data_callback:
            self.data_callback(filtered_data)

def main():
    """메인 함수"""
    # EMG 시스템 생성
    emg_system = EMGSystem()
    
    # 데이터 모니터링 콜백
    def monitor_data(filtered_data: float):
        print(f"필터링된 EMG 데이터: {filtered_data:.2f}")
    
    emg_system.set_data_callback(monitor_data)
    
    try:
        # 시스템 시작
        if emg_system.start_system():
            print("시스템이 실행 중입니다. Ctrl+C로 종료하세요.")
            
            # 메인 루프
            while True:
                time.sleep(1)
                
    except KeyboardInterrupt:
        print("\n프로그램을 종료합니다...")
    finally:
        emg_system.stop_system()

if __name__ == "__main__":
    main()
