import time
from IOBluetooth import IOBluetoothDeviceInquiry

# 델리게이트 클래스 (스캔 결과를 처리)
class BluetoothDelegate:
    def deviceInquiryDeviceFound_classOfDevice_rssi_completeName_(self, inquiry, device, deviceClass, rssi, name):
        # device.addressString()이 실제 MAC 주소를 반환합니다.
        print(f"발견된 기기: {name}")
        print(f"MAC 주소: {device.addressString()}")
        print("-" * 30)

    def deviceInquiryComplete_error_aborted_(self, inquiry, error, aborted):
        pass

def scan_bluetooth():
    print("Bluetooth 장치 스캔 중... (약 5-10초 소요)")

    delegate = BluetoothDelegate()
    inquiry = IOBluetoothDeviceInquiry.inquiryWithDelegate_(delegate)

    # 스캔 시작
    inquiry.start()

    # 스캔이 끝날 때까지 대기 (약 10초 정도 실행)
    time.sleep(10)

    inquiry.stop()
    print("스캔 종료")

if __name__ == "__main__":
    scan_bluetooth()
