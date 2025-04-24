# BBB Custom UART Driver (BeagleBone Black)

## High-Reliability/Efficiency Custom UART Linux Device Driver Development (BeagleBone Black)
(고신뢰성/고효율 UART 리눅스 디바이스 드라이버 개발)

## 1. 프로젝트 개요 (Overview)
* 임베디드 리눅스(BeagleBone Black) 환경 기반의 고신뢰성/고효율 커스텀 UART 디바이스 드라이버 개발 프로젝트
* 기존 UART 통신의 낮은 효율과 신뢰성 문제를 해결하는 것을 목표로 함.

## 2. 주요 기능 및 성과 (Key Features & Achievements)
* 데이터 확장 및 오류 제어: 기존 8bit UART 통신을 64bit로 확장하고, ECC(해밍코드) 및 CRC 알고리즘을 적용하여 데이터 전송 효율 및 신뢰성 동시 향상 추구.
* 핵심 문제 해결: 데이터 길이 증가에 따른 타이밍 지연 및 심각한 수신 오류 발생 -> 3배 샘플링 및 비트 길이 추론 기반의 독자적인 오류 보정 알고리즘 설계 및 구현을 통해 문제 해결 주도.

## 3. 사용 기술 (Tech Stack)
* Language: C
* OS: Linux (5.10.168-ti-r79) on BeagleBone Black
* Core: Linux Kernel Device Driver Development, UART Protocol Internals
* Algorithms: ECC (Hamming Code), CRC

## 4. 빌드 방법 (How to Build)
1. BeagleBone Black에서 빌드하거나, 해당 보드용 Cross-Compile 환경을 구성합니다.
2. 프로젝트 루트 디렉토리에서 `make all` 명령을 실행합니다.
   ```bash
   make all
   ```
3. `Send/uart_dd_tx.ko`, `Receive/uart_dd_rx.ko` 커널 모듈 파일과 `Send/send`, `Receive/main` 실행 파일이 생성됩니다.

## 5. 사용 방법 (How to Use)
1.  **모듈 로드 및 디바이스 노드 생성:**
    ```bash
    make load
    ```
2.  **수신 앱 실행:** 별도의 터미널에서 수신 앱을 실행합니다.
    ```bash
    ./Receive/main
    ```
3.  **송신 앱 실행:** 다른 터미널에서 송신 앱을 실행하고 메시지를 입력합니다.
    ```bash
    ./Send/send
    ```
4.  **모듈 언로드 및 디바이스 노드 제거 및 빌드 파일 제거:**
    ```bash
    make unload
    ```

## 5. (향후 추가 예정)
* 상세 설계 및 알고리즘 설명 (Detailed Design)
* 결과 측정 및 분석 데이터 (Results & Analysis)
* 배운 점 및 개선 방향 (Lessons Learned)
