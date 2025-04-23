# bbb-custom-uart-driver

# 고신뢰성/고효율 UART 리눅스 디바이스 드라이버 개발 (BeagleBone Black)

## 1. 프로젝트 개요 (Overview)
* 임베디드 리눅스(BeagleBone Black) 환경 기반의 고신뢰성/고효율 커스텀 UART 디바이스 드라이버 개발 프로젝트 (팀 프로젝트, 본인 팀장 역할 수행)
* 기존 UART 통신의 낮은 효율과 신뢰성 문제를 해결하는 것을 목표로 함.

## 2. 주요 기능 및 성과 (Key Features & Achievements)
* **데이터 확장 및 오류 제어:** 기존 8bit UART 통신을 64bit로 확장하고, ECC(해밍코드) 및 CRC 알고리즘을 적용하여 데이터 전송 효율 및 신뢰성 동시 향상 추구.
* **핵심 문제 해결:** 데이터 길이 증가에 따른 타이밍 지연 및 심각한 수신 오류 발생 -> **3배 샘플링 및 비트 길이 추론 기반의 독자적인 오류 보정 알고리즘** 설계 및 구현을 통해 문제 해결 주도.
* **결과:** 비트 오류율(BER)을 기존 약 3% 수준에서 **0.01% 미만**으로 획기적으로 개선하여 통신 신뢰도 확보.
* **인정:** 프로젝트 결과의 우수성을 인정받아 담당 교수로부터 **논문 작성 권유**.
* **프로젝트 관리:** 클라우드 기반 협업 환경 구축 및 활용, 명확한 역할 분담 및 일정 관리를 통해 **제한된 시간 내 성공적인 프로젝트 완수** (팀장 역할).

## 3. 사용 기술 (Tech Stack)
* **Language:** C
* **OS:** Linux (Kernel vX.Y - 가능하다면 버전 명시) on BeagleBone Black
* **Core:** Linux Kernel Device Driver Development, UART Protocol Internals
* **Algorithms:** ECC (Hamming Code), CRC, Custom Error Correction Algorithm (3x Sampling based)
* **Tools:** (사용한 컴파일러, 디버거 등 명시하면 좋음)

## 4. (향후 추가 예정)
* 빌드 및 사용 방법 (Build & Usage)
* 상세 설계 및 알고리즘 설명 (Detailed Design)
* 결과 측정 및 분석 데이터 (Results & Analysis)
* 배운 점 및 개선 방향 (Lessons Learned)
