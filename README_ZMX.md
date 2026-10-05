# MiSTer FPGA ↔ ZMXDRIVE 하이브리드 카트리지 슬롯 브리지

이 문서는 **MiSTer FPGA MSX 코어(Cyclone V RTL)**와 **Main_MiSTer (ARM Cortex-A9 Linux)**, 그리고 **RPMP2 프로젝트의 가상 드라이브/슬롯 에뮬레이션 엔진(`libzmxdrive.a`)**을 실시간으로 인터페이스하는 **하이브리드 카트리지 슬롯 브리지**의 전체 아키텍처, 동작 원리, 프로토콜 및 사용자 가이드를 상세히 기술합니다.

---

## 1. 개요 (Overview)

MSX의 메인 하드웨어(Z80/R800 CPU, V9958 VDP, PSG, OPLL 등)는 **Cyclone V FPGA의 고속 하드웨어 로직**으로 정확하게 구동하고,
카트리지 슬롯(Slot 1 또는 Slot 2)의 실시간 메모리/IO 읽기·쓰기 트랜잭션은 **ARM HPS(리눅스) 상의 ZMXDRIVE 엔진**으로 브리징하여 가상 메가롬, 가상 디스크(DSK), MP3 오디오 스트리밍을 실시간으로 처리합니다.

---

## 2. 전체 시스템 아키텍처

```
+------------------------------------------------------------------------------------+
|  Cyclone V SoC (DE10-Nano / MiSTer)                                                |
|                                                                                    |
|  [ ARM Cortex-A9 (HPS - Linux) ]                                                   |
|    │                                                                               |
|    ├─ Main_MiSTer 메인 프로세스                                                    |
|    │    │                                                                          |
|    │    └─ zmx_service.cpp (실시간 I/O 서비스)                                     |
|    │         ├─ user_io_status_get("OIJ") 실시간 OSD 감지                          |
|    │         └─ libzmxdrive.a (C++ 가상 하드웨어 엔진)                             |
|    │              - msxread(cmd, addr)                                             |
|    │              - msxwrite(cmd, addr, val)                                       |
|    │              - SubSlotManager / RPMPROM16 / FATFS                             |
|    │                                                                               |
|    └─ SPI 통신 (EnableIO, spi_uio_cmd_cont, spi_w, DisableIO)                      |
|         │                                                                          |
|  [ FPGA Fabric (MSX Core) ]                                                        |
|         ▼                                                                          |
|    ┌────────────────────────────────────────────────────────┐                      |
|    │ sys/hps_io.sv (EXT_BUS SPI 인터페이스)                │                      |
|    └───────────────────────────┬────────────────────────────┘                      |
|                                │ EXT_BUS [35:0]                                    |
|    ┌───────────────────────────▼────────────────────────────┐                      |
|    │ rtl/peripheral/zmx_bridge.sv                           │                      |
|    │  - FSM: S_IDLE -> S_WAIT_HPS -> S_DONE                 │                      |
|    │  - CPU WAIT 제어 (cpu_wait_n, ~93us 세이프티 타임아웃) │                      |
|    │  - Slot 1 / Slot 2 커맨드 분기                         │                      |
|    └───────────────────────────┬────────────────────────────┘                      |
|                                │ slt_cs, slt_merq_n, cpu_wait_n, slt_rdata         |
|    ┌───────────────────────────▼────────────────────────────┐                      |
|    │ rtl/emsx_top.vhd (MSX 시스템 RTL)                      │                      |
|    │  - Page 1 & 2 엄격 게이팅 (0x4000 ~ 0xBFFF)            │                      |
|    │  - Page 0 (BIOS), Page 3 (RAM / Stack / 0xFFFF) 보호   │                      |
|    │  - T80pa (Z80 CPU WAIT_n 인터페이스)                  │                      |
|    └────────────────────────────────────────────────────────┘                      |
+------------------------------------------------------------------------------------+
```

---

## 3. 메모리 맵 및 페이지 게이팅 아키텍처

MSX는 64KB의 CPU 주소 공간을 4개의 16KB 페이지(Page 0~3)로 분할하여 관리합니다:

| 페이지 | 주소 범위 | 표준 역할 | ZMX 브리지 동작 |
| :---: | :---: | :--- | :--- |
| **Page 0** | `0x0000 ~ 0x3FFF` | **Main BIOS / 부팅 ROM** | **ZMX 개입 안 함 (보호)** |
| **Page 1** | `0x4000 ~ 0x7FFF` | **카트리지 슬롯 1/2 ROM** | **ZMX Bridge 활성화 (`slt_cs = 1`)** |
| **Page 2** | `0x8000 ~ 0xBFFF` | **카트리지 슬롯 1/2 ROM** | **ZMX Bridge 활성화 (`slt_cs = 1`)** |
| **Page 3** | `0xC000 ~ 0xFFFF` | **Main RAM, CPU 스택, `0xFFFF` 레지스터** | **ZMX 개입 안 함 (보호)** |

### ⚠️ 핵심 보호 메커니즘 (`emsx_top.vhd`)
카트리지 슬롯 칩셀렉트(`pZmxSltCs`)와 데이터 멀티플렉서(`cpu_di`)에 **Page 1 및 Page 2 게이팅**을 적용하여 부팅 프리즈를 원천 방지합니다:
```vhdl
-- ZMX Slot bus output connections (Only active on Page 1 & 2: 0x4000-0xBFFF)
pZmxSltCs <= (w_prislt_dec(1) and (w_page_dec(1) or w_page_dec(2))) when (pZmxSlotSel = '0') else
             (w_prislt_dec(2) and (w_page_dec(1) or w_page_dec(2)));

cpu_di    <= pZmxSltRDat when (pZmxSltRDatEn = '1' and (w_page_dec(1) = '1' or w_page_dec(2) = '1')) else ...
```
- **Page 0 (BIOS)**: 부팅 코드가 0ns 지연으로 완벽하게 실행됩니다.
- **Page 3 (Stack & Subslot)**: CPU 스택(PUSH/POP/RET) 조작 및 확장 슬롯 레지스터(`0xFFFF`) 접근 시 ZMX가 개입하지 않아 스택 파괴로 인한 시스템 다운을 차단합니다.

---

## 4. OSD 설정 및 슬롯 선택 (Slot 1 / Slot 2)

MiSTer OSD 메뉴(`F12`)에서 ZMX 브리지의 동작 모드를 자유롭게 지정할 수 있습니다:

```
"OIJ,ZMX Bridge,Disabled,Slot 1,Slot 2;"
```

- **`Disabled` (`2'b00`)**:
  - `enable = 0`으로 유지되어 ZMX 브리지가 완전히 격리되며, 오리지널 순수 MSX 코어로 구동됩니다.
- **`Slot 1` (`2'b01`)**:
  - ZMXDRIVE가 슬롯 1 확장 롬팩으로 매핑됩니다.
  - 전송 커맨드: `RD_SLTSL1 (0x00)` / `WR_SLTSL1 (0x01)`
  - *권장*: OSD의 `Slot 1` 설정을 `Empty`로 두십시오.
- **`Slot 2` (`2'b10`)**:
  - ZMXDRIVE가 슬롯 2 확장 롬팩으로 매핑됩니다.
  - 전송 커맨드: `RD_SLTSL2 (0x10)` / `WR_SLTSL2 (0x11)`
  - *권장*: OSD의 `Slot 2` 설정을 `Empty`로 두십시오. (슬롯 1에는 일반 게임 ROM 실행 가능)

---

## 5. SPI 실시간 핸드셰이크 프로토콜 (`EXT_BUS`)

### 5.1 SPI 커맨드 사양
| 커맨드 | 값 | 송수신 데이터 | 설명 |
| :--- | :---: | :--- | :--- |
| `CMD_ZMX_POLL` | `0x68` | `[15:8]`: cmd, `[0]`: pending | FPGA에 대기 중인 트랜잭션 요청 폴링 |
| `CMD_ZMX_ADDR` | `0x69` | `addr` (16비트), `wdata` (8비트) | 대상 메모리/IO 주소 및 쓰기 데이터 수신 |
| `CMD_ZMX_RESP` | `0x6A` | `rdata` (8비트) | HPS `msxread()` 처리 결과를 FPGA로 반환 |

### 5.2 2단계 Full-Duplex 핸드셰이크 시퀀스
HPS와 FPGA 간의 16비트 SPI 통신은 다음 순서로 엄격하게 동기화됩니다:
```
[HPS: Main_MiSTer]                                    [FPGA: zmx_bridge]
       │                                                      │
       │── 1. spi_uio_cmd_cont(CMD_ZMX_POLL) ───────────────>│ (byte_cnt=0: 커맨드 수신)
       │                                                      │  ext_dout <= {cmd, pending}
       │── 2. spi_w(0) ──────────────────────────────────────>│ (byte_cnt=1: 상태 전송)
       │<─ status 반환 ([0]: pending) ────────────────────────│
       │   DisableIO()                                        │
       │                                                      │
(pending == 1 인 경우)                                         │
       │                                                      │
       │── 3. spi_uio_cmd_cont(CMD_ZMX_ADDR) ───────────────>│ (byte_cnt=0: 커맨드 수신)
       │                                                      │  ext_dout <= req_addr
       │── 4. addr = spi_w(0) ───────────────────────────────>│ (byte_cnt=1: 주소 전송)
       │                                                      │  ext_dout <= req_wdata
       │── 5. wdata = spi_w(0) ──────────────────────────────>│ (byte_cnt=2: 쓰기데이터 전송)
       │   DisableIO()                                        │
       │                                                      │
  [msxread / msxwrite 실행]                                    │
       │                                                      │
       │── 6. spi_uio_cmd_cont(CMD_ZMX_RESP) ───────────────>│ (byte_cnt=0: 커맨드 수신)
       │── 7. spi_w(rdata) ──────────────────────────────────>│ (byte_cnt=1: rdata 수신)
       │   DisableIO()                                        │  req_pending <= 0
       │                                                      │  cpu_wait_n <= 1 (WAIT 해제!)
```

### 5.3 초고속 세이프티 타임아웃 (~93µs)
- Z80 CPU 클럭(21.48MHz) 기준 **2,000 클럭 (약 93µs)** 동안 HPS 응답이 없으면 타임아웃이 발동합니다.
- 타임아웃 발생 시 즉시 `cpu_wait_n <= 1`로 CPU를 릴리즈하고 오픈 버스 기본값(`0xFF`)을 반환하여, **부팅 시 슬롯 검색에서 지연이 누적되어 화면이 멈추는 문제를 완벽 차단**합니다.

---

## 6. 빌드 및 배포 가이드

### 6.1 `Main_MiSTer` 크로스 컴파일
```bash
cd /home/msx/Main_MiSTer
export PATH=/home/msx/Main_MiSTer/gcc-arm-10.2-2020.11-x86_64-arm-none-linux-gnueabihf/bin:$PATH
make -j$(nproc)
# 결과물: bin/MiSTer
```

### 6.2 `MSX_MiSTer` FPGA 코어 컴파일 (Quartus Prime)
```bash
cd /home/msx/MSX_MiSTer
export PATH=/home/msx/intelFPGA_lite/23.1std/quartus/bin:$PATH
quartus_sh --flow compile MSX
# 결과물: output_files/MSX.rbf
```

### 6.3 SD 카드 / USB 원클릭 자동 배포 (`misterfpga_to_sd.sh`)
```bash
# 마운트된 드라이브 자동 감지 및 rsync 동기화
./misterfpga_to_sd.sh

# 또는 경로 직접 지정
./misterfpga_to_sd.sh /media/msx/MiSTer_Data
```
동기화 항목:
- `MiSTer` 실행 바이너리 → `/MiSTer`
- `MSX.rbf` FPGA 코어 → `/_Computer/MSX.rbf`
- `X68000.rbf` FPGA 코어 → `/_Computer/X68000.rbf`
- `boot.vhd` 256MB 부팅 디스크 → `/games/MSX/boot.vhd`
- `*.mgl` 런처 스크립트 → `/games/MSX/`
- ZMX 롬 데이터 파일들 → `/games/MSX/roms/`