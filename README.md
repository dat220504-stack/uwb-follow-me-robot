# Xe bám theo người sử dụng UWB

Đồ án xe tự hành hỗ trợ mua sắm dùng 3 module BU01 (DW1000), mỗi module nối với một ESP32 thường. Người mang Tag; hai Anchor đặt trên xe. Nhánh `feature/anchor-uart-processing` chuyển phần xử lý sang **ESP32 A1**, nhận khoảng cách A2 qua UART.

| Node | Sketch | Công việc |
|---|---|---|
| Tag trên người | [Tag.ino](firmware/Tag/Tag.ino) | Giữ giao thức UWB và thứ tự A2 trước, A1 sau |
| A1 bên trái xe | [Anchor1.ino](firmware/Anchor1/Anchor1.ino) | Nhận d1, nhận UART d2, ghép lượt đo, auto-offset, hình học, Kalman, vote |
| A2 bên phải xe | [Anchor2.ino](firmware/Anchor2/Anchor2.ino) | Lưu d2 RAW trong callback, gửi UART ngoài callback |

Mỗi node chỉ cần một file `.ino`, phần UART nằm ngay trong hai sketch Anchor. Các chức năng tự cân offset, tính góc, Kalman và vote của Tag cũ được giữ ở A1. Bản gốc nằm trên `main` tại [commit c435ee5](https://github.com/dat220504-stack/uwb-follow-me-robot/commit/c435ee53ac9a2b7701af0314960180eef9f1a5f2). [Markdown Tag gốc](docs/source/Tag_original.md) được giữ để đối chiếu.

Đọc `setup()` và `loop()` trước. A1 chia thành các mục có chú thích: cấu hình → nhận dữ liệu → ghép cặp → góc/Kalman → vote → calibration. Các khai báo hàm Arduino tự sinh đã được bỏ khỏi sketch; không thêm thư viện hỗ trợ.

## Cấu hình

| Thông số | Giá trị |
|---|---|
| Mode cả ba node | `DW1000.MODE_SHORTDATA_FAST_LOWPOWER` |
| Tag EUI / short address | `7D:00:22:EA:82:60:3B:9C` / `0x007D` |
| A1 EUI / short address | `86:17:5B:D5:A9:9A:E2:9C` / `0x1786` |
| A2 EUI / short address | `87:17:5B:D5:A9:9A:E2:9C` / `0x1787` |
| Antenna delay | Tag: mặc định **16384**; A1: **16461**; A2: **16450** |
| Khoảng cách tâm anten A1–A2 | **0,50 m** |
| Tuổi mẫu tối đa để ghép | **80 ms**, cùng timestamp POLL của Tag |
| Vote hướng / quá hạn mục tiêu | **500 ms** / **500 ms** sau cặp đo cuối |
| Serial Monitor / UART A2 → A1 | **115200 baud** / **115200 baud, 8N1** |

Tag giữ A2 ở index 0, A1 ở index 1. Đã bỏ điều kiện loại mẫu theo reply slot `7000/21000 µs`; lịch phát của thư viện được giữ. Auto-offset cân hai khoảng cách, còn timestamp POLL xác định hai mẫu thuộc cùng lượt đo.

Giữ nguyên thư viện `DW1000.zip` người dùng cung cấp, không thêm thư viện Kalman. Bản thư viện và kết quả kiểm tra được ghi trong [docs/VALIDATION.md](docs/VALIDATION.md).

## Nối dây

ESP32 ↔ BU01 trên cả ba node:

| Tín hiệu BU01 | GPIO ESP32 |
|---|---:|
| SCK | 18 |
| MISO | 19 |
| MOSI | 23 |
| CS | 4 |
| RST | 27 |
| IRQ | 34 |

UART giữa hai ESP32:

```text
A2 GPIO17 (TX2) ─────────► A1 GPIO16 (RX2)
A2 GND         ────────── A1 GND
```

Đây là UART mức **3,3 V**. Cấu hình dành cho ESP32 thường; GPIO16/17 phải còn trống trên bo đang dùng. Chân BU01, mode và antenna delay được giữ như bản nguồn. [docs/UART.md](docs/UART.md) mô tả gói, kiểm soát tuổi mẫu và cách thử.

## Nạp và chạy

1. Cài hỗ trợ board ESP32 và đúng thư viện `DW1000` đã dùng với bộ BU01 này.
2. Mở và nạp ba file `.ino` tương ứng với Tag, A1 và A2.
3. Nối UART/GND như trên; đặt A1 bên trái, A2 bên phải khi nhìn từ xe ra phía trước, cách tâm anten 0,50 m.
4. Mỗi lần bật/reset **A1**, đặt Tag đứng yên chính giữa phía trước hai Anchor, cách trung điểm khoảng 1 m. A1 chờ ổn định 2 s rồi lấy ít nhất 30 cặp trong ít nhất 3 s; độ lệch chuẩn mỗi khoảng cách phải ≤ 0,10 m.
5. Đọc Serial Monitor của **A1**, chờ `CALIB_OK` rồi mới di chuyển. A1 in hướng thắng vote cùng `dA1`, `dA2`, góc và `valid=1`; cửa sổ không có mẫu hợp lệ in `KHONG CO DU LIEU | valid=0`.
6. Tắt/bật lại Tag hoặc A2 sau khi calibration hoàn thành: A1 giữ offset trong RAM. Khi nhận báo mất/kết nối lại Tag hoặc không có cặp mới quá 500 ms, A1 xóa lịch sử hướng cũ rồi đợi cặp mới. Reset A1 sẽ calibration lại.

`PRINT_DETAILS=false` ở A1 giữ cách in chỉ nhãn hướng như bản nguồn. `DEBUG_LOG=true` ở A1 bật thông tin cặp RAW, tuổi mẫu và lỗi dòng UART; ở A2 bật log dòng UART. Chưa có Wi-Fi, OTA, ESP-NOW, điều khiển motor hoặc xử lý LiDAR.

Auto-offset không thay thế calibration khoảng cách tuyệt đối. Với hai Anchor, hình học hiện tại chọn nghiệm phía trước và không phân biệt Tag ở trước hay sau xe.

## Kiểm tra

[docs/VALIDATION.md](docs/VALIDATION.md) ghi bộ công cụ, mã kiểm tra thư viện, kết quả biên dịch và các bài kiểm tra logic chạy trên máy. Các kiểm tra dùng dữ liệu giả lập ở biên I/O; độ ổn định UWB/UART và sai số góc cần thử trên phần cứng theo [docs/UART.md](docs/UART.md).

Nguồn tham khảo của dự án:

- [thotro/arduino-dw1000](https://github.com/thotro/arduino-dw1000)
- [Makerfabs ESP32 UWB](https://github.com/Makerfabs/Makerfabs-ESP32-UWB)
- [jremington UWB Indoor Localization](https://github.com/jremington/UWB-Indoor-Localization_Arduino)

Repo chưa kèm thư viện bên thứ ba và chưa chỉ định giấy phép cho code dự án.
