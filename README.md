# Xe bám theo người sử dụng UWB

Đồ án xe tự hành hỗ trợ mua sắm dùng 3 module BU01 (DW1000), mỗi module nối với một ESP32 thường. Người mang Tag; hai Anchor đặt trên xe.

## Phiên bản gốc

Đây là code do chủ dự án cung cấp ngày 02/10/2026 (giờ Việt Nam), trước khi chuyển xử lý góc sang Anchor 1 qua UART.

- [Tag](firmware/Tag/Tag.ino): đo với hai Anchor, tự cân OFFSET A2 lúc khởi động, tính góc, lọc Kalman và vote hướng mỗi 500 ms.
- [Anchor 1](firmware/Anchor1/Anchor1.ino): đo và in RAW range, antenna delay **16461**.
- [Anchor 2](firmware/Anchor2/Anchor2.ino): đo và in RAW range, antenna delay **16450**.
- [File Tag gốc](docs/source/Tag_original.md): giữ nguyên tệp Markdown được cung cấp để đối chiếu.

Chưa triển khai UART giữa hai Anchor, điều khiển động cơ hoặc xử lý LiDAR trong phiên bản này. Các sketch không có Wi-Fi/OTA/ESP-NOW.

## Cấu hình đang dùng trong code

| Thông số | Giá trị |
|---|---|
| Mode cả ba node | `DW1000.MODE_SHORTDATA_FAST_LOWPOWER` |
| Tag EUI | `7D:00:22:EA:82:60:3B:9C` |
| A1 EUI / short address dự kiến | `86:17:5B:D5:A9:9A:E2:9C` / `0x1786` |
| A2 EUI / short address dự kiến | `87:17:5B:D5:A9:9A:E2:9C` / `0x1787` |
| Khoảng cách hai Anchor trong Tag | **0,50 m** |
| Cửa sổ ghép hai mẫu | 80 ms |
| Reply time Tag kiểm tra | A2: 7000 µs; A1: 21000 µs |
| Thứ tự danh sách Tag yêu cầu | A2 index 0; A1 index 1 |
| Serial Monitor | 115200 baud |

Chú thích đầu hai file Anchor còn ghi `MODE_LONGDATA_RANGE_LOWPOWER`, nhưng lệnh chạy thực tế dùng **SHORTDATA_FAST_LOWPOWER**. Giữ nguyên bản gửi; bảng trên phản ánh lệnh chạy thực tế. Antenna delay là giá trị riêng của bộ phần cứng này, không phải giá trị dùng chung cho mọi BU01.

## Nối ESP32 với BU01

| Tín hiệu BU01 | GPIO ESP32 |
|---|---:|
| SCK | 18 |
| MISO | 19 |
| MOSI | 23 |
| CS | 4 |
| RST | 27 |
| IRQ | 34 |

Cấu hình chân dành cho ESP32 thường, không dùng nguyên trạng cho ESP32-C3. Cấp nguồn theo đúng module/adapter đang dùng và nối GND chung giữa ESP32 với BU01.

## Nạp và chạy

1. Cài hỗ trợ ESP32 cho Arduino IDE và đúng bản thư viện `DW1000`/`DW1000Ranging` đã dùng với phần cứng. Bản thư viện cụ thể chưa được đưa vào repo này.
2. Mở riêng từng sketch trong `firmware/`, chọn đúng board/cổng rồi nạp vào node tương ứng.
3. Đặt A1 bên trái, A2 bên phải khi nhìn từ xe về phía trước; khoảng cách tâm anten phải khớp `ANCHOR_SPACING_M`.
4. Mỗi lần bật/reset Tag, đặt Tag đứng yên chính giữa phía trước hai Anchor, cách trung điểm 1 m theo quy trình của code.
5. Chờ `CALIB_OK` rồi mới di chuyển. Tag cần ổn định 2 s, thu ít nhất 30 cặp trong ít nhất 3 s; thời gian thực tế phụ thuộc tốc độ đo.
6. Đọc nhãn hướng, `dA1`, `dA2`, góc; khi không có mẫu hợp lệ trong cửa sổ vote, Tag in `KHONG CO DU LIEU`.

Auto-offset chỉ cân bằng hai khoảng cách khi Tag nằm giữa, không thay thế calibration khoảng cách tuyệt đối. Hai Anchor và giả thiết hình học hiện tại không phân biệt được Tag phía trước hay phía sau xe; code chọn nghiệm phía trước.

## Nguồn và trạng thái kiểm tra

Tag `.ino` được khôi phục từ Markdown bằng cách bỏ escape `\#`, `\*`, `\<`, đổi khoảng trắng NBSP sang khoảng trắng thường và gọn dòng trống. Không sửa logic, tham số hoặc API. Hai file Anchor giữ nội dung người dùng cung cấp.

Lần nhập này kiểm tra nội dung và cấu hình; **chưa biên dịch cho ESP32 hoặc thử trên BU01**. Chú thích kiểm tra cũ trong code là thông tin của bản nguồn, không phải kết quả kiểm tra mới. Cần xác nhận đúng phiên bản thư viện, đặc biệt các API danh sách thiết bị và reply time.

Nguồn tham khảo của dự án:

- [thotro/arduino-dw1000](https://github.com/thotro/arduino-dw1000)
- [Makerfabs ESP32 UWB](https://github.com/Makerfabs/Makerfabs-ESP32-UWB)
- [jremington UWB Indoor Localization](https://github.com/jremington/UWB-Indoor-Localization_Arduino)

Repo này chưa kèm thư viện bên thứ ba và chưa chỉ định giấy phép cho code dự án.
