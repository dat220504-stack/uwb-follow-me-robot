# UART A2 → A1

## Nối và nạp

Mỗi node dùng một file `.ino`. Code gửi UART nằm trong `Anchor2.ino`, code nhận nằm trong `Anchor1.ino`; không có thư viện UART tự viết đi kèm.

```text
A2 GPIO17 (TX2) ──► A1 GPIO16 (RX2)
A2 GND         ─── A1 GND
```

UART2: 115200 baud, 8N1, mức 3,3 V. Serial Monitor cũng dùng 115200 baud. Cấu hình dành cho ESP32 thường, GPIO16/17 còn trống.

## Dòng dữ liệu dễ đọc

Ví dụ A2 gửi khoảng cách 1,234 m:

```text
$A2,123,1234,0,1*46
```

Mỗi dòng kết thúc bằng ký tự xuống dòng `\n`.

| Trường | Ý nghĩa |
|---|---|
| `A2` | Nguồn gửi |
| `123` | Timestamp POLL 40 bit của Tag, viết dạng thập phân |
| `1234` | d2 RAW, đơn vị mm; A2 chưa bù offset |
| `0` | Tuổi mẫu tại A2 lúc gửi, đơn vị ms |
| `1` | Trạng thái: 1 = range tốt; 0 = range lỗi; -1 = mất/kết nối lại Tag |
| `46` | Checksum XOR của phần `A2,123,1234,0,1`, viết bằng hai chữ số hex |

Checksum là phép XOR từng byte, dùng kiểm tra lỗi cơ bản. Nó đơn giản hơn và phát hiện lỗi kém hơn CRC16 của bản trước. Đã bỏ mã khởi động, bộ đếm gói riêng và công suất nhận khỏi UART.

## Code chạy thế nào

1. **A2:** callback lưu d2 RAW, timestamp POLL và thời điểm đo. `loop()` tạo dòng chữ và gửi khi FIFO UART trống; UART bận thì giữ mẫu mới nhất, không đứng chờ.
2. **A1:** callback lưu d1. `loop()` đọc UART từng byte, kiểm tra dòng, giữ d2, rồi ghép hai mẫu khi timestamp POLL **bằng nhau**.
3. **A1:** mỗi cặp còn mới chỉ dùng một lần. Calibration tính offset A2; sau đó bù d2 một lần, tính góc, lọc Kalman và vote hướng mỗi 500 ms.

Timestamp lấy trực tiếp từ `device->timePollSent.getTimestamp()` của thư viện hiện tại. Không sửa thư viện DW1000. Tag vẫn quản lý A2 index 0 → A1 index 1; thứ tự UART đến không thay lịch UWB.

Mẫu quá 80 ms bị bỏ. Tuổi d1 tính trên đồng hồ A1; tuổi d2 gồm tuổi A2 gửi, 6 ms dự phòng truyền và thời gian ráp/lưu dòng tại A1. Hai ESP32 chỉ trao đổi **khoảng thời gian đã trôi qua**, không so trực tiếp `millis()` của hai bo. Mỗi dòng gửi nhỏ hơn 64 byte; 6 ms đủ dự phòng thời gian truyền tại 115200 baud.

A1 đọc tối đa 64 byte mỗi lượt loop. Dòng thiếu quá 20 ms, tràn bộ đệm, sai checksum hoặc sai trường dữ liệu bị bỏ; gặp `$` thì bắt đầu lại dòng. Nếu A1 ngừng phục vụ UART quá 80 ms hoặc RX tích quá 128 byte, A1 bỏ dữ liệu tồn và mẫu đã lưu. Dữ liệu tồn lúc khởi động cũng được bỏ.

Báo trạng thái -1 xóa mẫu, Kalman và vote cũ, giữ offset đã calibration. A2 khởi động lại sẽ báo kết nối khi tìm lại Tag. Sau calibration, không có cặp mới quá 500 ms thì A1 xóa lịch sử hướng; cửa sổ vote trống in `KHONG CO DU LIEU | valid=0`. Trong calibration giữ giới hạn ngắt cặp 1500 ms và thu mẫu tối đa 60000 ms như bản nguồn.

## Thử trên ba bo

1. Nạp đúng ba sketch, nối UART/GND. Kiểm tra thông báo `A1 READY`, `A2 READY`, `TAG READY`.
2. Đặt A1 trái, A2 phải, tâm anten cách nhau 0,50 m. Đặt Tag đứng yên chính giữa phía trước, cách trung điểm khoảng 1 m; chờ `CALIB_OK`.
3. Di chuyển Tag trái/phải, kiểm tra dấu góc và nhãn hướng ở A1. `DEBUG_LOG=true` bật log cặp đo/`uart_bad` ở A1 và dòng gửi ở A2.
4. Rút/nối dây TX hoặc tắt/bật Tag, A2: A1 phải bỏ hướng cũ khi mất dữ liệu, nhận cặp mới và giữ offset đã chốt.
5. Reset A1: đặt Tag giữa và calibration lại.

Chưa nạp/thử bộ ESP32 + BU01 thật trong lần sửa này. Cần đo độ trễ, tỷ lệ ghép cặp và sai số góc trên bo. Code hiện chỉ xuất Serial, chưa điều khiển motor hoặc xử lý LiDAR.
