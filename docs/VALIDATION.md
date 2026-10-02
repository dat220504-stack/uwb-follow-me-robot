# Kết quả kiểm tra UART đơn giản — 02/10/2026

Bản A1 hiện tại có 284 dòng: gom dữ liệu calibration, dùng chung phép tính cho hai khoảng cách, gộp xử lý timeout và rút ngắn log. UART vẫn dùng cùng định dạng dòng chữ với A2 hiện tại. Công thức và ngưỡng hình học, Kalman, vote, calibration của Tag cũ được giữ; cấu hình UWB, chân nối, antenna delay và thư viện giữ nguyên.

## Source thư viện

Dùng đúng `DW1000.zip` người dùng đã cung cấp trong dự án Điện. Bản tìm thấy trên máy có 1.286.019 byte, kiểm tra CRC toàn bộ ZIP thành công. Mã SHA256 của `DW1000Ranging.h`, `DW1000Ranging.cpp` và `library.properties` trùng các mã đã ghi khi kiểm tra tệp đính kèm trước đó. Source được giải nén riêng để biên dịch, không sửa và không đưa thư viện bên thứ ba vào repo.

| File | SHA256 |
|---|---|
| `DW1000.zip` | `b49b7e41eac0d9d0a963fbb32fd967a6338e7151708ac3e1b7912dbad6893a04` |
| `src/DW1000Ranging.h` | `a95f9056e10d779ee374dac7f10587ea731bf35e2eb4bf7faadbbc030153f11a` |
| `src/DW1000Ranging.cpp` | `831baad90246694fce2bfbf01c26c9ced3fcedf91b3872e0c767c9e2d26f8a83` |
| `library.properties` | `d9de8487632a85b2cfabe9c5946480cbc5b6d9c2743247750e2756f109fd86ba` |

Đã đọc source để xác nhận:

- Short address của Tag là `0x007D` từ hai byte đầu EUI, thứ tự byte thấp trước.
- Anchor có `getRange()`, `getRXPower()` và `timePollSent` public trong `DW1000Device`.
- Tag chép timestamp POLL chung vào các mục của gói RANGE; Anchor đọc timestamp trước khi tính range và gọi callback (`DW1000Ranging.cpp`, phần xử lý RANGE và `transmitRange`).
- Callback range chạy trong xử lý `DW1000Ranging.loop()` và được gọi sau khi đã cập nhật range; sketch chỉ chụp mẫu trong callback.
- Thư viện giữ reply delay mặc định 7000 µs và lập lịch theo index. Sketch không còn loại mẫu theo reply slot.
- Antenna delay đã set được dùng khi commit configuration; A1/A2 giữ 16461/16450, Tag giữ mặc định 16384.

## Biên dịch ESP32 thực

Arduino CLI **1.2.0**, ESP32 Arduino core **3.3.11** có sẵn trên máy, FQBN `esp32:esp32:esp32`. Tag và A2 không đổi so với lần biên dịch thành công trước. **Chưa xác nhận biên dịch bản A1 284 dòng:** tiến trình kiểm tra bản thử không trả được kết quả hoàn tất, còn lần chạy lại bị sandbox chặn khởi chạy Arduino CLI dù đã cấp quyền đọc công cụ. Không lấy kết quả của A1 cũ để khẳng định bản mới đã biên dịch.

| Sketch | Kết quả | Flash | RAM tĩnh |
|---|---|---:|---:|
| Tag hiện tại | Thành công | 303.435 byte | 23.700 byte |
| Anchor1 — 284 dòng | Chưa xác nhận | — | — |
| Anchor2 — bản rút gọn | Thành công | 304.139 byte | 23.804 byte |

Số liệu lịch sử: A1 506 dòng tại commit `3f5daae` đã biên dịch, dùng 327.771 byte flash / 24.180 byte RAM. Anchor1 gốc dùng 304.467 / 23.668 byte. Bản UART nhị phân tại commit `3286a96` dùng 310.731 / 24.196 byte ở A1 và 304.343 / 23.812 byte ở A2.

Lệnh mẫu, thay đường dẫn thư viện bằng bản đã kiểm tra của dự án:

```powershell
arduino-cli compile --fqbn esp32:esp32:esp32 --library 'C:\path\to\DW1000' firmware\Tag
arduino-cli compile --fqbn esp32:esp32:esp32 --library 'C:\path\to\DW1000' firmware\Anchor1
arduino-cli compile --fqbn esp32:esp32:esp32 --library 'C:\path\to\DW1000' firmware\Anchor2
```

## Kiểm tra logic chạy trên máy

Dùng Zig **0.14.1** với C++17 trên Windows, `-Wall -Wextra`. Các test include trực tiếp sketch; giả lập phần cứng/đồng hồ/UART ở biên I/O. Cả ba bộ kiểm tra đã chạy thành công với bản rút gọn.

| Bộ kiểm tra | Kết quả |
|---|---|
| Anchor1: ghép cặp, parser dòng chữ, tuổi mẫu, calibration, hình học, vote, timeout | PASS |
| Anchor2: chụp mẫu, dòng gửi, UART bận, tuổi mẫu, range lỗi, mất Tag | PASS |
| Tag: A2 trước A1, loại Anchor lạ, kết nối lại, callback chỉ lưu sự kiện | PASS |

Các tình huống đã chạy gồm A1 đến trước/A2 đến trước, sai timestamp POLL, cặp dùng lại, tràn timestamp 40 bit, mẫu cũ và biên 80 ms, checksum sai, mất byte, ký tự nhiễu, sai nguồn, số âm/quá giới hạn hoặc dư ký tự, dòng dở quá hạn, tràn dòng, dòng lặp không làm mới tuổi mẫu, backlog UART sau khi loop A1 ngừng hoặc RX tích quá nhiều, giữ offset khi mất/kết nối lại Tag, calibration ổn định/không ổn định, bù offset một lần, dấu góc trái/phải, tam giác vô lý, hòa phiếu, cửa sổ trống và mất mục tiêu.

Kiểm tra thêm các hành vi Tag gốc khi rút gọn: trước mốc 2 s chưa thu calibration; đủ 30 cặp nhưng chưa đủ 3 s vẫn chưa chốt; đủ 3 s nhưng thiếu 30 cặp vẫn chưa chốt; gain Kalman, reset sau 800 ms và vùng trễ 2°; mẫu tại đúng mốc 500 ms thuộc cửa sổ mới; chi tiết lấy mẫu mới nhất của hướng thắng; loop đình trệ qua hai cửa sổ không phát lại phiếu cũ.

Thực hiện lại theo [tests/README.md](../tests/README.md).

Bản 284 dòng còn được kiểm tra trực tiếp qua `loop()` từ lúc calibration đến lúc xuất hướng, tuổi mẫu khi `millis()` tràn và mốc ngắt cặp 1500 ms. Đã đối chiếu macro `TAG` của thư viện thật và dùng tên `TAG_SHORT` để tránh trùng; stub cũng có macro này để bắt lại lỗi đặt tên.

## Phần chưa kiểm tra

**Chưa nạp hoặc thử ba bo ESP32 + BU01 thật.** Chưa đo độ trễ UART/loop thực, tỷ lệ cặp ghép được, nhiễu ranging, khả năng tìm lại Anchor và sai số góc khi Tag di chuyển. Không có dữ liệu để kết luận các chỉ tiêu phần cứng này. Cần thử theo [docs/UART.md](UART.md).

Bản này chỉ xuất Serial ở A1. Điều khiển motor, LiDAR và gửi kết quả sang MCU chính chưa triển khai.
