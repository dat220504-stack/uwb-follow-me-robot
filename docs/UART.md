# UART A2 → A1 và ghép lượt đo

## Nối và nạp

Nạp đầy đủ ba thư mục `firmware/Tag`, `firmware/Anchor1`, `firmware/Anchor2`. Hai thư mục Anchor có bản `UwbUart.h` giống nhau để Arduino IDE có thể mở mỗi sketch độc lập.

| Dây | Từ A2 | Sang A1 |
|---|---|---|
| Dữ liệu | GPIO17 / TX2 | GPIO16 / RX2 |
| Tham chiếu điện áp | GND | GND |

115200 baud, 8N1, UART mức 3,3 V, dùng `HardwareSerial(2)`. Các chân này dành cho ESP32 thường và phải còn trống trên bo; bo có PSRAM sử dụng GPIO16/17 cần chọn chân UART khác tại `UART_RX_PIN` / `UART_TX_PIN`. RX của A2 và TX của A1 không cần nối trong bước này. Serial Monitor 115200 baud dùng UART0 riêng.

## Gói nhị phân 32 byte

Các số nhiều byte dùng little endian. `sequence` chỉ dùng chống lặp/đảo thứ tự gói từ **cùng A2**, không dùng làm số lượt đo chung với A1.

| Byte | Nội dung |
|---|---|
| 0–1 | Magic `B5 62` |
| 2 | Phiên bản = 1 |
| 3 | Loại: 1 = mẫu; 2 = Tag mất/kết nối lại |
| 4–5 | Nguồn A2 = `0x1787` |
| 6–7 | Tag đang đo = `0x007D` |
| 8–11 | `bootId`: số ngẫu nhiên lúc A2 khởi động |
| 12–15 | `sequence`: bộ đếm gói A2, uint32 |
| 16–20 | Timestamp phát POLL của **Tag**, 40 bit |
| 21 | `valid`: 1 khi d2 RAW hữu hạn, > 0 và ≤ 10 m; 0 khi lỗi |
| 22–25 | d2 RAW, int32, đơn vị mm |
| 26–27 | Công suất nhận, int16, đơn vị 0,1 dBm |
| 28–29 | Tuổi mẫu tại A2 khi gửi, uint16, đơn vị ms |
| 30–31 | CRC16-CCITT-FALSE của byte 0–29, polynomial `0x1021`, init `0xFFFF` |

Gói loại 2 có `valid=0`. A1 xóa mẫu, Kalman và vote cũ khi nhận gói này hoặc `bootId` đổi, giữ offset đã calibration. Kết nối lại Tag tại A2 được báo bằng cùng loại gói để vô hiệu hóa mẫu trước lần kết nối.

A2 gửi ngoài callback khi FIFO UART đủ chỗ cho toàn bộ 32 byte. Không dùng hàng đợi TX phần mềm; tuổi gửi gồm thời gian từ callback và thời gian chờ của số byte đã có trong FIFO. Khi UART bận, A2 giữ mẫu mới nhất; mẫu quá tuổi bị bỏ. 32 byte trên dây mất khoảng 2,78 ms.

## Timestamp và tuổi mẫu

Source `DW1000.zip` của dự án cho thấy Tag ghi `timePollSent` vào mỗi mục Anchor trong gói RANGE. Mỗi Anchor đọc timestamp này trước khi tính range và gọi callback. A1/A2 lấy trực tiếp `device->timePollSent.getTimestamp()` trong callback; không sửa giao thức UWB hay thư viện.

A1 ghép khi:

1. Có một mẫu mới từ BU01 của A1 và một mẫu mới từ UART A2.
2. Hai mẫu có timestamp POLL **bằng nhau**. Không ghép chỉ vì chúng đến gần nhau.
3. Mỗi mẫu còn trong giới hạn 80 ms; cặp đã dùng không được dùng lại.

A1 nhận bên nào trước thì giữ bên đó. Thứ tự đến qua UART không quyết định thứ tự UWB; Tag tiếp tục quản lý A2 index 0 → A1 index 1.

Tuổi d1 tính bằng `millis()` A1 từ callback. Tuổi d2 gồm tuổi do A2 gửi, 3 ms dự phòng truyền 32 byte, thời gian ráp gói và thời gian lưu tại A1. Giá trị A2 gửi là **khoảng thời gian đã trôi qua**, không phải `millis()` tuyệt đối; hai đồng hồ ESP32 không được so trực tiếp. Timestamp POLL chỉ so bằng nhau, nên không cần sắp thứ tự timestamp qua vòng tràn 40 bit.

Parser nhận tối đa 64 byte mỗi lượt loop, không đợi đủ gói. Sai CRC, sai nguồn/Tag/phiên bản, gói lặp và gói quá tuổi bị bỏ; gói dở quá 20 ms được bỏ và parser tìm lại magic. Nếu A1 ngừng phục vụ UART quá 80 ms hoặc buffer tích quá 128 byte, A1 bỏ backlog và mẫu đã lưu, đợi dữ liệu mới. Buffer từ lúc khởi động cũng được bỏ.

Sau calibration, cửa sổ vote không có kết quả hợp lệ in `KHONG CO DU LIEU | valid=0`. Không có cặp đo mới quá 500 ms thì xóa lịch sử hướng/Kalman. Vote vẫn xuất mỗi 500 ms, nên thông báo Serial có thể đến ở mốc vote tiếp theo. Trong calibration giữ giới hạn ngắt cặp 1500 ms và thời gian thu tối đa 60000 ms như bản nguồn.

## Thử trên ba bo

1. Kiểm tra đường UART/GND, nạp đúng sketch cho từng node. A1 phải in `A1 READY`, A2 in `A2 READY`, Tag in `TAG READY`.
2. Đặt Tag giữa, đứng yên phía trước khoảng 1 m. Bật `DEBUG_LOG=true` ở A1 nếu cần xem `PAIR`, d1/d2 RAW, tuổi mẫu và `crc_bad`. Nếu A1 chỉ báo chờ cặp cùng POLL, kiểm tra dây, địa chỉ, mode và việc A2 vào trước A1 trên Tag.
3. Chờ `CALIB_OK`; kiểm tra offset, hai khoảng cách sau bù và hướng `THANG`. Không calibration bằng cách ép d1/d2 về 1 m.
4. Di chuyển Tag lần lượt sang trái/phải trong vùng phía trước, kiểm tra dấu góc và các nhãn hướng. Giữ baseline đúng 0,50 m.
5. Rút dây TX A2: A1 phải ngừng xuất kết quả hợp lệ sau khi các cửa sổ hiện tại hết mẫu. Nối lại: đợi cặp mới, giữ offset cũ.
6. Tắt/bật A2, rồi Tag: mẫu/vote cũ phải được xóa và đo lại, offset đã chốt vẫn giữ ở A1.
7. Reset A1: phải quay lại calibration; đặt Tag giữa và chờ `CALIB_OK` trước khi di chuyển.
8. Thử gói lỗi/thiếu và UART bận trong kiểm tra trên máy trước; sau đó kiểm tra log trên bo nếu có lỗi đường dây. Chưa nối phần điều khiển motor ở giai đoạn này.

Chưa thực hiện các bước phần cứng này trong lần chuyển code. Thời gian callback, tốc độ ghép và sai số góc thực tế cần log từ bộ BU01 của dự án.
