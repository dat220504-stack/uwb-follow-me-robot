# Kiểm tra logic trên máy

Các bài kiểm tra C++ include trực tiếp ba sketch. Chỉ `millis`, UART, SPI và đối tượng DW1000 ở biên I/O được giả lập; parser, ghép mẫu, calibration, hình học, Kalman, vote và thứ tự Anchor chạy từ code firmware.

Trên Windows có GCC C++17:

```powershell
.\tests\run_host_tests.ps1
```

Hoặc dùng Zig 0.14.1 (có C++ compiler):

```powershell
.\tests\run_host_tests.ps1 -Compiler 'C:\path\to\zig.exe' -Zig
```

Trên Linux/macOS, dùng `c++ -std=c++17 -Wall -Wextra -I tests/stubs tests/test_anchor1.cpp -o /tmp/test_anchor1`, chạy binary rồi làm tương tự với `test_anchor2.cpp` và `test_tag.cpp`.

Bộ test kiểm tra:

- Ghép cùng POLL theo cả hai thứ tự đến; không ghép hai lượt khác nhau, không dùng lại cặp, xử lý vòng tràn timestamp 40 bit.
- Checksum XOR, nguồn sai, trường số lỗi, mất byte, ký tự nhiễu, tràn dòng, timeout dòng dở, dòng lặp và backlog sau khi A1 ngừng loop.
- A2 giữ mẫu mới nhất khi UART bận; bỏ mẫu quá tuổi; báo range không hợp lệ và mất/kết nối lại Tag.
- Thông báo mất/kết nối lại Tag xóa mẫu/hướng cũ nhưng giữ offset A1.
- Calibration với dữ liệu ổn định/không ổn định; bù offset một lần; dấu góc, tam giác vô lý, hòa phiếu, cửa sổ trống và mất mục tiêu.
- Tag giữ A2 trước A1, loại Anchor lạ và không xóa thiết bị ngay trong callback.

Các test không mô phỏng sóng UWB, thời gian ISR, UART driver thật hoặc nhiễu đo. Biên dịch với thư viện thực và thử BU01 là bước riêng; xem [kết quả kiểm tra](../docs/VALIDATION.md).
