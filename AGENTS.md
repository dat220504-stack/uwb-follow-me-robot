# Quy tắc làm việc với dự án

- Đọc code hiện tại trước khi sửa. File người dùng gửi mới nhất là nguồn cấu hình, không lấy giá trị calibration cũ trong hội thoại để ghi đè.
- Giữ nguyên thư viện DW1000Ranging trừ khi người dùng yêu cầu thay đổi. Kiểm tra API theo đúng source thư viện đang dùng; không tự đoán API.
- Bản gốc trên main tính góc trên Tag. Nhánh feature/anchor-uart-processing đã chuyển xử lý sang A1 qua UART; xem docs/VALIDATION.md để biết mức kiểm tra thực tế.
- Hai file firmware/Anchor1/UwbUart.h và firmware/Anchor2/UwbUart.h phải giống nhau để mở từng sketch độc lập trong Arduino IDE. Khi sửa giao thức, cập nhật cả hai.
- Giữ các tính năng calibration, lọc và vote hiện có khi chuyển nơi xử lý, trừ khi có yêu cầu khác.
- Khi sửa code theo yêu cầu, kiểm tra phù hợp rồi commit và đẩy lên repo này; ghi rõ nội dung thay đổi và cung cấp link commit. Không force-push hoặc ghi đè thay đổi của người khác.
- Không khẳng định đã biên dịch hay thử phần cứng nếu chưa thực hiện. Không thêm test chỉ để lặp lại implementation.
- Giao tiếp với chủ dự án bằng tiếng Việt ngắn gọn, giải thích dễ hiểu. Báo rõ khi chưa đủ dữ liệu để kết luận.
