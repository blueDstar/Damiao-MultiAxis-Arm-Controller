# DAMIAO Robot Arm 2 DOF

GUI riêng trong `robot_arm_2dof/`, nằm cạnh `multi_motor/`. Bên trái là mô hình 3D cánh tay nối tiếp **vai–khuỷu**, với hai trục quay song song và motor nằm trong từng khớp. Bên phải là hai bảng Position nhỏ, độc lập. Nền đen, chữ sáng và nút có viền xanh neon.

## Chạy phần mềm

Từ thư mục `motor_controller`:

```powershell
python run_robot_arm_2dof.py
```

Mở ngay bus mô phỏng để thử mà không cần USB2CAN:

```powershell
python run_robot_arm_2dof.py --demo
```

Không cần thư viện 3D mới. GUI dùng Tkinter và các gói CAN/serial đã có trong `requirements.txt`; launcher dùng cùng cơ chế tìm gói cục bộ với `run_multi_motor.py`.

## Điều khiển

1. Chọn **MÔ PHỎNG** hoặc **CAN THẬT**, rồi bấm **MỞ PHIÊN**. Phần mềm đọc/xác minh cấu hình riêng từng motor; chưa Enable và chưa chạy khi mở phiên.
2. Bấm **Enable** ở motor cần dùng. Lệnh Enable dùng lại cách gửi trung tính của `multi_motor`, không đặt góc đích mới. Enable lần đầu lấy vị trí hiện tại làm zero phần mềm.
3. Nhập góc `deg`/`rad` và tốc độ `rad/s`/`rpm`. Đổi đơn vị sẽ chuyển giá trị tương đương. Tốc độ nhận −30…+30 rad/s hoặc VMAX nhỏ hơn đọc từ driver; Position chọn chiều theo góc cần quay, dùng độ lớn tốc độ. Tốc độ 0 yêu cầu dừng.
4. Mặc định **Quay thêm từ hiện tại**: 90° quay thêm 90°, 360° một vòng, 720° hai vòng. Có thể chọn **Tới góc motor từ zero** để dùng tọa độ tuyệt đối theo mốc phần mềm.
5. Bấm **Gửi góc** cho từng motor hoặc **GỬI GÓC CẢ HAI**. Hai giá trị được kiểm tra trước khi gửi; hai worker chạy độc lập với tốc độ riêng, không đồng bộ thời điểm đến đích và không nội suy đường đi Cartesian.
6. **Dừng** gửi yêu cầu tốc độ 0, vẫn giữ driver Enabled. **Disable** bỏ mô-men chủ động. **Zero** đặt lại mốc phần mềm của lệnh Position ở vị trí đang đứng; nút này không gửi lệnh quay về góc 0 và không làm mô hình đổi tư thế. Ngắt kết nối/đóng cửa sổ Disable cả hai trước khi đóng bus.

Mặc định J1: CAN `0x01`, Master `0x11`; J2: CAN `0x02`, Master `0x12`. Có thể đổi các ID khi chưa nối. COM, CAN bitrate và serial baud lấy từ cùng `.env` của `multi_motor`: `DAMIAO_CAN_CHANNEL`, `DAMIAO_CAN_BITRATE`, `DAMIAO_CAN_SERIAL_BAUDRATE`; tốc độ luồng lệnh lấy từ `DAMIAO_MULTI_COMMAND_RATE_HZ` (mặc định 100 Hz).

## Mô hình 3D và góc đo

Mô hình có đế, giá đỡ, hai motor dạng trụ có nắp/ốc/trục ra, thanh kim loại và kẹp thụ động ở đầu tay. Kẹp không phải một DOF điều khiển thêm. Phối cảnh và ánh sáng được dựng từ mesh 3D trên Canvas. Kéo chuột trái để xoay camera, cuộn để zoom, nhấp đúp hoặc **Góc nhìn gốc** để khôi phục camera. Thao tác camera không gửi lệnh CAN.

Khi nối CAN thật, khớp **chỉ cập nhật theo feedback góc nhiều vòng của đúng motor**. Nhập góc hoặc đổi camera không làm mô hình giả vờ tới đích. Khi feedback quá 1 giây chưa cập nhật, số đo hiển thị `—`, khớp giữ góc đo cuối và đánh dấu dữ liệu cũ. Lỗi một motor được xử lý bởi worker riêng, motor còn lại vẫn hoạt động.

**Cấu hình mô hình** cho phép đặt chiều dài hai thanh, cao trục vai, góc lắp lúc bắt đầu nối và chiều quay ±1 của từng khớp. Mốc mô hình được lấy từ feedback đầu tiên sau khi xác minh thành công, độc lập với zero của lệnh Position:

```text
q1 = góc_lắp_J1 + chiều_J1 × (feedback_J1 − góc_motor_J1_lúc_nối)
q2 = góc_lắp_J2 + chiều_J2 × (feedback_J2 − góc_motor_J2_lúc_nối)
góc cẳng tay so với đế = q1 + q2
```

Giá trị ban đầu: tay trên 0.30 m, cẳng tay 0.25 m, cao vai 0.17 m; góc lắp 55° và −80°. Đây là kích thước/tư thế minh họa, cần đặt theo cơ cấu thực để tọa độ đầu tay có ý nghĩa. Cấu hình được lưu vào `robot_arm_2dof/local_config.json`. Đổi cấu hình chỉ đổi mô hình hiển thị, không thay đổi lệnh/chiều quay của motor thật.

Chế độ mô phỏng dùng `DemoBus` với phản hồi tốc độ có thời gian đáp ứng và gia tốc hữu hạn, sau đó đi qua cùng codec, router và bộ điều khiển Position của `multi_motor`. Nhãn **FEEDBACK ẢO** phân biệt với dữ liệu motor thật. Mô hình chưa tính trọng lực, tải, va chạm hoặc giới hạn cơ khí; đây là giao diện điều khiển và quan sát động học, không phải mô phỏng động lực học đã hiệu chuẩn.

## Cấu trúc và kiểm thử

- `gui.py`: kết nối, thẻ Position, hàng đợi sự kiện Tk và ánh xạ feedback vào mô hình.
- `scene.py`: mesh, chiếu phối cảnh, shading và camera độc lập với CAN.
- `kinematics.py`: động học vai/khuỷu, chuyển đơn vị và kiểm tra ID.
- `demo_bus.py`: transport CAN mô phỏng, không mở cổng serial.

Phần mềm dùng trực tiếp `MultiMotorController`/`CANRouter` và `DamiaoUSB2CANBus` hiện có. Không sao chép hay thay đổi thuật toán Position trong app này.

```powershell
python -m unittest discover -s tests -p test_robot_arm_2dof.py -v
```

Kiểm thử bao gồm hình học, chiều/zero, đổi đơn vị, định tuyến hai CAN ID, Enable đứng yên, tới góc rồi dừng, quay nhiều vòng qua vùng wrap của feedback, xác minh hai nhánh kết nối bằng transport giả lập, Zero giữ nguyên tư thế 3D và giữ góc cuối khi mất feedback/ngắt kết nối.
