# Giao diện robot — Ubuntu 24.04 / ROS 2 Jazzy

Giao diện gồm bản đồ trung tâm, vị trí robot dạng chấm đỏ, waypoint, lộ trình,
tạo/tải bản đồ, trạng thái STM32 và hai LiDAR, Nav2/RViz và nút dừng.
Không sử dụng AI, micro, điều khiển tay hoặc định vị tự động.

## Chuẩn bị trên mini PC

Cần workspace robot đã build, Nav2, RViz, `velocity_arbiter` và các driver robot.
Cài các thư viện giao diện bằng Python hệ thống của Ubuntu:

```bash
sudo apt install python3-pyqt6 python3-yaml
source /opt/ros/jazzy/setup.bash
source /duong/dan/workspace/install/setup.bash
python3 /duong/dan/robot_ui/startup_layout.py
```

`rclpy` và các message/action ROS phải đến từ môi trường Jazzy; không cài chúng
bằng pip. `requirements.txt` chỉ liệt kê thư viện Python ngoài ROS.

## Cấu hình trước khi dùng

Chỉnh `robot_config.json`: `workspace`, `maps_directory`, các lệnh khởi chạy,
topic, action và frame theo robot mới. Đường dẫn tương đối của thư mục bản đồ
được tính từ workspace. Thư mục `robot_ui` có thể nằm ngoài workspace.

Lệnh Nav2 mặc định tương ứng repository cũ và truyền `map:={map}`.
Launch trên robot mới phải thực sự nhận đối số này. `nav2_opens_rviz: true`
nghĩa là launch Nav2 đã mở RViz; đổi thành `false` nếu cần giao diện chạy
`rviz_command` riêng (có thể thêm `-d /duong/dan/cau_hinh.rviz`).

Tải bản đồ chỉ ghi `selected_map.json` trong thư mục giao diện, không sửa launch,
không build lại workspace. Tắt Nav2 trước khi đổi hoặc tạo bản đồ. Tải bản đồ,
bật Nav2/RViz, sau đó đặt vị trí ban đầu bằng RViz. Giao diện nhận vị trí từ
`pose_topic` và bản đồ trực tiếp từ `map_topic`, kể cả khi tạo bản đồ.

Waypoint và lộ trình cũ được giữ trong `waypoints.json`, `multi_waypoints.json`.
Chỉ hiển thị điểm thuộc bản đồ đang chọn; thêm điểm lấy pose hiện tại từ AMCL.
Giữ tọa độ và quaternion trong dữ liệu để Nav2 sử dụng dù giao diện chỉ vẽ chấm.
Các thao tác lưu điểm, chọn điểm, xóa điểm có quyền xóa, tạo/chạy/xóa lộ trình
được giữ. Điểm đang được lộ trình tham chiếu không thể xóa.

## Dừng

Nút **DỪNG** gửi `Bool(true)` tới `emergency_stop_topic`, lặp lại khi khóa dừng,
hủy hàng đợi và mục tiêu Nav2; cũng gửi yêu cầu hủy mọi mục tiêu của action
`navigate_action` để bao gồm mục tiêu được gửi từ RViz.
**Cho phép chạy lại** gửi `Bool(false)` sau khi giao diện xử lý hủy mục tiêu.
Hành trình cũ không tự chạy tiếp. Trạng thái xác nhận dựa trên heartbeat
`arbiter_state_topic`, không chỉ dựa vào việc đã gửi lệnh.

Điều này giữ nguyên giao thức phần mềm của `velocity_arbiter` trong repository cũ.
Tay cầm đi trực tiếp tới STM32 chỉ dừng theo nút này nếu firmware STM32 thực sự
ưu tiên lệnh dừng của hệ thống ROS. Cần kiểm tra điều đó trên robot thật.

Nút tắt Nav2/RViz chỉ dừng nhóm tiến trình do giao diện khởi chạy.
Khi thoát giao diện sẽ yêu cầu dừng, xử lý hủy mục tiêu và dọn các tiến trình nó sở hữu.

## Phạm vi kiểm tra

Đã kiểm tra cú pháp Python và giao diện Qt trên Windows với ROS được mô phỏng.
Kết nối ROS 2 Jazzy, launch của mini PC, thiết bị, Nav2/RViz và dừng vật lý cần
được chạy thử trên Ubuntu với cấu hình robot thực tế.
