# UAV Farm Coverage — mã nguồn đồ án

Bản mã nguồn gọn để đưa lên GitHub, tách khỏi bộ bàn giao đầy đủ. [Hướng dẫn tiếng Anh](README.md).

## Nội dung

Ba UAV phối hợp theo đội hình tam giác, chung tiến độ, phục vụ 30 cảm biến trên nông trại 100 × 100 m có đồi cao tối đa 8 m. Có năm bố trí 101/202/303/404/505. Độ cao bắt đầu MISSION tham chiếu là 7 m; dọc tuyến, độ cao thay đổi theo kế hoạch và giới hạn an toàn. Các kế hoạch cung cấp sẵn dùng tốc độ ngang danh định 1,4 m/s.

- Lập vị trí phục vụ, phân công, thứ tự và đoạn nối 3D.
- Điều phối tập trung: cùng tiến/giữ; LiDAR có thể yêu cầu giữ và nâng chung trong giới hạn.
- Đọc PX4 qua px4_msgs và Micro XRCE-DDS; Gazebo cung cấp mô phỏng và pose để hậu kiểm.
- ns-3 phát lại vệt đã ghi để đánh giá byte nhận; không điều chỉnh đường bay.
- Có bảng tốc độ 15 cấu hình, bảng mạng 15 lượt và hình quỹ đạo đủ năm seed.

## Cài và kiểm tra

Xem [INSTALL.md](docs/INSTALL.md). Trước hết có thể kiểm tra offline, không cần mở Gazebo:

```bash
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python tools/validate_scenarios.py
python -m unittest discover -s tests -v
```

Sau khi cài ROS 2/PX4/Gazebo, sao chép `.env.example` thành `.env`, sửa các đường dẫn rồi chạy:

```bash
bash scripts/run_gazebo.sh 202
```

Thay 202 bằng seed khác. Lệnh tạo một case mới trong `runs/` và ghi log mới ở đó. Không sửa trực tiếp các kế hoạch trong `scenarios/`. Không chạy nhiều phiên cùng lúc.

```bash
bash scripts/run_ns3.sh 202 --build-only
bash scripts/run_ns3.sh 202
```

Đây là chạy mạng từ vệt cũ được kèm theo, không tự lấy vệt của chuyến Gazebo mới.

## Đọc code

Đọc theo thứ tự: [kiến trúc](docs/ARCHITECTURE.md) → `baseline_planner.py` → `mission_optimizer.py`/`arm_deadline.py` → `cruise_profile.py` → `execution_supervisor.py` → `fleet_guard.py`. Các file nằm trong `src/uav_farm/`. `tools/validate_scenarios.py` kiểm tra kế hoạch và hình học, không phát lệnh bay.

## Giới hạn cần giữ khi báo cáo

Coverage chính là phục vụ cảm biến về hình học; không phải quét kín ảnh toàn bộ nông trại. Byte nhận phải đọc ở ns-3. Energy proxy chưa hiệu chuẩn và không phải Wh đo từ pin PX4. Hai seed 404/505 có điểm gợi ý lịch sử. Điều phối tập trung chưa phải swarm tự tổ chức phân tán; khoảng cách giữa UAV chưa chứng minh liên kết radio. Các phép thử hữu hạn không bảo đảm mọi tốc độ, mọi địa hình hoặc bay thật.

Bộ GitHub không có log ULog lớn, build ROS, bản cài PX4/ns-3, Word đồ án hay toàn bộ các phiên bản thử nghiệm. Chúng được giữ trong bộ bàn giao đầy đủ của chủ dự án. Các mã `archive://` chỉ nhận dạng nguồn lịch sử, không phải đường dẫn để chạy.

## Công khai GitHub

Xem [PUBLISH.md](docs/PUBLISH.md). Chỉ đưa **thư mục repository này** lên GitHub; không chạy `git add` từ thư mục cha chứa bộ bàn giao hàng chục GB. Bản này chưa được đăng lên GitHub tự động.
