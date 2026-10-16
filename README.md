# Hall-sensor
 Read TLE4999C8 with STM32F103C8T6
# STM32F103C8T6 – Magnetic Sensor & Wheel RPM Counter

## 1. Giới thiệu

Dự án sử dụng vi điều khiển **STM32F103C8T6** để thu nhận và giải mã tín hiệu từ cảm biến từ trường, xử lý dữ liệu cảm biến và tính toán tốc độ quay của bánh xe (RPM) cùng số vòng quay.

Chương trình được viết bằng ngôn ngữ C, sử dụng STM32 HAL kết hợp với truy cập thanh ghi ngoại vi để xử lý tín hiệu với độ phân giải thời gian cao.

### Các chức năng chính

- Thu nhận cạnh xuống của tín hiệu bằng EXTI.
- Đo khoảng thời gian giữa các cạnh bằng Timer.
- Phát xung Trigger để yêu cầu cảm biến gửi dữ liệu.
- Giải mã các khoảng thời gian thành các giá trị nibble 4 bit.
- Ghép dữ liệu thành giá trị RAW 12 bit.
- Chuyển đổi giá trị RAW thành thang đo từ 1 đến 100.
- Quy đổi dữ liệu RAW thành giá trị từ trường.
- Phát hiện chu kỳ quay, đếm số vòng quay và tính RPM.
- Theo dõi dữ liệu debug để kiểm tra quá trình thu nhận và giải mã tín hiệu.

## 2. Phần cứng

| Thành phần | Chức năng |
|---|---|
| STM32F103C8T6 | Vi điều khiển trung tâm |
| Cảm biến từ trường | Phát tín hiệu phản ánh từ trường đo được |
| PA12 / EXTI12 | Đường tín hiệu dùng cho quá trình Trigger và thu nhận cạnh tín hiệu |
| TIM2 | Bộ định thời đo khoảng thời gian giữa các cạnh |
| Nguồn và mạch kết nối | Cung cấp nguồn và kết nối tín hiệu |

**Lưu ý:** Cần kiểm tra sơ đồ mạch, cấu hình chân và yêu cầu giao tiếp của cảm biến thực tế trước khi cấp nguồn. Cấu hình trong chương trình hiện sử dụng PA12 làm đầu ra open-drain để phát Trigger, đồng thời sử dụng EXTI12 để nhận cạnh xuống trên đường tín hiệu này.

## 3. Cấu trúc chương trình

Chương trình được tổ chức thành các nhóm chức năng chính:

| Hàm | Chức năng |
|---|---|
| `SystemClock_Config()` | Cấu hình xung nhịp hệ thống |
| `GPIO_Init()` | Cấu hình GPIO và EXTI |
| `Timer_Init()` | Cấu hình TIM2 để đo thời gian |
| `Spc_Schedule()` | Lập lịch phát Trigger |
| `Send_Trigger()` | Phát xung Trigger và khởi tạo quá trình nhận dữ liệu |
| `EXTI15_10_IRQHandler()` | Xử lý ngắt cạnh xuống từ EXTI |
| `On_Falling_Edge()` | Ghi nhận khoảng thời gian giữa hai cạnh và quản lý trạng thái nhận |
| `TIM2_IRQHandler()` | Phát hiện kết thúc khung nhận và gọi hàm giải mã |
| `Decode_Frame()` | Giải mã các khoảng thời gian thành dữ liệu nibble và RAW |
| `Raw_To_Level()` | Chuyển đổi RAW thành giá trị thang đo 1–100 |
| `Raw_To_uT()` | Quy đổi RAW thành giá trị từ trường theo cấu hình |
| `Sensor_Update()` | Cập nhật dữ liệu cảm biến và gọi thuật toán RPM |
| `WheelSpeed_Update()` | Phát hiện chu kỳ quay, tăng bộ đếm và tính RPM |
| `WheelSpeed_Service()` | Xử lý trạng thái dừng và hiệu chỉnh vị trí baseline |
| `main()` | Khởi tạo hệ thống và chạy vòng lặp chính |

## 4. Nguyên lý thu nhận và giải mã tín hiệu

### 4.1. Phát Trigger

Hàm `Spc_Schedule()` kiểm tra chu kỳ phát Trigger và khoảng thời gian an toàn kể từ cạnh tín hiệu gần nhất.

Khi đủ điều kiện, `Send_Trigger()` thực hiện các bước:

1. Tạm khóa ngắt.
2. Tắt ngắt so sánh của TIM2 và ngắt EXTI trên đường tín hiệu.
3. Kéo đường tín hiệu xuống mức thấp trong thời gian cấu hình.
4. Nhả đường tín hiệu về trạng thái mức cao.
5. Xóa cờ ngắt và chuyển trạng thái nhận sang `RX_WAIT_E1`.
6. Cho phép lại ngắt EXTI.

### 4.2. Máy trạng thái nhận dữ liệu

Chương trình sử dụng bốn trạng thái:

- `RX_IDLE`: Không nhận khung dữ liệu.
- `RX_WAIT_E1`: Chờ cạnh đầu tiên sau Trigger.
- `RX_WAIT_SYNC`: Chờ khoảng thời gian đồng bộ hợp lệ.
- `RX_DATA`: Thu thập các khoảng thời gian của khung dữ liệu.

Các khoảng thời gian được lưu trong mảng `s_dt[]`. Khi bộ định thời phát hiện khoảng nghỉ kết thúc khung, chương trình gọi `Decode_Frame()`.

### 4.3. Giải mã nibble và RAW

Hàm `Decode_Frame()` sử dụng khoảng đồng bộ để ước tính đơn vị thời gian UT, sau đó quy đổi từng khoảng thời gian thành nibble có giá trị từ 0 đến 15.

Theo cách ghép dữ liệu hiện tại:

```c
main_raw = ((uint16_t)nib[1] << 8) |
           ((uint16_t)nib[2] << 4) |
           (uint16_t)nib[3];
```

Ba nibble được ghép thành giá trị RAW 12 bit.

Ví dụ minh họa:

| Nibble | Giá trị |
|---|---:|
| `nib[1]` | 8 |
| `nib[2]` | 1 |
| `nib[3]` | 4 |

Kết quả:

```text
RAW = 0x814 = 2068
```

Đây là ví dụ minh họa cách ghép nibble; dữ liệu thực tế phụ thuộc vào tín hiệu nhận được và cấu hình cảm biến.

**Lưu ý:** Phiên bản code hiện tại giới hạn giá trị nibble trong khoảng 0–15 nhưng chưa thực hiện đầy đủ kiểm tra CRC, trạng thái, tính hợp lý của dữ liệu hoặc xác thực khung trước khi cập nhật RAW. Vì vậy, cần kiểm tra độ tin cậy của dữ liệu trong quá trình thử nghiệm thực tế.

## 5. Chuyển đổi dữ liệu cảm biến

### 5.1. Chuyển RAW thành thang đo 1–100

Hàm `Raw_To_Level()` chuyển giá trị RAW thành giá trị nguyên theo cấu hình trong `main.h`.

Các tham số quan trọng gồm:

- `LEVEL_RAW_CENTER`: Giá trị RAW tại điểm trung tâm.
- `LEVEL_RAW_MAX`: Giới hạn RAW phía trên.
- `LEVEL_RAW_MIN`: Giá trị mức thấp được trả về tại điểm trung tâm theo logic hiện tại.
- `LEVEL_MAX`: Giới hạn mức đầu ra.

Phép chia làm tròn gần nhất được thực hiện bởi `Div_Round_Nearest()` nhằm hạn chế sai lệch do phép chia số nguyên.

Cách quy đổi cụ thể phụ thuộc vào các hằng số trong `main.h`. Cần bảo đảm các giới hạn và giá trị trung tâm phù hợp với cảm biến thực tế.

### 5.2. Quy đổi từ trường

Hàm `Raw_To_uT()` sử dụng giá trị RAW, mã zero và hệ số hiệu chuẩn để tính giá trị từ trường.

Kết quả được lưu trong:

- `g_sensor.field_inst_uT`: Giá trị từ trường theo đơn vị µT.
- `g_sensor.field_mT`: Giá trị từ trường theo đơn vị mT.

Độ chính xác của kết quả phụ thuộc vào hệ số hiệu chuẩn, điểm zero và đặc tính thực tế của cảm biến.

## 6. Thuật toán đếm vòng quay và tính RPM

### 6.1. Nguyên lý đếm vòng

Hàm `WheelSpeed_Update()` sử dụng `g_sensor.level` để phát hiện một chu kỳ quay.

Quy trình hiện tại:

1. Lần đầu nhận dữ liệu, chương trình thiết lập `baseline_level`.
2. Khi mức tín hiệu tăng ít nhất 1 đơn vị so với baseline, chương trình đánh dấu đã phát hiện chuyển động.
3. Khi tín hiệu quay về đúng giá trị baseline, bộ đếm `revolutions` tăng thêm 1.
4. Khoảng thời gian giữa hai lần hoàn thành chu kỳ được dùng để tính RPM.

### 6.2. Công thức tính RPM

Nếu một vòng quay tương ứng với một chu kỳ được phát hiện, tốc độ quay được tính bằng:

\[
RPM = \frac{60000}{T_{\text{vòng}}}
\]

Trong đó:

- \(T_{\text{vòng}}\): Thời gian của một vòng quay, tính bằng mili giây.
- 60000: Số mili giây trong một phút.

Ví dụ:

| Thời gian một vòng | RPM |
|---:|---:|
| 1000 ms | 60 |
| 500 ms | 120 |
| 250 ms | 240 |
| 100 ms | 600 |

Chương trình chỉ cập nhật RPM khi khoảng thời gian đạt `WHEEL_MIN_REV_INTERVAL_MS`.

### 6.3. Xử lý khi bánh xe dừng

Hàm `WheelSpeed_Service()` kiểm tra thời gian kể từ chu kỳ quay gần nhất để đưa RPM về 0 khi không còn phát hiện chu kỳ mới.

Chương trình cũng có cơ chế hiệu chỉnh baseline khi tín hiệu ổn định trong thời gian cấu hình.

Các tham số liên quan gồm:

- `WHEEL_MIN_REV_INTERVAL_MS`
- `WHEEL_STOP_TIMEOUT_MS`
- `WHEEL_STABLE_FRAMES`
- `WHEEL_BASELINE_IDLE_MS`

Các giá trị này được cấu hình trong `main.h` và cần được lựa chọn dựa trên tốc độ quay, độ nhiễu của cảm biến và chu kỳ cập nhật dữ liệu.

**Điều kiện quan trọng:** Thuật toán hiện tại giả định một chu kỳ tín hiệu được nhận diện tương ứng với một vòng quay. Nếu tín hiệu thay đổi nhiều lần trong một vòng, hoặc không trở về đúng baseline, kết quả đếm vòng và RPM có thể sai. Cần kiểm chứng giả định này bằng cách quay bánh xe với số vòng đã biết.

## 7. Các biến dữ liệu quan trọng

### `g_sensor`

Chứa dữ liệu cảm biến đã giải mã, bao gồm:

- `raw`: Giá trị RAW 12 bit.
- `level`: Giá trị đã quy đổi sang thang đo 1–100.
- `field_inst_uT`: Từ trường tức thời theo µT.
- `field_mT`: Từ trường theo mT.
- `raw_sub`: Giá trị RAW của kênh phụ nếu có.
- `decoded_frames`: Số khung đã giải mã.
- `triggers`: Số lần phát Trigger.
- `n_nib`: Số nibble được nhận diện.
- `last_ut_ns`: Giá trị UT quy đổi sang nano giây.

### `g_dbg`

Chứa dữ liệu phục vụ debug:

- `frames`: Số khung giải mã.
- `edges`: Số cạnh tín hiệu đã nhận.
- `nib[]`: Các nibble được giải mã.
- `dt[]`: Các khoảng thời gian đo được.
- `n_nib`: Số nibble trong khung.

### `g_wheel`

Chứa trạng thái thuật toán tốc độ quay:

- `current_level`: Mức tín hiệu hiện tại.
- `baseline_level`: Mức tín hiệu chuẩn.
- `candidate_level`: Mức tín hiệu đang được xem xét để hiệu chỉnh.
- `initialized`: Trạng thái khởi tạo.
- `moving_seen`: Đã phát hiện tín hiệu rời baseline.
- `revolutions`: Bộ đếm vòng quay.
- `rpm`: Tốc độ quay hiện tại.
- `last_rev_interval_ms`: Khoảng thời gian giữa các chu kỳ.
- `last_rev_time_ms`: Thời điểm hoàn thành chu kỳ gần nhất.
- Các trường trạng thái ổn định phục vụ hiệu chỉnh baseline.

Tên trường và kiểu dữ liệu thực tế cần đối chiếu với `main.h`.

## 8. Cấu hình và biên dịch

### Yêu cầu

- STM32F103C8T6.
- STM32Cube HAL và các tệp CMSIS tương ứng.
- Bộ công cụ biên dịch ARM GCC hoặc môi trường STM32CubeIDE.
- Các tệp `main.c`, `main.h` và các tệp khởi tạo hệ thống cần thiết của dự án.

### Các bước thực hiện

1. Tạo hoặc mở dự án STM32F103C8T6.
2. Thêm mã nguồn vào dự án.
3. Kiểm tra cấu hình chân GPIO, EXTI, TIM2 và xung nhịp.
4. Kiểm tra các macro cấu hình trong `main.h`.
5. Biên dịch và nạp firmware vào vi điều khiển.
6. Quan sát các biến `g_sensor`, `g_dbg` và `g_wheel` bằng debugger.
7. Đối chiếu kết quả RPM với số vòng quay thực tế.

Không nên sao chép riêng `main.c` vào một dự án mới mà chưa kiểm tra các định nghĩa trong `main.h`, tệp khởi tạo HAL, vector ngắt và cấu hình clock.

## 9. Kiểm thử và hiệu chỉnh

Để đánh giá độ ổn định của hệ thống, nên thực hiện lần lượt:

- **Kiểm tra tín hiệu:** Xác nhận số cạnh, khoảng đồng bộ và các khoảng thời gian trong `g_dbg.dt[]`.
- **Kiểm tra giải mã:** Theo dõi `g_dbg.nib[]` và `g_sensor.raw` khi thay đổi vị trí nam châm.
- **Kiểm tra mức tín hiệu:** Xác nhận `g_sensor.level` thay đổi đúng chiều và không dao động quá mức khi cảm biến đứng yên.
- **Kiểm tra bộ đếm:** Quay bánh xe đúng số vòng đã biết và so sánh với `g_wheel.revolutions`.
- **Kiểm tra RPM:** Thử ở nhiều tốc độ khác nhau và so sánh với tốc độ tham chiếu.
- **Kiểm tra dừng:** Dừng bánh xe ở các vị trí khác nhau để đánh giá cơ chế timeout và hiệu chỉnh baseline.

Nếu bộ đếm tăng nhiều lần cho một vòng, cần kiểm tra điều kiện xác nhận chu kỳ. Nếu RPM dao động khi tốc độ thực tế ổn định, cần kiểm tra độ nhiễu của `level`, thời gian cập nhật khung và cách xác định thời điểm hoàn thành vòng quay.

## 10. Hướng phát triển

Các hướng cải tiến có thể cân nhắc:

- Bổ sung kiểm tra tính hợp lệ của khung trước khi cập nhật dữ liệu.
- Xác nhận chu kỳ quay bằng ngưỡng, hysteresis hoặc trạng thái tín hiệu phù hợp với cảm biến.
- Tách biệt số vòng tích lũy và trạng thái RPM hiện tại.
- Bổ sung lọc nhiễu và kiểm tra tính hợp lý của khoảng thời gian quay.
- Kiểm thử tràn bộ đếm và các trường hợp tín hiệu bị mất.
- Hiệu chuẩn RAW, baseline và hệ số chuyển đổi từ trường theo phần cứng thực tế.

## 11. Giấy phép

**Trạng thái dự án:** Phiên bản phát triển phục vụ thu nhận dữ liệu cảm biến, giải mã tín hiệu, thử nghiệm đếm vòng quay và tính RPM trên STM32F103C8T6. Độ chính xác và độ ổn định cuối cùng cần được xác nhận bằng kiểm thử trên phần cứng thực tế.
