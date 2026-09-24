# Wing–Tail flow: khung tính toán và hình học cần đọc cho seed DATCOM

## Phạm vi commit

Bổ sung `WingTailFlowField : ILocalFlowField` cho `HorizontalStabilizer`. Đây là khung dòng khí quasi-steady, trung bình trên đuôi, trong miền góc tấn tuyến tính. Chưa phải bộ seed T-6C, chưa số hóa đồ thị DATCOM. Mặc định epsilon=0, eta=1 tương đương không có wake. Không tự gắn hệ số giả vào factory toàn máy bay.

Không mô hình hóa propwash, ground effect, stall, thời gian truyền wake hoặc phân bố wake theo sải. Hệ số được xác định cho một hình học và miền Mach; vị trí truyền vào interface không làm tự thay đổi hệ số. Khi đổi vị trí đuôi cần tính lại seed. Seed PC-9M/T-6C sơ bộ hiện đã có trong [báo cáo seed đuôi](T6C_HORIZONTAL_TAIL_DATCOM_SEED.md); checklist dưới đây lưu cách đọc hình học và các mục cần kiểm chứng.

## Luật tính và quy ước

Tất cả góc dùng radian, gradient dùng rad/rad. Hệ body FRD: x trước, y phải, z xuống. Vận tốc là máy bay tương đối với không khí.

| Tham số cấu hình | Ý nghĩa | Giá trị mặc định |
|---|---|---:|
| `referenceBodyAlphaRad` | Góc tấn body tại điểm tham chiếu | 0 |
| `referenceDownwashRad` | Downwash tuyệt đối tại alpha và flap tham chiếu | 0 |
| `downwashGradientPerRad` | d epsilon / d alpha_body | 0 |
| `referenceFlapRad` | Góc flap tham chiếu | 0 |
| `flapDownwashGradientPerRad` | d epsilon / d flap; xấp xỉ cục bộ cần seed riêng | 0 |
| `dynamicPressureRatio` | eta=q_wake/q_free tại omega=0, cùng mật độ | 1 |

`epsilon = epsilon_ref + epsilon_alpha*(alpha_body-alpha_ref) + epsilon_flap*(flap-flap_ref)`.
Alpha được tính từ atan2(w,u) của vận tốc không khí hiện tại ở CG; không cộng incidence cánh lần nữa vào biến body alpha. Incidence/twist/camber của wing phải được phản ánh trong epsilon_ref lúc tạo seed. Không đọc trạng thái hoặc tải cánh từ bước trước.

`V_wake = sqrt(eta)*[cos(epsilon)*u + sin(epsilon)*w, v, -sin(epsilon)*u + cos(epsilon)*w]`.
`delta_V = V_wake - V_free` được trả qua interface. Với eta=1, phép quay bảo toàn tốc độ và giảm góc dòng khí một lượng epsilon. Vector delta_V không phải vector vận tốc không khí tuyệt đối; dấu z phải phù hợp quy ước aircraft-relative.

Đuôi hiện có tính `V_tail = V_free + omega cross r_tail + delta_V`.
Vì vậy `V_tail = V_wake + omega cross r_tail`: tốc độ do quay thân không bị quay hoặc nhân sqrt(eta) và chỉ cộng một lần. Khi omega khác 0, q_tail/q_free không nhất thiết bằng eta. Mỗi lần component được gọi trong RK4 đều tính lại từ EvaluationContext; không cache và không có trạng thái wake ẩn.

Sau đó giữ nguyên alpha_eff=atan2(w_tail,u_tail)+incidence-alpha_zero+tau_e*elevator, CL=a_h*alpha_eff, CD=CD0+k*CL^2. Moment tại CG vẫn là M_AC+r_tail cross F_tail. Không sửa LoadAccumulator, RigidBody6DOF hoặc RK4.

Không trừ epsilon lần thứ hai trong đuôi, không nhập a_h đã nhân (1-epsilon_alpha), không nhân eta lần thứ hai vào lực. Trong trạng thái tĩnh, derivative CL_tail theo alpha_body bằng a_h*(1-epsilon_alpha).

## Gắn vào đuôi ngang

```cpp
#include "trainer_aircraft/components/WingTailFlowField.hpp"
#include "trainer_aircraft/components/stabilizers/VATC_HorizontalStabilizer.hpp"
#include <memory>

// tailConfig: cấu hình đuôi đã được điền từ seed độc lập.
// flowConfig: các hệ số downwash/wake được điền sau khi có hình học.
auto wingTailFlow = std::make_shared<trainer_aircraft::WingTailFlowField>(flowConfig);
auto horizontalTail = std::make_unique<trainer_aircraft::HorizontalStabilizer>(
    tailConfig, wingTailFlow);
aircraft.addLoadComponent(std::move(horizontalTail));
```

Một object flow biểu diễn trung bình tại một đuôi cụ thể, không phải field không gian dùng chung cho các bộ phận khác. `evaluateDetailed` cung cấp epsilon, V_wake và delta_V để kiểm tra. Flap dùng góc thực tế trong controls; caller chịu trách nhiệm giới hạn hành trình. Hệ số flap tuyến tính chỉ phù hợp lân cận cấu hình đã seed; split flap lớn cần thay bằng bảng sau này.

## Reference DATCOM để tạo seed

AFWAL-TR-83-3048, bản PDF 3134 trang đã dùng cho Main Wing:

| Nội dung | Trang in / thứ tự trang PDF | Áp dụng |
|---|---|---|
| Subsonic downwash Method 1 | 4.4.1-3 đến -6 / 1205–1208 | Đường cong downwash, vị trí và sải đuôi; kiểm tra điều kiện áp dụng |
| Subsonic downwash Method 2 | 4.4.1-6 đến -7 / 1208–1209 | Gradient tuyến tính; Eq. 4.4.1-h, hiệu chỉnh Mach Eq. 4.4.1-i |
| Các hệ số KA, K_lambda, KH | Fig. 4.4.1-69a, -69b, -70 | Từ AR, taper và khoảng cách/cao độ đuôi |
| Subsonic dynamic-pressure ratio | Bắt đầu 4.4.1-9 / 1211 | Wake và suy giảm áp suất động; giá trị ở mặt phẳng đối xứng cần cân nhắc khi đại diện cả đuôi |

Method 2: epsilon_alpha_low = 4.44*[KA*K_lambda*KH*sqrt(cos(Lambda_c/4))]^1.19. Công thức/đồ thị này được dùng ngoài runtime để tính seed, chưa được triển khai trong class. Method 2 không tự cung cấp epsilon_ref. Các công thức DATCOM cho cánh straight-tapered cần hình học tương đương đối với cánh PC-9M đang chia panel. Không coi hệ số từ đồ thị plain/slotted flap là dữ liệu split flap đã xác thực.

## Quy ước đọc bản vẽ

Ưu tiên gửi tọa độ điểm trên cùng một datum hơn là ước lượng riêng cánh tay đòn. Có thể dùng x dương về đuôi và z dương lên trên bản vẽ, nhưng phải ghi rõ; khi nhập code sẽ chuyển về body FRD. Ghi đơn vị mm/m, tỉ lệ bản vẽ và kích thước dùng để chuẩn hóa. PC-9M là proxy cho T-6C, ghi rõ nguồn của từng số đo.

Chord gốc cần ghi đo tại centerline hay giao tuyến thân; chord bao gồm cả elevator trung hòa. Sải full gồm hai bên và phần chiếu xuyên thân; diện tích exposed được ghi riêng. Không dùng mép vỏ thân làm datum thay cho centerline mà không ghi chú.

## Checklist hình học cho người đọc bản vẽ

| Nhóm / input cần cung cấp | Cách đo hoặc mô tả | Dùng để tính |
|---|---|---|
| Sải đuôi ngang b_h | Tip–tip theo hình chiếu bằng; ghi thêm sải ngoài thân nếu có | AR_h, tỉ số b_h/b_w |
| Các station đuôi | y, x_LE, chord tại centerline, mép thân, từng điểm gãy, tip | Diện tích, taper, sweep và MAC bằng tích phân |
| Diện tích S_h nếu bản vẽ ghi | Full projected planform, gồm elevator trung hòa; diện tích ngoài thân riêng | Kiểm tra diện tích tích phân, diện tích ướt |
| Sweep nếu có số ghi | Nêu LE, 1/4 chord hay đường bản lề; không trộn định nghĩa | Hình học và lift slope |
| Incidence đuôi i_h | Góc dây cung gốc so với datum dọc; dương khi LE cao hơn TE | Góc tấn đuôi |
| Twist đuôi | Incidence gốc/tip hoặc theo station; nếu không thấy ghi chưa rõ | Alpha_zero hiệu dụng |
| Dihedral đuôi | Góc, station bắt đầu đổi góc; nếu phẳng ghi 0 | Vị trí và hình học hiệu dụng |
| Airfoil đuôi | Tên profile; hoặc t/c, độ cong, vị trí bề dày/camber tối đa | Section lift slope, alpha_zero, Cm0, drag |
| Tọa độ đuôi so với datum | x,z của LE gốc và các điểm đủ dựng đường dây cung; quarter-MAC nếu có | AC đuôi và PositionWrtCG |
| Tọa độ cánh cùng datum | x,z của LE gốc cánh, hướng dây cung gốc; quarter-MAC nếu có | Khoảng cách wing–tail theo DATCOM |
| Elevator span | y_in, y_out mỗi bên; ghi khoảng ngắt ở thân | Tỉ lệ diện tích và hiệu quả elevator |
| Elevator chord/hinge | x_hinge hoặc c_e tại gốc, điểm gãy, tip elevator; ghi có phần cân bằng phía trước hinge | c_e/c_h, sweep hinge, diện tích và hiệu quả mặt lái |
| Hình học khe/balance/tab | Khe kín/hở, horn/overhang, trim tab và kích thước nếu thấy | Chọn mô hình elevator; phân biệt góc elevator với tab |
| Giới hạn elevator | Up/down và quy ước dấu từ tài liệu điều khiển nếu bản vẽ không có | Saturation điều khiển |
| Bề rộng/cao thân tại đuôi | Giao tuyến thân–đuôi, phần bị che | Exposed area và tương tác thân–đuôi |
| Flap cánh chính cần bổ sung | c_f/c(y), panel boundaries và góc lệch thực tế | Downwash increment theo flap |
| CG của trường hợp FDR | x,z cùng datum; lấy từ loading/flight data, không suy ra từ outline bản vẽ | Cánh tay đòn tải về CG |

Có thể chỉ gửi bảng station `(ten_tram, y, x_LE, c, z_LE, incidence, x_hinge)` và hình nhìn cạnh có datum: từ đó suy ra S_h, MAC_h, AR_h, taper và sweep; không cần đọc lại tất cả các đại lượng dẫn xuất.

l_H trong DATCOM đo song song dây cung gốc cánh giữa quarter-MAC wing và quarter-MAC tail. h_H đo theo quy ước mặt phẳng dây cung gốc wing của DATCOM, không đồng nhất máy móc với delta z_body. Khi có incidence wing, cần biến đổi tọa độ trước khi tra KH. Cánh tay đòn CG–tail khác l_H wing–tail.

Những dữ liệu đã có của wing (S=16.28 m2, b proxy=10.124 m, twist/incidence, mặt bằng hiệu dụng) được tái sử dụng từ wing seed; chỉ cần xác nhận datum và vị trí wing–tail, không cần đo lại toàn bộ cánh. Không lấy b_h hoặc vị trí đuôi từ kích thước wing.

## Kiểm tra trong mã nguồn

Bộ test stabilizer kiểm tra identity flow, dấu downwash, bảo toàn tốc độ khi eta=1, tỉ số q tại omega=0, slope a_h*(1-epsilon_alpha), flap theo trạng thái hiện tại, omega cross r cộng một lần, chuyển moment về CG, zero-speed và loại bỏ eta không hợp lệ. Bộ test RK4 hiện có kiểm tra interface được đánh giá tại từng stage. Đây là kiểm tra implementation, chưa phải xác thực T-6C hay FDR.
