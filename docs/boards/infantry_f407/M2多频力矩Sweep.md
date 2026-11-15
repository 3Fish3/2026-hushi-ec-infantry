# M2 受监督多频力矩 / 角度正弦 Sweep

## 当前实现

> 2026-11-15：交接所列五项 Sweep 软件缺陷已在 `tzh` 修复并补回归，详见[修复记录](../../handoffs/2026-11-15_tzh-sweep-fixes.md)。新采集协议为 schema3，固件与客户端须配套更新；尚未进行本轮烧录或实机验收，软件测试通过不代表云台响应与底盘问题已经消除。

两种试验共用同一个有限时长引擎，均须在**左下＋右下维护模式**下显式 START，运行期间互斥：

- **torque（op1）**：保持当前世界航向，在PID/摩擦补偿及方向映射后叠加电机侧正弦力矩，按原输出限幅裁剪。`requested_excitation_nm` 与 `applied_excitation_nm` 都在电机坐标，后者仅表示CAN接口接纳的注入增量，不是独立测力结果。
- **angle（op2）**：START 捕获当前世界航向作为基准，每控制周期通过 APP 正常目标入口提交 `基准 + A·包络·sin(2πft)`，经过原角度环、速度环和 MIT 纯力矩路径。两项 excitation_nm 均为 0，实际闭环力矩仍记录在 `torque_command_nm` 中。维护心跳持续续权，主机不再重复发送基准目标。

原 `0x0A10/0x0A11` 无限正弦参数与 `InfantryCycle::set_sweep()` 已退役；旧参数ID不再枚举，写入返回 unknown_param，不能靠写幅值和切拨杆自动起振。PID参数ID不变。

- 控制1kHz；schema3记录**250Hz**，每条74字节，每页3条、238字节载荷。
- 保留M1的四槽分页FIFO和同一USART1唯一DMA/队列，采集缓存128条、约512ms。
- USART1/921600/8N1，串口DAP固定by-id `usb-Horco_Horco_CMSIS-DAP_332107148384-if02`。SWD使用另一只DAP `332107148313`，不得交换角色或只靠ttyACM编号选择。
- 0x37/B7只读采集，0x49是带session/sequence的维护启停消息，二者不是同一类操作。
- 1–6个0.2–5Hz严格递增频点；每频3–12周期，首尾各一周期线性淡入/淡出；计划总时长≤120s。torque软件幅值上限0.03Nm，angle上限0.2rad（约11.46°），不把这些值当作硬件安全能力证明。
- 0x49载荷为 `[version=3,type=0x49,session:u32,seq:u32,op:u8,count:u8,cycles:u8,amplitude:f32,hz:f32×count]`；op0取消（count=0），op1幅值单位Nm，op2幅值单位rad。
- schema3在原72字节末尾追加 `sweep_phase:u8`、`sweep_stop:u8`，完整小端布局为 `<III10fHBBI3fBB`；旧字段偏移不变。bit7表示running，bit8表示完成/中止锁存，bit9 `sweep_angle` 标识角度模式（终态保留，清锁存后清除）。schema1/2客户端不兼容，不能把旧数据按新布局解释。
- PID、IMU标定、机械齿比和部署限幅不自动改写。

## 控制资格与终态

右拨杆中挡仍是人工停止；**左下+右下**是维护资格组合，本身不发运动目标。必须持续收到该组合的新鲜DR16帧，且串口维护会话显式持权、存在有效惯性保持目标。

START被接受后，若DM尚未使能，进入arming并通过既有使能流程等待新鲜确认，500ms内未确认则中止；确认后才开始有限时长注入。反馈、IMU、目标、参考代次、遥控维护资格、速度/偏差、控制时步或CAN提交异常均可结束本次试验。现有速度保护为1.5rad/s、偏离START航向保护为0.35rad。角度计划在主机打开串口前和MCU受理时都检查每频点 `2πfA≤1.5rad/s`，数值先按线上float32量化；首尾整周期线性包络的峰速也满足这一上界。该校验不提高实测速率保护，也不保证实际闭环一定不超速。

`sweep_phase`：0 idle、1 running、2 complete、3 aborted、4 arming。`sweep_stop`：0 none、1 cancelled、2 authority、3 feedback、4 reference、5 speed、6 deviation、7 clock、8 can_error、9 invalid_plan。`reason` 和报告的 `terminal_reason` 仍是 APP 执行原因，不能替代引擎停止原因；报告另存 `terminal_phase/terminal_stop`。正常完成必须明确收到 phase=2、stop=0，接近预计结束时间的 aborted 仍报告失败。

**完成、中止、取消后都撤输出并锁存，不恢复旧目标保持力矩。** 新的显式START可以重新武装，避免“必须已使能才能START、锁存又不许使能”的互锁。总线无法发送时，disable请求不等于物理停机已实现，仍必须保留物理急停。

客户端退出时尝试取消、清全部目标、撤权，并核对新鲜Disabled反馈。丢失START ACK也按“可能已启动”执行收尾；收尾未确认必须报告失败。不要用关掉采集窗口替代停止。

### 用遥控器退出试验并恢复普通控制

1. **右拨杆拨中挡停止**。程序必须收到新鲜停止帧，并在无持权、无执行时取得新鲜 `Disabled` 反馈，才将已结束的 M2 状态清为 idle；没有停机确认就保留锁存。遥控失联后的停止锁存不算新鲜停止操作。
2. 退出时一并撤销旧正弦试验及 yaw 保持角，仍处于停止态，不直接发 enable，也不自动启动下一次试验。
3. **摇杆回中，左拨杆上挡选择本地，右拨杆上挡普通运行**。首次重新接管按当前有效 IMU 航向建立新目标，随后右摇杆左右调整、回中保持；右中挡随时停止。

不需要复位主控、重新连接PC或重写PID。RAM中的在线增益保留（例如已写入的速度Kp=0.50），但不会将其固化为开机默认；烧录或复位后仍需按原参数流程回填。只拨运行挡、反馈过期或电机仍Enabled均不能绕过停止确认。新固件本身仍须另行授权烧录；上述恢复遥控路径目前通过主机合成闭环验证，尚未上板验证。

## 使用（须每次取得对应动力试验授权）

1. 固定机构、急停就位，先右中挡，确认正式M2镜像身份与无执行状态。
2. 从status(0)读取即时有效的当前航向；操作员确认它就是需要保持的方向，不使用烧录前值或任意0。
3. 切左下+右下并确认维护资格。只允许一个程序占用串口。
4. 首轮经授权的低幅单频命令示例：

```sh
SERIAL=/dev/serial/by-id/usb-Horco_Horco_CMSIS-DAP_332107148384-if02
mkdir -p .tmp/m2-trial
.venv/bin/python robots/infantry/host/infantry_sweep_trial.py \
  --serial "$SERIAL" --baud 921600 --arm \
  --target-heading <即时读取并确认的航向rad> \
  --frequencies 0.5 --cycles 3 --amplitude-nm 0.01 \
  --output .tmp/m2-trial/new-trial.csv \
  --elf build/infantry-app-arm-Release/boards/infantry_f407/infantry_app.elf
```

`--arm`会申请维护权、下发保持目标并启动动力激励，不是预览选项。幅值/频率/时长变化及后续多频试验须按授权范围执行，不能因首轮成功自动升级。输出CSV和`.report.json`不得覆盖旧文件。

角度正弦使用同一客户端，把上述参数替换为 `--mode angle --amplitude-rad 0.05 --frequencies 0.5 --cycles 6`，并使用新的输出文件名（例如 `.tmp/m2-trial/new-angle.csv`）；不要同时提供 `--amplitude-nm`。默认仍为torque。单位混用、缺幅值或越界在打开串口前拒绝。

结束后右拨杆回中挡；看报告的`complete`、活性记录数、MCU时间覆盖与`disabled_confirmation.confirmed`。torque报告给中心化航向/速度RMS及请求/提交注入力矩统计，包含淡入淡出。angle报告按线上float32频点身份分组，剔除每频首尾各一个淡入/淡出周期，再对目标和实测航向做同频正弦拟合，给出幅值、增益、滞后角与残差。

angle的 `usable` 表示稳态数据与增益可用，要求至少两个稳态周期，并计入250Hz采样边界的量化误差；3周期计划不满足，建议至少4周期。零响应保留近零增益，使用 `phase_identifiable` 单独标识相位是否可辨识；不可辨识时 `phase_deg/lag_deg` 为null。相位判定基于独立同方差残差模型的近似95%幅值不确定度，并非真实噪声与闭环模型的完整统计验证。**报告不自动改PID，也不是完整带宽或最优PID结论。**

## 首轮已验证的范围

2026-11-13：修正版46项主机测试通过，ARM镜像审计通过，完整Flash校验通过；首轮0.5Hz/0.01Nm/3周期约6s实车试验完成。1500条活性记录、实际249.996Hz、序号无缺口、注入峰值0.01Nm，航向峰峰约0.141°。结束后新鲜DM反馈确认失能、无持权、无域执行。

这只验证了低幅单频激励、采集和自动结束链路；未验证多频实车、提高幅值、完整频响拟合或PID效果。镜像/数据哈希和详细产物见[本轮记录](../../handoffs/2026-11-13_m2-sweep-integration.md)。M1的1kHz历史验收与M2的250Hz抽样必须分开描述。
