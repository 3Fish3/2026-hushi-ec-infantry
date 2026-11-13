# 正式 APP 高频只读采集（0x37/0xB7）

## 范围

基于 `infantry_sweep_capture_source_v1.zip` 的 56 字节分页版本接入，ZIP SHA256 为 `9cf22bad97e8700a2918170ce63eb809e51dbf637896564141d70f328740b2f6`。不采用包中的 `patches/apply_patch.py`、`tools/infantry_sweep_receive.py`、48 字节流式协议或测试 stub。

本功能只记录并读取正式 APP 当次控制数据：不启动 Sweep，不设置目标，不取权，不修改 PID/寄存器，不使能电机。记录器在每次 `ec_board_app_commit()` 完成后追加一条，名义周期 1 ms；实际速率须由 MCU 时间戳验证。采集结束只停止 PC 请求，不会停止电机，也不会撤销另行启动的 Sweep。

## 固件与串口

- 固件：`build/infantry-app-arm-Release/boards/infantry_f407/infantry_app.elf`，正式 APP，而非独立 capture 角色。
- USART1，PA9 TX / PB7 RX，C 板外壳 UART2，**921600 8N1**。
- 一条已有 DMA 接收管线和共享 512 字节发送队列；不新增物理串口。
- SWD DAP 序列号：`332107148313`。
- 串口 DAP：`/dev/serial/by-id/usb-Horco_Horco_CMSIS-DAP_332107148384-if02`，不硬编码 ttyACM 编号。
- 切回正式 APP 后不再使用独立采集角色的 115200。烧录、复位、供电与动力试验仍按当次授权操作。

## 采样内容与有效性

每条 56 字节，按 `<III10fHBB` 逐字段小端序列化：

| 字段 | 含义 |
|---|---|
| sequence / timestamp_us / reference_epoch | MCU 记录序号、32 位微秒时间与 yaw 参考代次 |
| target_heading_rad / heading_rad / heading_rate_rad_s | 实际采用的惯性航向目标、实测航向及其变化率 |
| dm_position_rad / dm_velocity_rad_s | MIT 原始解码电机位置、速度，不是 p_m/xout，不自动展开 |
| torque_command_nm / torque_feedback_nm | 本周期 CAN 接口已接纳的电机力矩命令、DM 报告反馈；都不是独立力矩传感器测量 |
| speed_target_rad_s / speed_p_nm / speed_i_nm | 真实角度环输出与速度环 P/I 诊断，不重算控制器 |
| flags / reason / feedback_age_ms | 各字段有效位、执行原因、反馈年龄；年龄255表示不可用或超出可表达范围 |

flags 从 bit0 到 bit6：目标有效、航向有效、DM反馈新鲜、DM使能、力矩提交、PID诊断有效、yaw执行。无效字段中的 0 不能当成有效测量。

`dm_velocity_rad_s` 是电机相对运动速度，`heading_rate_rad_s` 是 IMU 水平航向变化率；底盘旋转时两者不是同一个量。P/I 取自速度 PID 原输出坐标；力矩命令还经过既有补偿、限幅和方向映射，不能不统一坐标就直接相减。

## 缓存与传输

- 128 条固定环形缓存，名义 1 kHz 下约保留最近128 ms，满后覆盖旧记录。
- 外层复用工程真实 `UartFrame`/CRC16/转义，请求type0x37，应答type0xB7。
- 请求体 `<BBI>`：schema=1、op=0状态/1分页、cursor。
- 应答头 `<BBBBIII>`：schema、0xB7、op、count、first、next、earliest；最多4条，满页240字节。
- 独立四槽 `sweep::ReplyQueue` 保序分页，不采用“同类仅保留最新值”的状态应答语义，也不受默认50ms低频页限速；普通 APP 应答先提交，采集每控制周期至多提交一页。主机最多四页在途以隐藏逐页往返延迟，不能向尚未产生的序号盲目请求。已进入FIFO/DMA的字节不能被后来请求抢占。
- 高频分页请求活跃时自动暂停同口 CAL/GAP 文本，最后请求250ms后按原观测开关自动恢复；不发送参数写入，不改变控制器或目标。
- 典型满页持续传输约62.5 KB/s；最坏转义和其它业务可超过921600的物理容量。128条缓冲不能保证任意PC调度或无限长无损采集，必须以序号与覆盖统计验收。

## 使用

先确认正式 APP 已按当次授权烧录、右拨杆停止挡/遥控器关闭、物理急停就位。不要同时运行 GUI、旧扫频脚本或其它占串口进程。

在仓库根目录执行：

```sh
SERIAL=/dev/serial/by-id/usb-Horco_Horco_CMSIS-DAP_332107148384-if02
mkdir -p .tmp/highrate
.venv/bin/python robots/infantry/host/infantry_sweep_capture.py \
  --serial "$SERIAL" --baud 921600 --seconds 3 \
  --output .tmp/highrate/stopped-01.csv \
  --elf build/infantry-app-arm-Release/boards/infantry_f407/infantry_app.elf
```

CSV 与摘要必须新建，不覆盖旧数据。摘要记录实际 MCU 采样速率/时间覆盖、有效性统计及失败原因。序号覆盖、超时、参考变化或窗口未收齐时保留已收内容并报告失败，不把完整传输与有效 Sweep 试验混为一谈。停止态目标/执行/PID 有效位可能为0，这是正常停机数据，不作为动态频响使用。

本客户端只发送0x37状态/分页，不发送HELLO、授权、保活或参数写。若原控制依赖另一个PC进程保活，独占串口会让其按既有规则失去主机持权；不能靠本采集程序维持运动。

## 构建和离线验证

```sh
cmake --build build/host -j4
ctest --test-dir build/host --output-on-failure -j4
cmake --build build/infantry-app-arm-Release --target infantry_app -j4
```

测试使用真实工程分帧/CRC与 C++/Python 互操作，不使用 ZIP 的 stub 证明兼容性。所有合成夹具明确标记 SYNTHETIC。构建成功不能代替停止态串口吞吐、时序和新镜像的 RAM/栈余量验证；实际结果单独记入本次交接记录。
