# 高频只读 APP 采集接入与实板验证

- 日期：2026-11-13
- 仓库：`/home/kekeyuyu/projects/步兵/infantry`，`main`，基线 `d4ab270b53c38fdf6999bf6e8201c36c61e5a92d`。
- 本轮开始工作区 clean，当前新增采集代码和测试未提交；未改索引、未提交、未推送。
- 基线快照：`.tmp/highrate-before-20261113-110930/source.tar.gz`；同目录 manifest 记录原镜像哈希。

## 已完成

1. 审阅用户 ZIP：`/home/kekeyuyu/Downloads/infantry_sweep_capture_source_v1.zip`，SHA256 `9cf22bad97e8700a2918170ce63eb809e51dbf637896564141d70f328740b2f6`。只采用0x37/B7、56字节的Recorder版本，不执行包内补丁，不引入另一代0x38/B8/B9工具或stub。
2. `infantry_execution.hpp/.cpp` 仅增加当次真实PID诊断和逐周期清有效标记，不改变PID、力矩、目标或停止行为，保留原CRLF。
3. `infantry_sweep_capture.hpp` 新增128条固定环形记录器；`board_app.cpp` 在当次commit后记录，新增0x37只读分流；复用现有USART1唯一DMA和收发队列。
4. 首版采用单槽采集ReplyMux，上板采样约1000.138Hz，但同步分页排空过慢；`.tmp/highrate/stopped-01.csv.summary.json` 正确报告失败：204条、0.202972s后落后并覆盖2条。失败原始数据保留，不作成功试验。
5. 随后换成四槽分页FIFO `sweep::ReplyQueue`，支持主机最多四页在途；每周期最多提交一页、普通APP应答先提交。高频请求活跃时自动暂停CAL/GAP文本，最后请求250ms后恢复，不发参数写。
6. C++记录器边界、真实CRC/转义和FIFO保序测试通过；Python模块含假串口、时间覆盖、缺口和互操作测试。流水线完成后的最终全量主机测试 **44/44通过**，退出0，3.96s；日志 `.tmp/highrate-final-tests.log`。
7. 实板停止执行状态的1秒与3秒连续采集均完成。3秒结果：3012条、MCU跨度3.011031s、实际999.990Hz，周期中位1000us（656–1563us），序号覆盖0、丢应答0、CRC拒绝0、时间gap0。CSV实际行数与摘要一致；航向有效3012/3012，DM反馈新鲜2951/3012（97.9748%）。无效DM样本明确保留有效位，不可作为有效反馈分析；未把记录频率称为每个传感器都有独立1kHz新样本。
8. 采集期间 dm_enabled/torque_submitted/executing/PID有效位全为0，力矩命令全为0；目标有效位全为0。这是采集链路验收，不是Sweep动态试验或PID调参结果。
9. 采集后业务HELLO/status/参数读取仍正常：source=none、authority=false、各域execute=false、sweep幅值0、补偿0、观测开关1。PID维持固件默认angle Kp2、speed Kp0.35，未写入。状态保存 `.tmp/highrate/postcapture-status.json`。

## 当前已烧录镜像

- 正式APP：`build/infantry-app-arm-Release/boards/infantry_f407/infantry_app.elf`。
- 修正版ELF SHA256：`567ca77b87b80d421d3ee3b80f170427f98c93a68c31f5055de73dafdbafb36b`。
- ARM构建/镜像审计退出0，RAM 116080/131072 B（88.56%），Flash224224 B。
- 用户通过确认框明确确认：底盘固定、急停就位、遥控器关闭或停止挡、电机保持失能，继续烧录。
- SWD锁定序列号 `332107148313`；11:42左右修正版写入成功，**完整224224字节Flash校验通过，包含.data在0x0802CDC0的40480字节初始化数据**；`.text matched`、GDB退出0、reset run完成。
- 不再用只按节VMA筛选的校验图像：那会遗漏.data。脚本按程序头把节VMA映射至Flash LMA，并按擦除值填节间空隙。
- 日志：当前会话临时目录 `highrate-flash-20261113-114209-{openocd,gdb}.log`；项目 `.tmp/highrate-flash-result.json` 保存成功摘要。

## 硬件与权限

- 串口固定 `/dev/serial/by-id/usb-Horco_Horco_CMSIS-DAP_332107148384-if02`，**921600**；不要依赖ttyACM编号，本轮实查它指向ttyACM2。
- 首次烧APP后HELLO/status/参数读取成功：identity=infantry_app、source=none、authority=false、所有execute=false、sweep幅值0、补偿0、angle Kp2、speed Kp0.35；manual_stop=false且此前DR16无反馈，因此执行停用不是人工停止标志的证据。现场安全条件由用户明确确认。
- 本轮没有授权或目标/增益写入、没有使能、没有正弦/动力试验。高频记录只读，不保活主机控制权，也不停止电机。
- 不自动再次提交或推送。采集结束后仅可读状态，后续动力试验需单独授权。

## 收敛状态与产物

用户确认成功后要求直接收敛；本次高频只读采集已完成，不再扩展功能或继续试验，不自动提交/推送。

- 1秒数据：`.tmp/highrate/stopped-pipeline-01.csv`，对应 `.csv.summary.json`；1008条、1.007098s、999.903Hz，覆盖/丢应答/CRC均0。
- 3秒数据：`.tmp/highrate/stopped-pipeline-03s.csv`，对应 `.csv.summary.json`；3012条、3.011031s、999.990Hz，覆盖/丢应答/CRC均0。
- 首次失败样本 `.tmp/highrate/stopped-01.csv` 与摘要保留，不覆盖、不计入成功采集。
- 当前源码构建出的ELF哈希与最终已烧镜像一致；本轮未暂存、提交或推送。
- 使用说明：`docs/boards/infantry_f407/APP高频只读采集.md`。
- 下一阶段如需Sweep有限时长激励/动态PID整定，由用户另行指定；本次未进行。
