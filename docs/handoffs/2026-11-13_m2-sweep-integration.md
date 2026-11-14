# M2 Sweep 接入、修复与实板检查

- 仓库 `/home/kekeyuyu/projects/步兵/infantry`，分支main，HEAD仍为0d0597e；M2修改未提交/未推送。
- 基线恢复快照 `.tmp/m2-repair-before-20261113-121257/source.tar.gz`（保存当时尚未完成的M2源码）。

## 关键修复

- 包基于另一套M1格式，未直接运行overlay。保留现有四槽分页FIFO；记录升级schema2、72字节、每页3条、128条缓冲，正式APP控制仍为1kHz，**M2记录为250Hz**。
- 修复此前遗漏的 `board_app.cpp` 0x49请求分流与M2 run_id/频率/请求和实际提交注入量记录。旧45项测试仅通过不表示这些板级路径接通。
- `InfantryApp/InfantryAuthority` 对步兵启用持续物理维护资格：左下+右下映射，遥控失联或退出组合撤维护；原非步兵仲裁回归保持。
- `sweep::Engine` 终态complete/aborted后强制执行器走现有disable并锁存。新显式START可进入arming，允许原使能流程重新确认，500ms内未确认则中止；不再以“仅停止注入但保留旧目标力矩”规避互锁。
- 力矩注入先映射到电机坐标，再叠加、按映射后原限幅裁剪；requested/applied/最终电机命令单位与符号统一，torque_direction=-1也有实际执行链测试。
- CAN健康取既有发送结果，队列拒绝中止；手动停止、反馈/IMU/目标/参考失效均经保护撤输出。
- 保留正常PID参数写入权限，不擅自把全部参数改为仅维护可写。旧角度目标sweep保留用于兼容，本次不启用。
- 主机 `infantry_sweep_trial.py` 改为唯一串口所有者、串行匹配应答，补齐exchange。先验证目标贴近当前航向，再申请维护权，持续刷新权与目标；15ms请求超时小于100ms目标TTL。START ACK丢失也尝试取消/清目标/撤权；终态后确认新鲜Disabled反馈。未自动改PID。

## 验证

- 最终主机构建及CTest：**46/46通过**，3.87s；`.tmp/m2-repair-host.log`。
- 实际Cycle/App/Executor/CAN合成测试覆盖不在组合不能维护、掉线撤权、±方向注入、完成/取消撤输出与显式再武装；协议新增取消权限/变更重放测试；Python真实分帧假设备试验10项通过。不以这些合成结果代替实车效果。
- ARM APP镜像审计通过；RAM118216/131072B（90.19%）、Flash230136B。
- 当前已烧ELF SHA256：`1d58c8a6899c8ca500aa3754e6ef19a580a21f36e87b1514bf88c57db3389a11`。
- 12:47最新烧录：完整230136字节Flash校验通过（包括.data的42528字节），.text matched，GDB退出0，reset run完成。日志在会话tmp `highrate-flash-20261113-124711-{openocd,gdb}.log`。12:40那次也校验通过，但随后链路不可用；用户重新确认后按授权再烧一次，12:47通信恢复。不将链路超时归因成已确认硬件故障。

## 最新实板状态（未启动Sweep）

- 串口固定 by-id `usb-Horco_Horco_CMSIS-DAP_332107148384-if02`，921600，本次实际节点ttyACM2。SWD固定序列号332107148313。不得交换两探针用途。
- HELLO identity=infantry_app；source=none，authority=false，**manual_stop=true**，所有execute=false，航向valid、约-1.54946rad（仅该次快照，不复用为后续目标）。`.tmp/m2-trial/postflash-1247.json`。
- 停止态0x49 CANCEL返回NOT_PERMITTED，证明新消息已路由且不会绕过持权。
- 停止态schema2采集73条、0.288068s、249.941Hz，覆盖0；初始化阶段frame_rejections=1，保留原摘要，不报告CRC零错误。数据 `.tmp/m2-trial/stopped-smoke-1247.csv` 与 `.csv.summary.json`。该值属于旧CAL文本过渡时的解码统计，尚未独立归因；不据此判定电机异常。

## 下一步与授权边界

用户已授权烧录并看Sweep效果，按要求确认维护组合后，**首轮实车单频试验已经完成并自动撤输出**，未改PID。该授权不自动扩展到增大幅值或多频试验。

## 首轮实车结果（2026-11-13 13:07）

- 试验：0.5Hz、0.01Nm、3周期，计划6s；即时读取的保持航向-1.145240306854248rad，不使用烧录前数值。
- CSV `.tmp/m2-trial/first-20261113-130744.csv`（1500条活性+1条终态），报告同名 `.csv.report.json`；独立质量复核 `.tmp/m2-trial/first-20261113-130744-quality.json`。
- run_id=1，complete=true；终态相对首个活性记录MCU跨度5.999906s，序号无缺口；活性记录实际249.9957Hz，周期中位4001us，3723–4244us。
- 注入峰值0.00999999Nm，requested与applied最大差9.31e-10Nm（只表示队列接纳的命令分量，不代表真实轴力矩）。总力矩命令峰值0.0214831Nm。
- 航向峰峰0.14139°；中心化RMS0.000768695rad（约0.044°）。幅值很小，本轮是激励/采集/有限时长撤输出链路验收，不是PID或摩擦根因结论，也不是已完成频响辨识。
- 客户端已在finally取消、清全部目标、撤维护；新鲜DM反馈确认dm_enabled=false且authority=false。随后独立HELLO/status再次读到source=none、authority=false、所有execute=false。
- 用户尚未回复已将右拨杆回中：最后manual_stop=false。已明确提示右拨杆回中，不以无执行冒称人工停止挡已确认。
- 试验前修正主机dm_healthy：不用紧接status(1)的diagnostics(1)（两者共用50ms慢页节流），改取快速status(0)中的同一DM机械反馈有效性；10项客户端试验测试通过。实际START仍由MCU再次检查DM在线/使能/目标/维护资格。

下一步由用户另行决定，不自动继续多频、升幅或调PID。不自动提交/推送。
