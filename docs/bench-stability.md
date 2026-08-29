# Bench 稳定性与运行规范

本文只记录当前固定下来的做法和验证结果，不记录历史过程。

## 结论

- 12 个标准组合（6 个有向对 × bare/mpsl）都必须产出 central + peripheral
  两边的 `BENCH` 数据；`bench_parse.py` 只汇总这 12 个标准组合。
- bench 循环按 TX 窗口剩余容量投放数据（不灌满窗口），重传因此有真实空位：
  `wf=0`，fwd 投递丢失降到 0~18%。
- `rev_raw%` 是 central RX slot 的原始空包率；`rev_arq%` 是经过 ARQ
  重传/重排后的反向投递丢失。两者必须分开看，原始空包率不代表重传失效。
- 跑批遇到“两边都 BENCH READY 但整个窗口 rx=0”的采集竞态时，
  `run_pair` 自动重试，最多 3 次。

## 当前做法

### 1. MPSL 每个操作锁定到一个具体 slot

`transmit` / `receive` 先发布 `op_kind` 和数据缓冲区，然后：

1. 等待下一次 timeslot START（`start_signal`）；
2. 读取该 START 时的 `slot_start_done`；
3. 等待 `done_count` 越过 `slot_start_done`，即正好完成这一个 slot。

回调在 START 时递增 `slot_count`、记录 `slot_start_done`、触发
`start_signal`，执行完后再递增 `done_count` 并触发 `done_signal`。

这样应用层算出的 slot phase 和无线电实际执行的 slot 不会悄悄错开。

代码位置：

- `thunders-phy-nrf/src/mpsl/callback.rs` — START 边界发布
- `thunders-phy-nrf/src/mpsl/mod.rs` — `transmit` / `receive` / `wait_slot`
  的等待流程
- `thunders-phy-nrf/src/mpsl/state.rs` — `slot_count` / `slot_start_done` /
  `start_signal` / `done_count` / `done_signal`

### 2. 采集阶段只让 peripheral 扫

- central 是稳定时间基准，永远保持 `slot_nominal`，不扫 RX。
- peripheral 连续 miss 8 次后链 `nominal + 2 us` 扫自己的 RX 网格；
  同时 SlotRequest 的 TX 延迟按 0..210 us 轮转。
- 两侧都扫相同周期会把相对相位冻住，因此 central 扫被禁止。

代码位置：`thunders-phy-nrf/src/mpsl/callback.rs`。

### 3. peripheral 采集占空比

在 peripheral 收到第一个 Data 之前，不按镜像比例决定 slot：

- 偶数 phase：RX，听 beacon/Data；
- 奇数 phase：TX，发 `SlotRequest`。

central 的 RX 窗口是连续两个 phase，因此每个周期至少有一个 SlotRequest
落进去，不依赖初始 phase 猜测；同时仍有半数 central TX 被监听。收到
Data 后立即切回严格镜像比例。

代码位置：`thunders/src/link.rs` 的 `Peripheral::frame`。

### 4. bench 按窗口容量投放，重传才有空位

`Central` / `Peripheral` 增加 `tx_window_full()`。所有 bench 循环在
`tx_window_full()` 为 true 时不再投新 PING/echo；窗口腾出空间后才投
下一条。

- `wf` 计数保持 0：没有“投了但窗口满被拒”的伪丢包。
- 发送窗口不会长期满载，NACK 触发的重传有真实空闲 slot 可用。
- 有效吞吐会自适应到链路能确认的速率；满负载高吞吐和低投递丢失不可
  兼得。

代码位置：`thunders/src/link.rs` 的 `tx_window_full()`，以及各 example
`src/main.rs` 的 bench 投放条件。

### 4b. central 严格 ping-pong 投放（ow 归零）

8:2 比例下前向每周期 8 个 TX slot，而 peripheral 的 echo 缓冲只有 1
个、每周期只被消费 1 次。central 按前向速率投 PING 时，待发 echo 必
被新 PING 覆盖（`ow`），`rev_loss` 量的是 8:2 容量错配而不是无线电。
现在 central 严格 ping-pong：上一 PING 收到 echo 或被丢弃（df）之前
不投新 PING（`echo_pending`）。代价：PING 流吞吐降到约 1/周期（反向
带宽仍由 filler 测量）。效果：`ow ≈ 0`，`rev_arq` 落到 0~2%，与 fwd
同量级——这才是 ARQ 层的真实投递率。`ow` 保留为回归检查。

### 4c. MPSL op 流水线（发布期限从 ~200 µs 放宽到 ~2.5 slot）

旧协议在「上一 op 完成 → 下一 slot START」之间发布下一 op：op 完成
在 slot 的 ~250-310 µs 处，预算只有 ~200 µs，而 frame() 的处理
（解密/NACK/deliver/应用工作）实测 ~100-250 µs——52840 勉强不晚，
5340/LM20 有 8-30% 的 op 晚发布（`late=`），集中在 RX→TX 边界，一
次晚发布损失两个 op。

现在 MPSL phy 用**深度 2 的奇偶 op 环**（entry = target % 2），link
的 `frame_pipelined` 每个 frame 先发布 `hw_slot + 2` 的 op（期限是该
slot 的 START，预算 ~2.5 slot），再 collect + 处理 `hw_slot + 1` 的
结果（collect 即节奏）。TX 内容（echo 载荷、NACK 位图）提前一个
slot 构建：NACK 比旧路径旧一个 slot（有界浪费：多一次重复重传），
echo 空口延迟 +1 slot。grace 机制保留（app 整 slot 卡死时兜底）。
bare 后端走原同步路径（`Phy::op_pipelined()` 默认 false）。

附带修复：`nack_for_peer` 的 run-end 收尾条件原来是全局 `phase ==
rx_run_end`（局部索引）——peripheral 的 RX 分支里全局 phase 最多到
c_tx-1，**收尾永远不会触发**，peripheral 从不发送真实 NACK（前向重
传只能靠 tick 超时）。改为 `local_phase == rx_run_end`（central 本
来就两者相等，行为不变）。

代码位置：`thunders-phy-nrf/src/mpsl/state.rs` 的 `OpEntry` 环、
`callback.rs` 的消费逻辑、`mod.rs` 的 `publish_rx/publish_tx/collect`、
`thunders/src/link.rs` 的 `frame_pipelined` / `handle_rx_packet` /
`handle_rx_miss`。

#### 4c-2. 镜像锚定的精确化（beacon catch_slot + 投票）

镜像偏移 `slot_offset` 的 re-anchor 原来用**处理时刻**的
`phy.slot_count()`：应用处理可能比捕获晚整个 slot（5 s 的 defmt 报
告卡 ~1 ms），晚处理会被当成假的偏移位移。现在用**捕获 slot**（该
op 的目标 slot，与处理延迟无关），且只有**连续两个 beacon 算出同一
候选值**才采纳（投票）——单个被晚处理的 beacon 无法冻结错误偏移。
实测曾抓到 `txph=[1,2]+[8,9]` 的混合：同一个 run 里偏移中途跳变并
冻结，整对变哑。差分锚定（按两次 beacon 间的计数滞后增量修正）在
实测中更差（7/8 失败 vs 4/8），已回退。

#### 4c-3. echo 放置的 peer-window 钳位

反向死亡的典型现场：central 只听到 len=3 的 SlotRequest，**19 字节
的 Data echo 全部死在窗口尾部裁剪**（帧尾超出对端监听窗，被 MPSL
硬边切断 CRC）。echo delay 公式测量值偏斜时会把 TX 推到窗口边缘；
现在给 delay 加**对端窗口尾部钳位**（帧尾必须落在对端窗口内，宁可
偏中心也要进窗），与 slot 内预算钳位取 min。

#### 4c-4. 中央 drop 死锁（SlotRequest 携带 ACK + 存活证明）

采集期的 peripheral 只回 SlotRequest（原来不带 ACK）：central 丢包后
的 `pending_drop` 永远等不到覆盖它的 ACK，只能靠强制清除。现在
SlotRequest 携带 peripheral 的累计 ACK（`rx.ack()`），central 在
SlotRequest 分支走正常的 `apply_ack_nack` + `clear_pending_drop`——
窗口从存活流量本身就能推进，强制清除作为兜底保留。实测好 run 的
PING 回显可达 tx=228/rx=222。

#### 4c-5. MPSL cadence floor：500 → 600 µs

最后的 run-level 硬币翻转并不是随机 RF：500 µs cadence 扣除 MPSL
要求的 150 µs inter-slot gap 后，grant 只有 350 µs。19 字节 Data echo
扣除 setup、TX ramp、airtime 和 40 µs tail，只剩约 **129 µs** 合法
delay；死态实测的 peer window 需要约 **150–180 µs**。短 SlotRequest
仍可放入（所以看起来采集活着），长 echo 却在该 slot 内物理上无解，
形成“SR 能到、Data echo 永远不到”的稳定死态。

MPSL cadence floor 改为 **600 µs**：grant 变 450 µs，长 echo 的合法
delay 扩至约 **229 µs**，完整覆盖 0–210 µs acquisition sweep。协商
也不得再把 cadence 缩回板卡的 500 µs 物理下限。代价是 slot rate 从
2000/s 降到约 1666/s（17%），换来连接确定性：

- 52840→5340 连跑 8 次：**8/8 成功**，每次都有双向数据，最好
  tx=636/rx=635；此前同样批次通常只有 1–3 次真成功；
- 完整 6 个 MPSL 方向：**6/6 成功**，包含原已知死点 LM20↔5340；
- 最弱的 5340↔LM20 再各跑 4 次：**8/8 成功**；
- 合计本轮验证 **22/22 MPSL run 成功**，forward loss ≤0.15%，
  rev_arq 0.04–0.75%。

曾尝试 ack-stall 触发的 echo-delay sweep；500 µs 下长 echo 的 slot
预算会把 sweep 钳到 ≤129 µs，因此无法触及需要的窗口，且实测更差，
已回退。这个失败反过来验证了 grant 长度才是最终物理约束。

#### 4c-6. 协商式短/长 phase cadence（500/600 µs）

全600稳定但让不需要 follower delay 的 central-TX 相位也付出17%成本。
现在两端先以全600采集，SlotRequest 上报 short/long capability；central
收到**一颗**SR后确定profile并选择未来16个superframe后的phase-0绝对
硬件slot作为生效点。生效前每个central TX slot都发commit beacon（8:2
下约128次接收机会）。peripheral只有在两-beacon投票得到精确
`slot_offset`后，才把central epoch翻译成本地apply slot并arm profile。

callback按逻辑phase分别计算当前距离和下一grant：

```text
current = cadence(slot)
next    = cadence(slot + 1)
request.distance_us = current + one-shot PLL correction
request.length_us   = next - 150
```

默认8:2 profile为：

- central TX phases 0..7：short **500 µs**；
- peripheral TX / reverse phases 8..9：long **600 µs**；
- superframe：`8×500 + 2×600 = 5200 µs`，实测 **1922–1923 slots/s**，
  比全600的1666/s提升 **15.4%**；
- 长echo仍保留450 µs grant和约229 µs合法delay。

曾A/B short=450：central达到2082/s，但5340 follower只能维持
1820–1925/s，出现2/8 profile失步。这组结果现在只作为**已知安全anchor与失败
样本**：500/600不是所有短包的硬下限，更短值必须在每次API协商期间按本次
payload合同实际尝试后才能提交。

旧版固定500/600实现曾验证52840→5340连续 **8/8**，完整六个MPSL方向
**6/6**；这些结果不能替代新版在线搜索的最终确认。bare继续走原uniform
cadence，不参与profile。

#### 4c-7. API触发的包长合同协商与有界稳定性Probe

应用不会因某一颗包变长而自动修改slot。central或peripheral显式调用
`negotiate_cadence(TrafficContract, CadenceProbePolicy)`后，正常`frame()`循环
驱动以下状态机：

```text
Request（peripheral API时）→ Offer → Accept
→ Probe(start,end) → Armed → Sample → Report
→ Commit(apply_epoch) → Applied → 双方同epoch生效 → Data确认 → Stable
```

central仍是唯一的candidate和绝对epoch决策者。合同中的forward/reverse payload是
**精确长度**而非上限；提交后Data采用固定wire格式
`marker(1)+seq(2)+ack(2)+NACK bitmap+payload`。NACK字节数由已知slot比例推导为
`ceil(peer_tx_slots/8)`，payload和NACK都不再逐包携带Vec长度。secure模式的4B/16B
认证tag也直接计入payload wire长度，因此PHY feasibility floor使用真实方向长度
`5+nack_bytes+encrypted_payload_len`，不再固定估算`+12B`。fixed-wire能力通过
Offer/Accept flag回显；旧peer不回显时双方继续使用postcard数据面及其精确最坏长度，
不会在同一generation误切codec。peripheral把本地forward/reverse floor复用Accept的
两个epoch字段返回，central取两端最大值。
**该公式不是最终slot选择**，500µs也不再是Probe硬下限。

搜索分两轴进行：先固定已知稳定long，下降forward/short；确定通过值后再固定它，
下降reverse/long，并始终保持`short <= long`。每个候选累计32次真实
`CadenceSample`试发，wire长度覆盖对应方向最坏Data。为避免失败候选连续运行一整个
superframe后永久拉开两颗MPSL硬件计数，每次试发只overlay一个目标phase，前后都恢复
active profile；wall time由MPSL callback边界精确计量。至少7/8试发必须同时满足
slot timing、TX完成、CRC正确接收、`op_late`和ARQ delivery条件。

两轴搜索得到带`safety_steps`余量的pair后，还要把**完整最终profile**连续运行最多
32个superframe。隔离候选已经要求对应方向真实wire包CRC正确；最终连续窗口进一步要求
两端TX按计划完成、两个方向都至少出现一次ADDRESS重叠、无`op_late`且slot数完整。
follower若恰好错过callback起始时间戳会得到`clock_us=0`，此时使用独立slot/TX/RX
计数判定而不误杀。最终确认前保留64个稳定superframe的Probe/Armed lead，使PLL从边缘
候选恢复。确认失败或协议异常走同步Release，最迟在deadline后统一回到600µs acquisition。

8B合同、500/600已知安全anchor的52840→5340硬件run在第3次启动进入
`CADENCE STABLE short=500 long=600`，随后fixed Data/Ack/Drop连续运行约80秒：slot rate
保持1922/s，多数5秒窗ping往返丢失7–14%，累计达到`rxd=27363, txd=3308`，稳定窗内
`delivery_failures=0`。这证明codec确实越过apply epoch并承载真实双向数据，而不只是
host端字节测试；实验台仍存在协商前空链路及弱向启动波动。

同一52840→5340组合随后用默认`min=300, step=25, probe=32`分别协商五种精确payload：

| payload | fixed Data wire（8:2） | 2M feasibility floor | 实测Stable |
|---:|---:|---:|---:|
| 1B | 7B | 333µs | 500/600µs |
| 4B | 10B | 345µs | 500/600µs |
| 8B | 14B | 361µs | 500/600µs |
| 16B | 22B | 393µs | 500/600µs |
| 32B | 38B | 457µs | 500/600µs |

这里使用当前2M实现的字面公式：`wire=payload+6`、
`airtime=28+4*(wire+3)`、`floor=airtime+265`。双方运行时实际值应以
`CADENCE BOUNDS`日志为准。这同时证明两件不同的事：软件的候选下界确实随包长变化
（纯planner测试在全部候选通过时分别得到333/333和457/457），但**最终稳定值不保证随包长不同**。本板对上475/575
以下候选失败的主导因素是MPSL grant/PLL/phase稳定性，而不是airtime；长度只改变可尝试
范围，实测稳定性仍可被同一个长度无关的硬件瓶颈钳在500/600。把公式floor直接宣称为
稳定slot反而违反“协商期间实际尝试后决定”的要求。

为避免只报告幸存的Stable run，`scripts/bench.sh`支持`BENCH_ALL_ATTEMPTS=1`并输出
`CADENCE ATTEMPT/YIELD`；`scripts/bench_cadence_grid.sh C P PAYLOAD SECS ATTEMPTS`会对
25/10/5µs step分别执行全部冷启动，日志按`-sSTEP`隔离。bench固件同时记录双方合并后的
`CADENCE BOUNDS`以及每个不同的`CADENCE CANDIDATE`，可直接验证475/600、500/575、
490/600、500/590等边界，而不是从最终profile反推。10µs-step smoke test在
LM20→52840、8B合同的两批各3次冷启动合计得到`successes=3/6`；成功run均记录
`local 361/361, peer 361/361, effective 361/361 → 490/600 → 500/590 → 500/600 Stable`，
证明两端实际Accept floor与当前公式一致，也把short-chain和reverse-echo两个失败边界
分开。测试时5340调试器临时
离线，因此完整20次三板矩阵仍留给后续正式实验。

Probe统计现改为MPSL callback在绝对`[start_slot,end_slot)`边界锁存slot、executed TX、
ADDRESS/CRC、`op_late`和DWT时间；累计值的每个字段均为atomic，并以odd/even generation
保证一致快照，避免IRQ与任务并发读写普通多字段struct。Link在安排Probe时提前保存累计
baseline，不再依赖app线程恰好在单slot窗口内醒来。错过精确START的窗口由callback显式
累计为abort、通过compact Report同步给central，最多重试3次；相同Armed descriptor只重复
应答而不重写可能已启动的overlay。step=10硬件A/B中，central
的490/600共32次全部精确为`slots=1, tx=1/1, clock=489–490/490`，原来的central零slot/
错operation消失。follower仍有7/32次零slot、4/32次错operation；500/590的central则
32/32次slot/time正确但CRC catch为0。由此可区分：central侧app采样竞态已消除，剩余问题
是follower晚arm/phase边界以及reverse echo，而不是payload floor或静态phase公式。

随后对这两个阻塞做了保守修复：follower只在至少提前2 slot收到Probe时才arm；更晚的
描述符立即以ABORT Report请求同candidate新epoch，最多3次，完全相同的Armed描述符不再
重写overlay。echo delay现在读取pending TX entry的绝对target，用该slot的candidate
`nominal-150`作为TX grant；peer窗口按RXEN-origin语义先裁到`min(W,G-100,G-40-O)`，
再减peer ramp用于READY后居中。可用的52840/LM20板对以step10和500/600 anchor双向各
6次冷启动仍为0次Stable，500/590也未继续下降，说明修复消除了错误边界处理但尚不足以
突破实际phase/echo cliff；PLL及窗口基线仍保持不变，没有通过放宽标准制造假阳性。

同轮审查修复了一个会污染吞吐结果的独立问题：depth-two pipeline在发布下一轮phase 0
时保留上一轮slot→seq map：第一颗反馈在peer完成本轮NACK前已发布，仍映射R-1；最后
反馈在本端发布下一轮phase 0并轮换map时映射刚完成的R。另确认当前
`HOP_MISS_THRESHOLD == LINK_LOSS_THRESHOLD == 16`且loss分支在前，adaptive-hop不可达；
不能只降低阈值，因为central单边换台后peripheral无法先验知道新频道。该项保持禁用，
等待“旧频道Beacon公告future hop epoch”协议后再单独修复。

#### 4c-7a. 52840时钟与PLL来源隔离

曾用52840 RC/LFXO以及fixed/probe-freeze PLL实验隔离来源。实验结束后仅保留有诊断价值的
`THUNDERS_52840_LFCLK=rc|xtal`；无效PLL feature及脚本入口已删除。8B、step10结果如下：

| 方向/配置 | 尝试 | Data链路 | Request | Stable |
|---|---:|---:|---:|---:|
| LM20→52840，RC+phase | 20 | 10 | 10 | 0 |
| LM20→52840，LFXO+phase | 8 | 3 | 3 | 0 |
| LM20→52840，LFXO+fixed PLL | 8 | 0 | 0 | 0 |
| LM20→52840，RC+probe-freeze | 8 | 4 | 4 | 0 |
| 52840→LM20，RC+phase | 6 | 1 | 1 | 0 |
| 52840→LM20，LFXO+phase | 8 | 0 | 0 | 0 |
| 52840→LM20，RC+probe-freeze | 8 | 2 | 2 | 0 |

LFXO没有改善candidate或Stable yield，因此不能把失败归因于52840 RC频漂。完全关闭phase
correction反而使follower无法完成acquisition，证明PLL修正是必要的；只在Probe overlay内
冻结修正仍为0 Stable，说明candidate期间的`err/4`修正也不是主因。52840作为LFXO central
时曾在启动早期累计1078个CRC catch，随后长期无新catch而slot rate仍为1922/s，更符合
两条独立chain的phase状态/重锚问题，而非CPU性能或长期LFCLK漂移。剩余优先排查项是
Probe epoch→local slot映射、overlay首slot实际op/phase，以及reverse on-air anchor。

#### 4c-7b. Probe arm lead与首slot callback trace

曾比较Probe arm lead 2/3/4；lead提升无效，实验feature及脚本入口现已删除并固定为2。8B、
step10结果：

| 方向 | lead | 尝试 | Data链路 | Request | Stable |
|---|---:|---:|---:|---:|---:|
| LM20→52840 | 2 | 20 | 10 | 10 | 0 |
| LM20→52840 | 3 | 8 | 3 | 3 | 0 |
| LM20→52840 | 4 | 8 | 5 | 5 | 0 |
| 52840→LM20 | 2 | 6 | 1 | 1 | 0 |
| 52840→LM20 | 3 | 8 | 3 | 3 | 0 |
| 52840→LM20 | 4 | 8 | 2 | 2 | 0 |

增加的callback trace锁存`start-1/start/start+1`的slot、profile phase、nominal、实际exec
kind/target及TXEN/RXEN/ADDRESS offset。一个LM20 central→52840 follower的lead=4 run给出：

```text
central:   phase=[9,0,1] nominal=[600,490,500]
           kind=[RX,TX,TX] target=[S-1,S,S+1]
follower:  phase=[9,0,1] nominal=[600,600,600]
           kind=[TX,IDLE,RX] target=[L-1,0,L+1]
```

因此静态epoch差值和phase编号一致，但central进入490µs Probe时，follower在对应phase-0既未
启用candidate nominal，也没有发布RX op。根因最终定位为：API协商期间仍收到的周期Beacon
满足旧`cadence_ack==0`条件，调用`align_slot_period(600)`并在descriptor与start之间撤销
follower的probe overlay；同时central过去会在未收到Armed时按本地`start-3`单边开始。

修复后central不预先arm，只有收到匹配且lead足够的Armed才schedule本地overlay；未Armed窗口
结束后同candidate换新epoch。Beacon在任何API cadence transaction中都不得调用uniform
align。Probe phase不再依赖可能陈旧的`slot_offset`，而以完整
`central_start + (local_slot-local_start)`计算（包括两端独立u32 wrap）；`probe_armed`改为
Release/Acquire atomic发布。修复后的trace中两端phase、candidate nominal及TX/RX op一致。

硬件结果随即从全部0 Stable变为：lead=4的8次冷启动有1次选择并提交`440/440µs`；生产默认
lead=2的8次有1次提交`500/500µs`。500/500在约20秒内维持1985–1990 slots/s、双向Data持续、
`delivery_failures=0`，440/440也短暂达到约2101 slots/s。但两者随后都触发安全fallback到
600µs，说明单边Probe根因已修复并真正解锁sub-500搜索，下一瓶颈已转为提交后的长期
phase稳定性；不能把140ms最终确认当作生产稳定证明。

Commit阶段也改用`apply_epoch-local_apply`得到权威pending slot offset；Applying窗口的Link op
使用descriptor phase，完成apply后才发布全局offset，避免Probe正确而生产profile又退回陈旧
`slot_offset`。新增2048-superframe（约8–10秒）强制Probation：期间恢复正常ARQ tick，任何
新delivery failure或16次连续miss都会由central同步Release，只有deadline通过才对API公开
Stable。此前会短暂报告440/440、500/500的同类8次A/B在Probation版本中为0 Stable，证明
不可持续profile不再被误报，同时仍保持600µs安全回退。

#### 4c-7c. 专用phase-0同步slot

协商能力flag新增`sync_slot`并由Offer/Accept回显；旧peer不回显时保留旧布局。新布局把phase0
固定为long Beacon/resync，只有phase1..7使用payload-aware short，phase8..9 reverse继续long。
`CadenceProfile.sync_slot`使API的superframe时长/容量自描述；MPSL Probe和active profile都
携带完整central/local apply epoch，避免独立u32 wrap。long feasibility floor同时覆盖36B
Beacon序列化上界。正常数据面每个superframe强制发送phase-0 Beacon。

LM20→52840、8B、step10硬件结果：lead4为6/8通过Probation并公开Stable，profile分别为
`410/433, 420/433, 390/433(2次), 361/433, 420/433`；其中361/433达到约2607–2609 slots/s，
连续约50秒双向Data、`delivery_failures=0`后才安全fallback。生产默认lead2为2/8 Stable，
profile为420/433和400/433，峰值约2353和2431 slots/s。相比旧500/600约1922/s，专用同步
slot已把可用运行区间提高约22–36%，并显著延长sub-500持续时间。随后把Beacon golden最大
33B上取整为36B floor、并将pending/active profile publication改为AtomicBool后，最终lead4
复测为2/8 Stable：420/449约2326 slots/s、410/449约2356 slots/s；两者均通过双向Data
Probation，但120秒内仍会安全fallback。下一步应让Beacon的绝对epoch参与持续
future-boundary重锚，而非只提供长RX窗口。

#### 4c-7d. 120秒生产稳定guardrail

继续A/B发现旧日志末尾统一回600的一部分是bench伪影：peripheral probe进程仅保留`SECS+20`，
会在central完整measurement window结束前退出。改为`SECS+120`后再评估。同时，sync profile
生产floor收紧为450/600µs，保留600µs phase-0/reverse恢复余量；active sync链路的连续miss
回退阈值从16提高到64（约30ms），该模式禁止旧的central单边adaptive hop，仍会在持续失联时
同步安全回退。

最终LM20→52840、8B、step10、生产默认lead2冷启动4次中2次完成协商；两次均选择
`450/600µs`，在完整120秒窗口末尾仍维持约2018–2019 slots/s、双向Data、
`delivery_failures=0`，没有回600。相对500/600基线约1922/s提升约5%，应用吞吐窗口约
3.6–4.8KB/s。lead4复测也得到完整120秒不回退的450/600 run。

phase-0慢速频率积分、fixed PLL和Probe freeze实验均未改善yield，相关feature、状态和遥测已
删除；生产只保留经硬件验证的一次性phase correction。

#### 4c-7e. Beacon与8:2 Data分配彻底分离

最终布局不再把Beacon算入8个forward slot：物理superframe是`1 sync + 8 forward Data +
2 reverse Data`共11 slots。`CadenceProfile.forward_slots/reverse_slots`只表示应用Data容量，
`sync_slot`独立增加period和long时长；phase0在Link/ARQ中没有local Data phase，phase1..8映射
forward 0..7，phase9..10映射reverse 0..1。Beacon不会写RX run mask、轮换TX map或重置
Stable期Data miss streak。MPSL初始profile、Probe、Commit和active profile统一使用11-slot布局。

该布局是MPSL wire-layout版本变更：sync capability未回显时明确PeerRejected，不能在建立连接前
兼容旧10-slot固件。Config新增`physical_period_slots(sync_slots)`，bench pacing和phase telemetry
使用11-slot物理grid。同步Release还保留旧fixed contract解码2个depth-two残留包，避免codec
retirement边界把`0xF1` fixed ACK误判为InvalidPacket。

LM20→52840、8B、默认lead2、hold模式4次冷启动有1次完成协商；该run的500/600 profile在完整
120秒窗口末尾仍维持约1895 slots/s、双向Data、`delivery_failures=0`。450/600在4次样本中
没有通过完整Probation，因此独立sync布局的生产floor暂保持500/600。虽然slot rate低于旧的
10-slot 450/600，但8:2现在是完整8个forward Data slots，Beacon不再偷占Data容量。

#### 4c-7f. Fast模式与有界重试

新增默认关闭的`cadence-fast`能力和bench入口`THUNDERS_CADENCE_MODE=fast`；它只把独立布局的
forward Data floor从500降到450µs，sync/reverse仍为600µs，并保留Probe、双向Data
Probation和64-miss fallback。450/600在90秒8次冷启动中3次Stable、120秒4次中1次Stable；
成功run均保持到窗口末尾、约2035–2036 slots/s、`delivery_failures=0`。新fast入口复测4次有
2次Stable。

失败分类证明fast timing不是主因：失败run主要在建立初始Data前（无Request），少数停在
Request后无peer bounds；没有run进入450 candidate后被Probe/Probation拒绝。为后者增加
Request/Offer/Accept 2048-superframe控制deadline，示例在`ControlTimeout/Failed`后等待2秒、
排空TX并用新generation重试，最多4次。无Data的冷启动仍需bench/设备级重启，不能靠cadence
generation重试修复。

#### 4c-7g. 显式initial SyncReady/SyncArmed

初始mixed profile不再由central收到一颗SlotRequest就单边schedule。新握手为：central Beacon广播
proposal epoch；peripheral在两Beacon anchor一致后只回`SyncReady{generation,
proposal_epoch}`；central收到精确Ready后选择新的future commit epoch并以Beacon bit6广播；
peripheral仅对该commit schedule成功后回`SyncArmed{generation, apply_epoch}`；central收到精确
Armed且lead足够才schedule。`schedule_slot_profile`改为返回bool，拒绝pending replacement时不会
伪报Armed；fallback会清空双方initial state，已协商peer发送zero-ack SlotRequest时central也
重新加入fallback。Sync控制包本身不形成Data Connected。

该协议消除了initial profile单边apply的安全漏洞，但未提高本对硬件的冷启动yield：最终90秒
8次样本为1/8 Stable，只有3/8建立普通Data；其余失败仍发生在首个Data之前。说明剩余根因在
初始RADIO/PLL acquisition，而不是initial profile epoch确认。该安全握手保留，yield优化需单独
重置/随机化PHY acquisition，不能继续靠cadence generation重试。

#### 4c-7h. 长时间双向soak

整理后的`34859a2` safe hold固件做了长窗口验证。bench新增
`BENCH_STABLE_DEADLINE_SECS`：长soak若在deadline内未进入Stable会提前结束该冷启动，避免把
整段测量时间浪费在已知的acquisition失败；一旦Stable仍运行完整`SECS`。

| 方向 | 窗口 | Profile | 最终rate | 最终Data RX/TX | df | retx |
|---|---:|---:|---:|---:|---:|---:|
| LM20→52840 | 15分钟 | 500/600 | 1895/s | 305582 / 35737 | 0 | 14 |
| 52840→LM20 | 10分钟 | 500/600 | 1895/s | 199357 / 6326 | 0 | 85 |

两次均无Exit/Release/fallback，所有5秒窗口保持约1886–1895 slots/s，双向Data持续增长。
反向累计重传较多但10分钟内仅85次、`delivery_failures=0`，不构成稳定性回归。没有观察到可供
进一步修复的运行期失稳，因此本轮不调整PLL、miss阈值或生产slot floor；剩余问题仍是冷启动
acquisition yield，而非Stable后的持续性。

最终profile与固定wire codec在同一个apply epoch生效；`frame()`长度只要不等于
对应方向合同的精确payload长度就返回`PayloadExceedsCadenceProfile`，不会填充、截断、
自动扩slot或偷偷重协商。协商开始前双方TX窗口必须排空，进入控制状态后也不再接收
新的应用offer；apply边界最多容忍two-slot pipeline已发布的2颗旧postcard数据包，随后
fixed-active节点严格拒绝postcard Data/Ack/Drop，但始终接受postcard控制。任一端进入
600µs emergency fallback时，central以`apply_epoch=0`的权威Beacon通知peripheral一起
清除fixed codec，避免普通包仍能收到时形成单向codec分裂。协商期间ARQ retry age被冻结，
避免Probe占用slot导致应用数据超时；forward完整描述携带ACK/NACK，
脆弱reverse方向改用短`CadenceAck`承载Accept/Armed/Report/Applied。

最终切换使用两阶段arm：central持续发送未来Commit但不先切换；peripheral arm后
持续发送精确`{generation, apply_epoch}` Applied；central收到后才arm。Applied
若迟到越过epoch，central滚动到新的未来phase-0 epoch重新Commit。central生效并
恢复正常Data/Ack后，peripheral才停止Applied并进入Stable。因此丢失整个Commit
窗口只会延迟协商，不会形成单边profile。

PHY保存`active + pending + probe overlay`三个profile层次；合法候选只在预先协商
的`[probe_start, probe_end)`生效，结束后由MPSL callback自动恢复active profile。
profile字段用release/acquire fence发布给MPSL IRQ。

#### 4c-8. 退出短包合同

central和peripheral都提供`exit_cadence()`。peripheral先发Release请求，central以
采集阶段保存的`cadence_safe_profile`作为唯一权威目标，双方执行
`Release → Accept → Commit → Applied`，到同一phase-0 epoch后才解除payload合同。
在退出协商、失败或重试期间，原合同仍由独立的`cadence_active_contract`约束，不能
借状态切换发送超长包。`cadence_status()`在流程中返回`Releasing`，完成后返回
`Idle`；重复退出是幂等的并返回generation 0。

`set_cadence_exit_policy(Some(CadenceExitPolicy::new(delivery_failures,
consecutive_misses)))`可启用安全自动退出。任一非零阈值独立生效：从合同Stable时
的基线开始累计retry耗尽的delivery failure，或统计连续无peer包slot。阈值只触发
同一个Release协议，**包长永远不是自动退出/切换条件**；0表示禁用对应条件，`None`
禁用全部自动退出。两端另有跨跳频累计的16-slot失联安全线，用户配置的大于16的
`consecutive_misses`会收敛为16；更长期、允许中间偶尔成功收包的劣化应使用
`delivery_failures`阈值。

Release/Commit/Applied都使用不超过16B的紧凑控制，保证退出消息能装进当前短slot。
退出完成generation及apply epoch继续由周期Beacon广告，peripheral即使丢失第一颗
post-apply Data/Ack也能最终确认并解除合同，不会在idle或reverse-first业务中永久
停留于Applied。若整个握手单向失联或超过256 superframe，节点撤销profile overlay、
回到统一600µs acquisition并清除合同；peer收到`cadence_ack=0`的SlotRequest后也进入
相同fallback，再由原有Beacon/SlotRequest流程协商并提交采集阶段的安全profile。

可选bench feature `cadence-probe`可由`CADENCE_PROBE=1 scripts/bench.sh build`
启用。示例在Stable 3秒后调用`exit_cadence()`，依次记录`CADENCE EXIT`和
`CADENCE RELEASED`，同时验证退出后Data继续传输。最终六个MPSL方向
（52840/5340/LM20互为central/peripheral）都观察到完整
`STABLE → EXIT → RELEASED → post-release Data`；central均为 **1922 slots/s**，
peripheral为 **1912–1922 slots/s**。其中52840→LM20因历史性的全600 acquisition
波动在第二次启动通过，其余方向第一次通过；该启动波动发生于API调用之前。

### 4.2 fast cadence短包长度扫描

`scripts/bench_payload_sweep.sh 25`依次构建并测试1/4/8/16/32B应用payload，
覆盖三个芯片互为central/peripheral的六个MPSL方向。扫描设置`CADENCE_PROBE=1`
和`CADENCE_HOLD=1`：只有日志出现`CADENCE STABLE`才接纳该次运行，且整个25秒
窗口保持已协商profile，不在3秒后调用退出API。日志以`-p1`…`-p32`区分，
`scripts/bench_payload_parse.py bench/logs`生成汇总表。

吞吐是两个方向成功交付的应用payload字节总和；`raw loss`是reverse RX slot未收到
应用包的比例；`df`及`retx`是两端链路层累计delivery failure和重传计数之和。8B以上payload
内含32-bit应用sequence，因此可额外计算双向ARQ后应用丢包；1B/4B没有足够空间容纳
该sequence，表中相应两列显示`n/a`，不能误写为0%。所有长度都使用相同的饱和
PING/echo+FILL流量模型，因而吞吐可直接比较。

25秒实测聚合结果（每个长度六个方向，丢弃第一个5秒warmup窗口）：

| payload | central slot/s | peripheral slot/s | 平均吞吐 B/s | 范围 B/s | reverse空slot | RTT中位数 | df合计 | 重传合计 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1B | 1835 | 1823 | 273 | 98–440 | 36.8% | 17175µs | 0 | 56 |
| 4B | 1836 | 1826 | 1049 | 286–1755 | 40.3% | 12173µs | 0 | 46 |
| 8B | 1846 | 1833 | 2201 | 1353–3251 | 36.7% | 15340µs | 0 | 72 |
| 16B | 1833 | 1818 | 4373 | 1348–6868 | 37.6% | 13120µs | 0 | 56 |
| 32B | 1827 | 1814 | 5085 | 1689–13245 | 62.3% | 105441µs | 49 | 3186 |

结果表明已验证的500/600µs cadence floor决定slot rate，1–16B间包长几乎不改变
slot频率，吞吐基本随payload线性增加。**16B是本轮可靠性优先的最佳点**：相比8B
平均吞吐约翻倍，六方向df仍为0且双向总重传仅56。32B虽然平均吞吐继续增加，但跨芯片
方向出现明显退化：reverse空slot升至62.3%、RTT中位数约105ms、双向累计3186次重传且
49次delivery failure；因此不能把32B作为当前fast profile的稳定短包上限。逐方向原始
表可由上述parser从保留的`-p*`日志重新生成。

### 5. bench 计时从 central BENCH READY 开始

`scripts/bench.sh run-pair`：

1. 先启动 peripheral，等日志出现 `BENCH READY`（最多 60 s）；
2. 再启动 central，等日志出现 `BENCH READY`（最多 90 s）；
3. central READY 后才开始计 `SECS` 秒测量窗口；
4. 结束先发 INT，5 s 后未退出再发 KILL，清理不无限等待。

这样 LM20 约 14 s 的烧录时间不会吃掉测量窗口，短 `SECS` 也能拿到窗口。

代码位置：`scripts/bench.sh` 的 `run_pair`。

### 6. 解析只认标准 12 组合

`scripts/bench_parse.py` 只解析 `{52840,5340,lm20}-{52840,5340,lm20}-{bare,mpsl}`
且 central≠peripheral 的 12 个名字；实验日志（如 `*-sr-*`）保留在磁盘但
不进入汇总表。每个 log 多于 1 个窗口时才丢弃第一个 warmup 窗口。

代码位置：`scripts/bench_parse.py`。

## 提高原始成功率、减少重传触发

重传只能恢复残余丢包；真正降低重传次数要看 **PHY 层首次成功率**。当前已固定的
手段，按影响排序：

1. **发射功率打满**：LM20 和 52840 的 bare/MPSL 都使用 +8 dBm；5340 net
   core 受 RADIO 前端的 0 dBm 上限限制。代码在
   `thunders-phy-nrf/src/mpsl/radio.rs` 和 `src/radio_phy.rs`。
2. **可切换 1 Mbps 模式**：六个 example 都有 `radio-1m` feature，选择
   `RadioMode::Nrf1Mbit`；bare 同时把 slot 周期调到 600 us，MPSL 仍用
   500 us fallback。链路预算约提高 3 dB，但空中时间翻倍、吞吐下降。
   bench 用 `THUNDERS_RADIO_MODE=1m scripts/bench.sh build/run` 统一切换。
3. **slot 边界同步 + 采集占空比**：减少“包发在正确窗口之外”的失败。
4. **按容量投放**：避免窗口满导致的重传风暴；同时给 NACK 重传留空位。
5. **信道保持与抗干扰**：未建链时固定 `initial_channel`，建链后只在连续
   16 个 RX miss 时跳频。若环境有固定干扰，优先换干净的
   `DEFAULT_HOP_SEQUENCE` 信道集合。

进一步降低原始失败率的候选手段（尚未启用，需要单独验证）：

- **1 Mbps 全矩阵标定**：`radio-1m` 开关已经存在，但当前 12 组合基准表
  仍是 2 Mbps；1M 的 slot 预算、echo 公式和吞吐需要单独跑矩阵验证。
- **动态信道选择**：RSSI 采样已可读；按 RSSI 排序选择 25 个安静信道会
  显著降低同频干扰，代价是实现和两侧同步成本。
- **加宽 RX 窗口**：仅当日志显示 `addr` 少、`crcbad` 少（窗口擦边）时
  有效；当前高丢包组合多为 `crcbad`/`crcbadl` 高，说明是 SNR/CRC 问题，
  加窗口收益有限。
- **硬件/RF**：天线方向、距离、屏蔽、避免 USB 3.0/2.4 GHz 同频源。

判断方向看 MPSL `PLL` 行：

- `addr` 低、`crcbad` 低 → 包没进窗口，查 timing/PLL；
- `crcbad`/`crcbadl` 高 → 包到了但 CRC 不过，查 SNR/功率/干扰/速率；
- `txp`/`rxph` 相位散开 → 查 slot 边界同步和采集占空比。

注意 `crcbad` 的口径是"任何非完整合法帧的 RX slot"（含空听窗口），
只有 `crcbadl`（DMA 长度字节 ≥ 11 的地址匹配帧）才能证明真 CRC
失败；逐窗口差分分类用 `scripts/pll_diag.py`（empty/trunc/phantom/
corrupt 标签）。PLL 行的 `rsum`/`rcnt` 是 CRC 成功帧的累计 RSSI
和/计数（差分得窗口均值，dBm = −值），`rmax` 是自复位的最弱成功帧
——链路余量探针，替代"拉近板子"实验：2M 灵敏度约 −95 dBm，
`rmax` 对应的余量 ≈ 95 − rmax dB。

## 运行命令

```sh
scripts/bench.sh build      # 构建 12 个 ELF 到 bench/bin
scripts/bench.sh run 30     # 12 组合，每个测量 30 s
scripts/bench.sh run-pair 5340 lm20 mpsl 20  # 单个组合
python3 scripts/bench_parse.py bench/logs    # 汇总表
python3 scripts/bench_parse.py bench/logs --rssi

# 1 Mbit 模式（链路余量更高、吞吐更低）：build 与 run 必须同模式
THUNDERS_RADIO_MODE=1m scripts/bench.sh build
THUNDERS_RADIO_MODE=1m scripts/bench.sh run 30

# 非对称 idle ratio：THUNDERS_RATIO=844|622|422
THUNDERS_RATIO=844 scripts/bench.sh build
THUNDERS_RATIO=844 scripts/bench.sh run 20
```

运行前提：三个探针都可访问（`probe-rs list` 不显示 inaccessible）。
探针映射和芯片参数以 `scripts/bench.sh` 顶部的数组为准。

## 当前矩阵

以下为当前代码 + 当前固件跑出的标准 12 组合；parser 已丢弃 warmup 窗口。

```text
run                    wins   fwd% rev_raw% rev_arq%  rtt_avg    min    max    bw B/s  c-rate/s
-----------------------------------------------------------------------------------------------
52840-5340-bare           3  17.78    67.48    59.01    845.5    244   3753     11738      2502
52840-5340-mpsl           3   1.59    55.71    83.45   1983.4   1098   7659     13502      1999
52840-lm20-bare           3  19.06    91.72    85.34   1136.9    244  14526      8025      2502
52840-lm20-mpsl           3   1.59    69.72    77.36   1427.6    915   5004     10902      1999
5340-52840-bare           3   4.13     6.80    77.42   1002.6    518   3997     19710      2500
5340-52840-mpsl           3   1.65    14.33    78.00   1390.4   1129   6683     14941      1929
5340-lm20-bare            3   0.00   100.00     0.00      0.0      0      0         0      2500
5340-lm20-mpsl            3   1.70    75.11    97.77   1622.8   1098   5004     12251      1888
lm20-52840-bare           3  12.03    67.36    58.95    886.3    388   2154     11749      2500
lm20-52840-mpsl           3   1.63    70.83    80.16   1381.0   1144   6645     10981      1962
lm20-5340-bare            3  12.23    67.41    58.96    892.8    355   3758     11741      2500
lm20-5340-mpsl            3   1.80    99.53    82.25   2470.0      0   7660       487      1980
```

1M spot check（同样的 capacity-gated bench）：

```text
-----------------------------------------------------------------------------------------------
52840-lm20-mpsl-1m        3   2.97    99.54    97.71   5860.5      0  13000      1539      1537
5340-52840-bare-1m        3  19.20    53.28    86.17    802.3    305  20538     11643      1667
5340-52840-mpsl-1m        3   0.00    72.07    90.05   1631.8   1007  13000     11620      1824
```

1M 两侧必须同时启用；LM20 在 1M 下用 650 us MPSL slot，52/53 用
500 us，bare 统一 600 us。

## 非对称 idle ratio 当前矩阵（THUNDERS_RATIO=844）

```text
run                    wins   fwd% rev_raw% rev_arq%
52840-5340-bare-r844      3  23.47    67.42    32.40
52840-5340-mpsl-r844      3   3.85    75.65    65.63
52840-lm20-bare-r844      3   7.32    99.98    92.31
52840-lm20-mpsl-r844      3   3.25    87.53    78.78
5340-52840-bare-r844      3  34.54    51.65    52.92
5340-52840-mpsl-r844      3   3.21    54.29    62.41
5340-lm20-bare-r844       3  46.26    99.45    86.93
5340-lm20-mpsl-r844       3   3.24    54.27    62.49
lm20-52840-bare-r844      3  26.67    67.65    31.65
lm20-52840-mpsl-r844      3   6.01    95.30    92.52
lm20-5340-bare-r844       3   0.00   100.00     0.00
lm20-5340-mpsl-r844       3   0.00   100.00     0.00
```

`lm20 -> 5340` 在 844 下仍是当前 RF 死角。容量 API：
`Config::period_slots()` / `Config::tx_slots_per_period(role)`。

## 已知 RF 死角

`lm20 -> 5340`（LM20 central，5340 peripheral）在 bare/mpsl、2M/1M、
默认 ratio 和 `844` ratio 下均无法建链：

- LM20 central 的 TX 能被 5340 peripheral 收到；
- 5340 peripheral 的 TX（SlotRequest/echo）到不了 LM20 central；
- 5340 net-core RADIO 的 TX power 上限为 0 dBm，而 52840/LM20 已使用
  +8 dBm；
- 1M 模式没有改善，说明剩余约 3 dB 链路预算不足以覆盖该 RF 路径。

结论：这是当前硬件/RF 路径预算问题，不是 slot、ratio、ARQ 或 bench
工具问题。该 pair 标记为 unsupported-in-current-setup，后续应从天线/
距离/屏蔽或硬件 PA 入手。

## 代码位置索引

| 问题 | 文件 |
|---|---|
| MPSL slot 边界同步 | `thunders-phy-nrf/src/mpsl/{callback,mod,state}.rs` |
| central 固定、peripheral 采集扫描 | `thunders-phy-nrf/src/mpsl/callback.rs` |
| 各板 TX power 上限 | `thunders-phy-nrf/src/mpsl/radio.rs`、`src/radio_phy.rs` |
| 1M 可选模式 | 各 example `Cargo.toml`/`src/main.rs`、`scripts/bench.sh`、PHY mode/airtime |
| peripheral 采集占空比 | `thunders/src/link.rs` |
| 按窗口容量投放 + rev_arq 统计 | `thunders/src/link.rs`、各 example `src/main.rs`、`scripts/bench_parse.py` |
| bench 计时/清理/重试 | `scripts/bench.sh` |
| 标准组合过滤 | `scripts/bench_parse.py` |
