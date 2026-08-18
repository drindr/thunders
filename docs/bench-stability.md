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
1820–1925/s，出现2/8 profile失步；因此能力协商必须使用**实测PHY下限**
而非纯airtime预算。当前2M板广告500，1M LM20广告650；以后只有验证过
的backend才可广告更短值。

验证：52840→5340连续 **8/8**（最高tx=506/rx=505）；完整六个MPSL
方向 **6/6**，forward loss 0–0.13%，rev_arq 0.12–0.38%，包括原弱点
LM20↔5340。bare继续走原uniform cadence，不参与profile。

#### 4c-7. API触发的包长合同协商与有界稳定性Probe

应用不会因某一颗包变长而自动修改slot。central或peripheral显式调用
`negotiate_cadence(TrafficContract, CadenceProbePolicy)`后，正常`frame()`循环
驱动以下状态机：

```text
Request（peripheral API时）→ Offer → Accept
→ Probe(start,end) → Armed → Sample → Report
→ Commit(apply_epoch) → Applied → 双方同epoch生效 → Data确认 → Stable
```

central仍是唯一的candidate和绝对epoch决策者。Probe发送紧凑的
`CadenceSample`，其真实序列化wire长度匹配合同的最坏Data长度；同时采样两端的
slot wall time、slot/发送/ADDRESS成功数、`op_late`、长帧CRC和ARQ delivery
failure。搜索从当前稳定short向下按固定步长尝试；首次失败即停止，最终值为最低
通过candidate加policy安全档，并且不超过原稳定值。

生产API只允许candidate处于**双方backend已经过硬件验证的floor及以上**。曾尝试
在线Probe 475/450；失败候选会让两颗芯片的MPSL硬件计数以不同wall rate前进，
之后即使恢复相同slot period，原absolute epoch映射也已失效。全600在线恢复实验
仍不能可靠修复这个计数差，因此低于验证floor的实验不会暴露给生产API。未来某
backend离线验证并降低其floor后，同一协商/Probe状态机才会使用更短candidate。
包长决定需要覆盖的最坏wire长度，但不能绕过芯片对的验证floor。

最终profile生效后，超过对应方向合同长度的`frame()`发送明确返回
`PayloadExceedsCadenceProfile`，不会自动扩slot或偷偷重协商。协商期间ARQ retry
age被冻结，避免Probe占用slot导致应用数据超时；forward完整描述携带ACK/NACK，
脆弱reverse方向改用短`CadenceAck`承载Accept/Armed/Report/Applied。

最终切换使用两阶段arm：central持续发送未来Commit但不先切换；peripheral arm后
持续发送精确`{generation, apply_epoch}` Applied；central收到后才arm。Applied
若迟到越过epoch，central滚动到新的未来phase-0 epoch重新Commit。central生效并
恢复正常Data/Ack后，peripheral才停止Applied并进入Stable。因此丢失整个Commit
窗口只会延迟协商，不会形成单边profile。

PHY保存`active + pending + probe overlay`三个profile层次；合法候选只在预先协商
的`[probe_start, probe_end)`生效，结束后由MPSL callback自动恢复active profile。
profile字段用release/acquire fence发布给MPSL IRQ。

可选bench feature `cadence-probe`可由`CADENCE_PROBE=1 scripts/bench.sh build`
启用。52840→5340、8B/8B合同在双方500 µs验证floor下完成Offer/Accept/Commit，
最终仍为 **500/600 µs**；成功运行中central保持 **1922 slots/s**，peripheral
约 **1887–1921 slots/s**，严格ping-pong仍完成逐包echo。采集期仍存在历史性的
run-level启动波动，失败运行发生在API触发前的全600 acquisition，不属于合同状态机。

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
