# Luckfox Pico Zero 读取银尔达 M100MG-B1 DTU 定位数据（C 语言 / Buildroot）

注意需要先通过模块厂商的私有iot和dtu平台配置完成后，才能看到定位信息。
https://yinerda.yuque.com/yt1fh6/iot/mmtu92gx798qmo2n
https://yinerda.yuque.com/yt1fh6/4gdtu/mgp5olalo7norg03 还有这个定位数据的格式解析说明要看一下

通过串口（UART3）让 Luckfox Pico Zero 读取银尔达 M100MG-B1 DTU（4G + GPS/北斗）的定位数据。
纯 C 实现，适合 Buildroot 精简系统：

- 定时查询并解析定位信息（经度、纬度、速度、海拔、方向、GPS 时间）
- 兼容 DTU 的 `config` 指令返回格式和标准 NMEA 语句
- 无第三方依赖，仅使用标准 POSIX 接口（termios / poll）
- 在 PC 端用 SDK 交叉编译器编译，`adb push` 到板子即可运行

## 1. 硬件接线

### 1.1 使用的引脚

推荐使用 **UART3**，对应 40 针排针上的引脚 3 / 5 / 6：

| Pico Zero 引脚 | 功能 | 接 DTU |
| --- | --- | --- |
| Pin 5 | UART3_RX（GPIO1_A1） | DTU TXD |
| Pin 6 | GND | DTU GND |
| Pin 3 | UART3_TX（GPIO1_A0） | DTU RXD |

> 注意：Pin 8 / Pin 10 是 UART2（调试串口/控制台），**不要占用**。

### 1.2 供电

- DTU 需要 **5~12V 独立供电**（推荐 12V/1A），VIN 接正极、GND 接负极。板子的 3.3V/5V 排针供电能力有限，不要用它给 DTU 供电。
- Pico Zero 通过 USB-C 供电。
- 有条件的话，把 DTU 的 **RST 引脚** 接到 Pico Zero 的一个 GPIO（3.3V 兼容），程序跑飞时可以做异常恢复。

### 1.3 其他可用串口（备选）

| 串口 | TX | RX | 就近 GND |
| --- | --- | --- | --- |
| UART3 m0（推荐） | Pin 3 | Pin 5 | Pin 6 |
| UART3 m1 | Pin 13 | Pin 15 | Pin 14 |
| UART4 m1 | Pin 32 | Pin 33 | Pin 30 / 34 |
| UART0 m1 | Pin 27 | Pin 28 | Pin 25 / 30 |
| UART0 m0 | Pin 35 | Pin 37 | Pin 39 |

若改用其他串口，把程序里的端口 `/dev/ttyS3` 改成对应设备（UART0→`/dev/ttyS0`、UART3→`/dev/ttyS3`、UART4→`/dev/ttyS4`）。

## 2. 使能 UART3

### 方式一：写入 SDK 设备树（推荐，零手动操作）

把使能写进设备树、编译进固件，开机自动生效，之后不需要在板子上敲任何命令。

1. 在 SDK 目录执行 `./build.sh lunch` 选择 Luckfox Pico Zero 板级支持
2. 打开 `<luckfox-pico SDK>/config/dts_config`（该文件是 `rv1106g-luckfox-pico-zero.dts` 的软链接）
3. 在文件末尾追加（或直接使用本工程里的 `uart3_enable.dts`）：

```c
&uart3 {
    status = "okay";
    pinctrl-names = "default";
    pinctrl-0 = <&uart3m0_xfer>;   /* Pin 3 = UART3_TX, Pin 5 = UART3_RX */
};
```

4. 重新编译内核镜像并烧录即可，`/dev/ttyS3` 开机自动存在

> 为什么程序不能自己"使能"串口：`/dev/ttyS3` 是内核在启动时根据设备树创建的设备节点。设备树里 uart3 默认是 disabled，运行时程序无法凭空创建这个节点，所以要把使能写进设备树（固件）而不是程序里。这也是 Luckfox 官方支持的配置方式。

### 方式二：luckfox-config（临时测试用）

开发板终端执行：

```bash
luckfox-config
```

进入 `Advanced Options -> UART`，选中 UART3 并 `enable`。如果提示引脚冲突，先禁用冲突的配置再使能。UART 配置保存后立即生效，无需重启。

查看是否生效：

```bash
ls /dev/ttyS*
```

应能看到 `/dev/ttyS3`。

## 3. 快速验证串口

```bash
# 查看设备
ls /dev/ttyS*

# 设置波特率
stty -F /dev/ttyS3 ispeed 115200 ospeed 115200

# 向 DTU 发一条查询指令（\r\n 结尾）
printf 'config,get,gps\r\n' > /dev/ttyS3
cat /dev/ttyS3
```

若 DTU 正常连接且 GPS 已定位，应看到类似：

```
config,gps,ok,E,113.9739056,N,22.6927826
```

## 4. 编译（PC 端 SDK）

在 PC 端 Luckfox SDK 交叉编译环境中：

```bash
export PATH=<SDK目录>/tools/linux/toolchain/arm-rockchip830-linux-uclibcgnueabihf/bin:$PATH
make            # 或手动: arm-rockchip830-linux-uclibcgnueabihf-gcc -O2 -o dtu_gps dtu_gps.c
```

## 5. 部署到开发板并运行

```bash
adb push dtu_gps /root/
adb shell chmod +x /root/dtu_gps
adb shell /root/dtu_gps
```

正常输出示例：

```
打开串口 /dev/ttyS3 @ 115200 bps ...
GPS 定位已开启（config,set,location,2,1,5,0,0,1）
14:02:31 | 已定位 | 经度 113.973906 | 纬度 22.692783 | 速度 12.5 km/h | 海拔 184.0 m
```

支持参数：

```bash
/root/dtu_gps --port /dev/ttyS3 --baud 115200 --interval 5
```

`--port` 换串口、`--baud` 换波特率（1200~460800）、`--interval` 设查询间隔秒数。

> 提示：Buildroot 系统没有板载编译器，需要在 PC 上交叉编译后传输。如果改了参数想常驻运行，可参考 Wiki 的"自启动配置"把程序加入开机启动。

## 6. DTU 串口协议说明

银尔达 DTU 透传固件通过 TTL 串口支持以下指令（行尾 `\r\n`）：

| 指令 | 说明 |
| --- | --- |
| `config,set,location,2,1,5,0,0,1` | 开启 GPS 定位：类型 2=GPS、开启、周期 5 秒、上报类型 0=不上报（仅串口查询） |
| `config,get,gps` | 查询简单定位，返回 `config,gps,ok,E,经度,N,纬度` |
| `config,get,gpsext` | 查询扩展定位：定位标记、经纬度、速度、海拔、方向、GPS 时间 |
| `config,get,lbsloc` | 查询基站定位（城市精度约 500m，可能失败） |

返回数据为 **WGS84 坐标系**。`config,get,gpsext` 返回格式：

```
config,gps,ok,1,E,113.9739056,N,22.6927826,12.5,6.7,184,-3.6,90,2026-08-10 2:43:51
             |  |          |          |     |    |    |    |  |  └─ GPS时间
             |  |          |          |     |    |    |    |  └─ 方向角
             |  |          |          |     |    |    |    └─ 椭球高
             |  |          |          |     |    |    └─ 海拔(米)
             |  |          |          |     |    └─ 速度(海里/小时)
             |  |          |          |     └─ 速度(公里/小时)
             |  |          |          └─ 纬度(十进制, WGS84)
             |  |          └─ 纬度类型 N/S
             |  └─ 经度(十进制, WGS84)
             └─ 经度类型 E/W
fix=1 表示定位成功，fix=0 表示未定位
```

程序启动时会自动发送 `config,set,location,2,1,5,0,0,1` 开启 GPS，之后每 5 秒查询一次。若你的 DTU 已配置为向服务器自动上报，可把上报类型改为 2 并自定义上报内容（参考银尔达官方文档），不影响本程序串口查询。

## 7. 坐标说明

- DTU 返回的是 **WGS84** 坐标系（GPS 原始坐标）。
- 国内地图（高德、腾讯）使用 **GCJ-02**，直接使用 WGS84 会有几十到几百米偏移。本程序当前直接输出 WGS84，如需 GCJ-02 可在 `handle_line()` 中按需加转换算法。
- 百度地图使用 BD-09，需要再做一次转换。
- GPS 必须在室外空旷处才能定位，室内无法定位；基站定位精度差且可能失败。

## 8. 常见问题

| 现象 | 排查 |
| --- | --- |
| 打开串口失败 | 是否执行了 `luckfox-config` 使能 UART3；`ls /dev/ttyS*` 是否有 `ttyS3` |
| 无任何返回 | TX/RX 是否交叉；GND 是否共地；DTU 是否上电（NET LED 状态） |
| 返回乱码 | 波特率不匹配，确认 DTU 实际波特率（默认一般 115200，也有 9600） |
| fix=0 或经纬度为 0 | GPS 未定位：天线接好、放到室外空旷处、等待冷启动（≤35s） |
| 数据时有时无 | 检查接触；DTU 发送完数据约 12 秒后进入低功耗，查询间隔建议 ≥5 秒 |
| 想换成其他串口 | 改 `--port /dev/ttySx` 并确认引脚复用与接线 |

## 9. 文件说明

| 文件 | 说明 |
| --- | --- |
| `dtu_gps.c` | C 串口读取程序（交叉编译，Buildroot 使用） |
| `Makefile` | 编译脚本（`make`） |
| `uart3_enable.dts` | 设备树使能 UART3 的代码片段（追加到 SDK 的 config/dts_config） |
| `_ref/` | 参考资料：Pico Zero 引脚图、设备树源码等 |
