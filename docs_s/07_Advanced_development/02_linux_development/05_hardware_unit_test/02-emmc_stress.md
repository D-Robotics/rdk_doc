---
sidebar_position: 2
---

# eMMC 压力测试

## 测试原理

eMMC 压力测试通过 iozone 工具对 eMMC 存储设备进行压力测试，模拟不同的 I/O 操作模式，测量读写性能、吞吐量、延迟等指标。具体测试原理如下：

- 创建测试文件： IOzone 首先在 eMMC 存储上创建一个或多个文件，这些文件的大小和数量可以自定义来模拟不同类型的负载，创建文件时， IOzone 会使用特定的块大小来进行读写操作。
- 执行多种 I/O 操作：
  - 顺序写入（ Sequential Write）：连续写入数据到存储设备，测试设备的最大写入带宽。
  - 顺序读取（ Sequential Read）：连续读取数据，测试设备的读取带宽。
  - 随机写入（ Random Write）：对数据进行随机写入，测试设备的随机写入能力。
  - 随机读取（ Random Read）：对数据进行随机读取，测试设备的随机读取能力。
  - 混合读写：模拟多种读写操作，测试设备的综合性能。
  - 重写（ Re-write）：对已经存在的文件进行修改或覆盖，测试设备在更新文件时的性能。
- 数据记录与分析： IOzone 会记录每个操作的执行时间，并计算吞吐量和延迟等关键指标生成数据报告。

### 测试内容

eMMC 压力测试包含了 `emmc_performance_test.sh` 和 `emmc_stability_test.sh` 两个测试脚本，分别代表 eMMC 性能测试与 eMMC 稳定性压力测试：

#### eMMC 稳定性压力测试

**1. 测试目的：**

- 稳定性压力测试脚本：主要目的是通过长时间的压力测试验证系统的稳定性，通常会使用 -z（填充文件）和设置较大的文件（如 -n 和 -g 参数）来确保在长时间运行时对系统的可靠性进行验证。

**2. 命令解析：**

- 稳定性测试：`iozone -e -I -az -n 16m -g 2g -q 16m -f "$output_dir/iozone_data" -Rb "$output_dir/test_iozone_emmc_ext4_stability_${loop_num}.xls"`
- 参数解析：
  - -e：启用扩展测试 (extended test)，这是 iozone 的一个特性，可以执行更多的测试类型，如重写 (rewrite)、反向写 (reverse write) 和 EOF 写入等。
  - -I：启用直接 I/O（ O_DIRECT），绕过操作系统的缓存直接进行磁盘读写。
  - -a：执行自动模式测试。 Iozone 会自动测试不同的操作和文件大小，通常会进行顺序读写、随机读写等多种测试。
  - -z：在这个命令中，用于在每次测试时填充文件，以确保测试期间会进行连续的读写操作，增强了对系统压力的考验。这是常见的稳定性测试选项。
  - -n 16m：设置每次写入测试的最小数据块大小为 16MB。这意味着测试从 16MB 数据块开始，并逐渐增大。
  - -g 2g：设置测试的最大文件大小为 2GB。这表示 Iozone 将测试最大 2GB 文件的读写性能。
  - -q 16m：在执行测试时，设置 Iozone 使用的文件块大小为 16MB，且这个块大小是内存缓冲区的大小。
  - -f "$output_dir/iozone_data"：指定测试结果存储的文件路径。
  - -Rb "$output_dir/test_iozone_emmc_ext4_stability_$\{loop_num}.xls"：使用 Excel 格式 (.xls) 输出测试结果文件。

#### eMMC 性能测试

**1. 测试目的：**

- 性能测试脚本：主要用于衡量存储设备在不同文件和记录大小下的性能，测试通过多种记录大小和文件大小组合来评估不同场景下的性能，命令中有多种 -r（记录大小）和 -s（文件大小）参数。

**2. 命令解析：**

- 性能测试：`iozone -e -I -a -r 4K -r 16K -r 64K -r 256K -r 1M -r 4M -r 16M -s 16K -s 1M -s 16M -s 128M -s 256M  -f "$output_dir/iozone_data" -Rb "$output_dir/test_iozone_emmc_ext4_performance_${loop_num}.xls"`
- 参数解析：
  - -e：启用扩展测试。
  - -I：启用直接 I/O，绕过操作系统缓存。
  - -a：执行自动模式测试，包含多个读写操作，测试多个文件大小和记录大小。
  - -r 4K -r 16K -r 64K -r 256K -r 1M -r 4M -r 16M：指定测试中使用的记录大小，影响 eMMC 在不同数据块尺寸下的性能表现。
  - -s 16K -s 1M -s 16M -s 128M -s 1G：指定测试文件大小的范围。从较小的 16KB 到较大的 1GB，测试不同大小文件的读写性能。
  - -f "$output_dir/iozone_data"：指定测试文件存储位置。
  - -Rb "$output_dir/test_iozone_emmc_ext4_performance_1.xls"：输出测试结果为 Excel 格式。

## 准备工作

**1.** 输入 `lsblk -f` 命令确保 eMMC 存储设备的挂载状态正常，且有足够的存储空间进行测试。

```shell
# lsblk -f
NAME         FSTYPE FSVER LABEL    UUID                                 FSAVAIL FSUSE% MOUNTPOINTS
mmcblk0
|-mmcblk0p1
|-mmcblk0p2
|-mmcblk0p3
|-mmcblk0p4
|-mmcblk0p5
|-mmcblk0p6
|-mmcblk0p7
|-mmcblk0p8
|-mmcblk0p9
|-mmcblk0p10
|-mmcblk0p11
|-mmcblk0p12 ext4   1.0            a9e4b580-42a3-43cd-94f6-26cd8f75fd56   24.6M    47% /boot
|-mmcblk0p13 ext4   1.0            a9e4b580-42a3-43cd-94f6-26cd8f75fd56
|-mmcblk0p14 ext4   1.0   ota      6f9bb2c5-4460-4295-a68a-3d28aa23dfbc    7.4G     0% /ota
|-mmcblk0p15 ext4   1.0   log      612c6e0a-05eb-420e-8cef-fdf3380ab4d0    2.9G    21% /log
|-mmcblk0p16 ext4   1.0   userdata d7051400-ef66-4202-b63f-b795bdd66501  486.8M    69% /userdata
`-mmcblk0p17 ext4   1.0            a0d7870d-a01d-4a9e-8fe8-f1a7b159fee7   24.5G    39% /
mmcblk0boot0
mmcblk0boot1
```

在 eMMC 性能测试中，命令 -s 256MB 会创建指定大小的文件进行读写测试，例如，-s 256M -f "$output_dir/iozone_data"， 请注意在 `/app` 挂载路径下的剩余空间是否满足最大文件读写测试的需求。

**2.** 确认在 /app/chip_base_test/02_emmc 路径下存在 `emmc_performance_test.sh` 和 `emmc_stability_test` 两个测试脚本。

```shell
02_emmc/
├── emmc_performance_test.sh
└── emmc_stability_test.sh
```

## 测试方法

压测脚本支持输入后缀 -h 查看命令参数的说明 , 例如（同时适用与两个压测脚本）：

```shell
./emmc_performance_test.sh -h

Usage: ./emmc_performance_test.sh [options]

Options:
  -t <time>       Set the test duration (e.g., 2h for hours, 30m for minutes; default: 48h).
  -d <seconds>    Set the sleep time between loops in seconds (default: 30).
  -o <directory>  Set the output directory for logs (default: script's '../output' folder).
  -h              Show this help message and exit.
```

各参数解析如下：

- `-t <time>`：设置测试时长，例如 2h 表示 2 小时， 30m 表示 30 分钟，默认是 48 小时。
- `-d <seconds>`：设置循环之间的睡眠时间（以秒为单位），默认值是 30 秒。
- `-o <directory>`：设置日志输出目录，默认值是脚本所在目录的 ../output 文件夹。
- `-h`：显示帮助信息并退出脚本。

**示例：**
例如，使用命令： `./emmc_performance_test.sh -t 2h -d 10 -o /userdata/output` 自定义测试时长 2 小时， 10 秒循环间隔，输出目录为 /userdata/output 。

### eMMC 稳定性测试：

确保完成准备工作后，运行测试命令：

```shell
/app/chip_base_test/02_emmc/

./emmc_stability_test.sh
```

运行一段时间后，打印结果如下：

```shell
eMMC stability test starting...
Test configuration:
  Test duration: 2880 minutes
  Sleep duration: 30 seconds
  Output directory: /app/chip_base_test/log
loop_test: 1
        Iozone: Performance Test of File I/O
                Version $Revision: 3.489 $
                Compiled for 64 bit mode.
                Build: linux

        Contributors:William Norcott, Don Capps, Isom Crawford, Kirby Collins
                     Al Slater, Scott Rhine, Mike Wisner, Ken Goss
                     Steve Landherr, Brad Smith, Mark Kelly, Dr. Alain CYR,
                     Randy Dunlap, Mark Montague, Dan Million, Gavin Brebner,
                     Jean-Marc Zucconi, Jeff Blomberg, Benny Halevy, Dave Boone,
                     Erik Habbinga, Kris Strecker, Walter Wong, Joshua Root,
                     Fabrice Bacchella, Zhenghua Xue, Qin Li, Darren Sawyer,
                     Vangel Bojaxhi, Ben England, Vikentsi Lapa,
                     Alexey Skidanov, Sudhir Kumar.

        Run began: Wed Nov 22 08:28:46 2023

        Include fsync in write timing
        O_DIRECT feature enabled
        Auto Mode
        Cross over of record size disabled.
        Using minimum file size of 16384 kilobytes.
        Using maximum file size of 2097152 kilobytes.
        Using Maximum Record Size 16384 kB
        Excel chart generation enabled
        Command line used: iozone -e -I -az -n 16m -g 2g -q 16m -f /app/chip_base_test/log/iozone_data -Rb /app/chip_base_test/log/test_iozone_emmc_stability_1.xls
        Output is in kBytes/sec
        Time Resolution = 0.000001 seconds.
        Processor cache size set to 1024 kBytes.
        Processor cache line size set to 32 bytes.
        File stride size set to 17 * record size.
                                                              random    random     bkwd    record    stride
              kB  reclen    write  rewrite    read    reread    read     write     read   rewrite      read   fwrite frewrite    fread  freread
           16384       4    55907    68813    43361    42502    34044    64746    33578     61774     30137   235263   233180  3905668  3824589
           16384       8    81221   100104    69678    70623    59293    90806    56909    106010     54285   239120   245316  5734840  5483605
           16384      16   121686   150470   103277    99504    89824   137999    84926    145251     76522   237380   249554  7382994  7145026
           16384      32   154998   181618   127561   127261   120903   176306   116168    173954    100347   219207   252293  8502985  8410367
           16384      64   164194   186220   155888   156079   153358   215180   151573    228310    149445   246653   239001  9776835  9377906
           16384     128   217106   244035   212321   212280   212484   228983   212498    255261    212567   249767   250846  9822953  9245428
           16384     256   197041   261028   260493   260805   259927   265663   258818    250669    252585   254769   250091  8233027  7780028
           16384     512   202081   270639   310755   310039   293284   270823   292638    271752    296772   217107   249592  8497728  7757194
           16384    1024   190352   253050   315167   314074   307086   232952   304852    291712    305991   230519   252777  8467363  7858319
           16384    2048   214068   231021   315921   315969   312522   268264   311613    280990    313240   252278   237820  8111550  7539324
           16384    4096   269717   269823   316611   316776   316238   269221   314454    249649    313803   240665   251199  8192783  7502285
           16384    8192   273111   234372   318953   318972   319264   269748   318253    257137    317770   245938   253022  8217275  7543462
           16384   16384   202940   269646   319743   270483   319395   265410   319089    269957    320018   229632   255214  8090540  7554242
           32768       4    54066    64638    39354    38982    31632    63344    32313     63174     33163   219761   237241  3928981  3764336
           32768       8    92884   106133    70454    71452    60261    97361    54351    102527     49974   222602   244227  5713575  5523894
           32768      16   131871   142064    99654    99860    90072   134791    82454    141394     79341   240144   243749  7426801  7027319
           32768      32   170242   180050   129171   126850   120804   157536   116730    188782    104283   234844   254196  8873437  8395368
           32768      64   195311   211873   155443   153652   155365   195772   150359    221615    133568   244320   246057  9944351  9338277
           32768     128   216944   236646   212274   211917   209541   219317   213839    265331    212390   236153   251861 10100750  9272128
           32768     256   251307   253604   261157   261595   261587   259134   261871    268029    259291   234278   244457  8285042  7877148
           32768     512   241543   262614   312889   310467   293996   259139   293417    289316    293543   241047   235724  8268096  7934447
           32768    1024   252317   260456   315462   315367   306540   270108   305224    263404    306965   242704   246276  8000029  7474461
           32768    2048   225344   271310   316482   316449   312337   255497   312132    274519    286263   233461   251373  7984692  7543381
           32768    4096   259591   262680   317304   317883   316198   268896   316068    269906    313908   234326   246179  8095213  7594233
           32768    8192   151997   268295   318844   318730   318102   242257   317291    282331    317083   238814   253789  8043101  7599271
           32768   16384   242659   263429   319769   319775   319887   268440   319028    242125    318938   238324   243818  8096643  7572892
```

**关键信息说明：**

- `Test duration`：测试持续时间 : 这是测试的持续时间，单位为分钟（即 48 小时）。
- `Sleep duration`：睡眠时间 : 在每次循环执行测试时，脚本会等待 30 秒钟，可以让系统和存储设备有足够的时间进行恢复，减少测试过程中可能的波动。
- `Output directory`：输出目录：`/app/chip_base_test/log`
- `Command line used`：使用命令：`iozone -e -I -az -n 16m -g 2g -q 16m -f /app/chip_base_test/log/iozone_data -Rb /app/chip_base_test/log/test_iozone_emmc_stability_1.xls`，关键参数如下：
  - 文件大小范围：最小 16MB，最大 2GB。
  - 记录大小范围：从 4KB 到 16384KB（即 16MB）。
- `主要性能指标`：
  - kB：表示文件大小（单位： KB）。
  - reclen：表示记录大小（单位：字节），即每次读写操作的数据块大小。
  - random write / rewrite / read / reread：这些列分别表示在随机写入、重写、随机读取、重读时的吞吐量。
  - bkwd (Backward Read)：表示倒序读取（ Backward Read）操作的吞吐量。
  - record write / record read：表示记录顺序写入和顺序读取吞吐量。
  - stride read / stride write：表示跳跃读取（ stride read）和跳跃写入（ stride write）的吞吐量。
  - fwrite / frewrite / fread / freread：这些列表示基于 O_DIRECT 标志进行的直接 I/O 操作的吞吐量。

### eMMC 性能测试：

确保已完成准备工作后，运行测试命令：

```shell
/app/chip_base_test/02_emmc/

./emmc_performance_test.sh
```

运行一段时间后，打印结果如下：

```shell
        Output is in kBytes/sec
        Time Resolution = 0.000001 seconds.
        Processor cache size set to 1024 kBytes.
        Processor cache line size set to 32 bytes.
        File stride size set to 17 * record size.
                                                              random    random     bkwd    record    stride
              kB  reclen    write  rewrite    read    reread    read     write     read   rewrite      read   fwrite frewrite    fread  freread
              16       4     9117    10499    30465    28878    36615     8850    32643     18626     24610    12029    21684  1775099  1985134
              16      16    10177    26485    62496    61522    59929    14545    53142     26664     55562    24547    10760  3993221  3993221
            1024       4    56577    67823    44622    45509    36629    66827    35150     66670     34735   200186   211660  3970183  3778101
            1024      16   114286   139984   102052   101789    90716   131381    85992    139167     86144   211524   220268  7257394  7160597
            1024      64    37102   208637   155548   151680   155289   206451   154191    204271    154942   216996   228360  9569770  8822754
            1024     256   173500   205916   257928   258067   257989   206998   256909    206779    257603   192187   200438  7699755  7631350
            1024    1024   217413   244855   301892   302594   301871   245037   300814    243744    300835   221540   230728  6963241  7755368
           16384       4    52089    67825    44063    43392    34234    61255    34520     62737     32502   222993   222677  3930014  3865899
           16384      16   130293   148830   102421   100899    90598   136409    85619    152741     78490   252406   251524  7529411  7275131
           16384      64   170570   217548   157302   152955   155700   221124   150764    208553    148248   260120   253610  9839832  9183651
           16384     256   205811   261328   261758   261212   261182   261993   254650    262417    259947   217130   255413  8082927  7798569
           16384    1024   197879   253559   195796   315156   305323   269961   305090    257443    303378   224878   254283  8291637  7423670
           16384    4096   234452   266060   317604   317734   317150   236261   315150    282556    316232   232737   240104  8055449  7440550
           16384   16384   240055   270140   318581   317402   318656   242187   318959    269181    318860   241165   258027  8067744  7613666
          131072       4    56479    65254    38724    38878    24507    63658    31512     66954     29941   213707   231208  3996689  3912245
          131072      16   131802   140292    97772    99376    88566   124478    82700    144209     78159   230221   246694  7635518  7316341
          131072      64   204553   202647   155240   153775   153168   186635   150078    224102    130841   234680   249686  9918412  9275478
          131072     256   250039   251249   259159   259635   241930   231149   259701    281143    261231   230374   247556  8228546  7831747
          131072    1024   268136   259076   315126   315142   305432   244546   306057    288222    296339   233233   244865  8118454  7560646
          131072    4096   259529   259422   317268   317829   315557   248712   316373    284430    317030   233850   248057  8043150  7644011
          131072   16384   254348   261764   319717   291364   318772   260554   319036    261823    318974   233891   253259  8131062  7652523
          262144       4    55824    64514    39155    38960    22375    61074    32539     66794     30422   223029   225658  3986253  3910378
          262144      16   131122   139688    98845    98777    89264   118019    83554    145550     80158   226106   247385  7604771  7274702
          262144      64   209918   204716   154163   154048   155217   176537   145248    219781    132243   228970   238124  9886555  9151477
          262144     256   256556   250649   260172   259484   259429   210923   257072    281685    260446   221835   245302  8224656  7816814
          262144    1024   262673   263106   314717   312165   306281   232667   305545    288026    306395   223386   248159  7836425  7515828
          262144    4096   262847   260217   313293   318018   316159   250633   316430    280339    316726   231561   250969  7959412  7616677
          262144   16384   266675   259687   318466   318828   308448   255214   317642    263443    318205   231295   248930  8029629  7606981
```

**关键信息说明：**

- `Test duration`：测试持续时间 : 这是测试的持续时间，单位为分钟（即 48 小时）。
- `Sleep duration`：睡眠时间 : 在每次循环执行测试时，脚本会等待 30 秒钟，可以让系统和存储设备有足够的时间进行恢复，减少测试过程中可能的波动。
- `Output directory`：输出目录：`/app/chip_base_test/log`
- `Command line used`：使用命令：`iozone -e -I -a -r 4K -r 16K -r 64K -r 256K -r 1M -r 4M -r 16M -s 16K -s 1M -s 16M -s 128M -s 256M -f "$output_dir/iozone_data" -Rb "$output_dir/test_iozone_emmc_ext4_performance_${loop_num}.xls"`
  - 记录大小（ Record Size）包括 4KB, 16KB, 64KB, 256KB, 1MB, 4MB, 16MB。
  - 文件大小（ File Size）设置为 16KB, 1MB, 16MB, 128MB, 256MB。
- `主要性能指标`：
  - kB：表示文件大小（单位： KB）。
  - reclen：表示记录大小（单位：字节），即每次读写操作的数据块大小。
  - random write / rewrite / read / reread：这些列分别表示在随机写入、重写、随机读取、重读时的吞吐量。
  - bkwd (Backward Read)：表示倒序读取（ Backward Read）操作的吞吐量。
  - record write / record read：表示记录顺序写入和顺序读取吞吐量。
  - stride read / stride write：表示跳跃读取（ stride read）和跳跃写入（ stride write）的吞吐量。
  - fwrite / frewrite / fread / freread：这些列表示基于 O_DIRECT 标志进行的直接 I/O 操作的吞吐量。

## 测试指标

### eMMC 稳定性测试

测试程序启动后，稳定性测试会在 `/app/chip_base_test/log` 目录下生成以下文件：

- test_iozone_emmc_stability.log：记录压测时的状态信息。
- test_iozone_emmc_stability_*.xls：记录压测时的数据结果。

测试目标是确保系统能够在 48 小时内稳定运行，不发生重启或挂死的情况。为确保测试过程中的稳定性，可通过以下命令检查日志文件中是否存在 fail、 error、 timeout 等异常信息：

```shell
cd "/app/chip_base_test/log/" && grep -iE 'error|fail|timeout' test_iozone_emmc_stability*.log
```

### eMMC 稳定性测试结果

运行测试 24H 后检测 log 日志，并未出现异常状态信息，说明稳定性压测合格。

```shell
Test loop 1 succeeded!
Test loop 2 succeeded!
Test loop 3 succeeded!
.....
```

### eMMC 性能测试

测试程序启动后，性能测试将在 /app/chip_base_test/log 目录下生成以下文件：

- test_iozone_emmc_performance.log：记录压测时的状态信息。
- test_iozone_emmc_performance_*.xls：记录压测时的数据结果

测试目标是确保系统能够在 48 小时内稳定运行，期间不发生重启或挂死现象。为检查日志中的异常信息，可使用以下命令查找 fail、 error、 timeout 等关键字：

```shell
cd "/app/chip_base_test/log/" && grep -iE 'error|fail|timeout' test_iozone_emmc_performance*.log
```

此外，性能应符合实际使用中的通用标准。针对 RDKS100 （ eMMC 5.1 ），其最高支持 HS400 模式。通常，读取速度在 250 MB/s 到 300 MB/s 之间，写入速度略低，通常在 120 MB/s 到 200 MB/s 之间。

### eMMC 性能测试结果

经过 48 小时的测试，使用命令检查 log 日志时未发现异常状态信息且通过日志输出，最大读取速率可达到约 168 MB/s，最大写入速率约为 102 MB/s，性能压测合格。

```shell
Test loop 1 succeeded!
Test loop 2 succeeded!
Test loop 3 succeeded!
.....
```
