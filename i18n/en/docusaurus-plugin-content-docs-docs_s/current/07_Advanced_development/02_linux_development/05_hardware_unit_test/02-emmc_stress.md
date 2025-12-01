---
sidebar_position: 2
---

# eMMC Stress Testing

## Test Principle

The eMMC stress test uses the iozone tool to perform stress testing on eMMC storage devices, simulating various I/O operation patterns and measuring metrics such as read/write performance, throughput, and latency. The specific test principles are as follows:

- **Create test files**: iozone first creates one or more files on the eMMC storage. The size and number of these files can be customized to simulate different types of workloads. During file creation, iozone uses specific block sizes for read/write operations.
- **Perform various I/O operations**:
  - **Sequential Write**: Continuously writes data to the storage device to test the maximum write bandwidth.
  - **Sequential Read**: Continuously reads data to test the read bandwidth.
  - **Random Write**: Performs random writes to test the device's random write capability.
  - **Random Read**: Performs random reads to test the device's random read capability.
  - **Mixed Read/Write**: Simulates a combination of read and write operations to evaluate overall device performance.
  - **Re-write**: Modifies or overwrites existing files to test performance during file updates.
- **Data recording and analysis**: iozone records the execution time of each operation and calculates key metrics such as throughput and latency to generate a data report.

### Test Content

The eMMC stress test includes two test scripts: `emmc_performance_test.sh` and `emmc_stability_test.sh`, representing eMMC performance testing and eMMC stability stress testing, respectively:

#### eMMC Stability Stress Testing

**1. Test Objective:**

- **Stability stress test script**: The primary goal is to verify system stability through prolonged stress testing. Typically, the `-z` (file pre-filling) option and large file sizes (e.g., `-n` and `-g` parameters) are used to ensure reliability validation during extended operation.

**2. Command Analysis:**

- Stability test command:  
  `iozone -e -I -az -n 16m -g 2g -q 16m -f "$output_dir/iozone_data" -Rb "$output_dir/test_iozone_emmc_ext4_stability_${loop_num}.xls"`
- Parameter explanation:
  - `-e`: Enables extended testing, an iozone feature that executes additional test types such as rewrite, reverse write, and EOF writes.
  - `-I`: Enables direct I/O (`O_DIRECT`), bypassing the OS cache for direct disk reads/writes.
  - `-a`: Runs tests in automatic mode. iozone automatically tests various operations and file sizes, typically including sequential and random read/write tests.
  - `-z`: In this command, ensures files are pre-filled before each test, guaranteeing continuous read/write operations during testing and intensifying system stress—commonly used in stability tests.
  - `-n 16m`: Sets the minimum block size for write tests to 16MB, meaning tests start at 16MB and gradually increase.
  - `-g 2g`: Sets the maximum file size for testing to 2GB, meaning iozone will test read/write performance up to 2GB files.
  - `-q 16m`: Specifies the block size used by iozone during testing as 16MB, which also defines the memory buffer size.
  - `-f "$output_dir/iozone_data"`: Specifies the file path where test results are stored.
  - `-Rb "$output_dir/test_iozone_emmc_ext4_stability_${loop_num}.xls"`: Outputs test results in Excel format (.xls).

#### eMMC Performance Testing

**1. Test Objective:**

- **Performance test script**: Primarily measures storage device performance under various file and record sizes. The test evaluates performance across different scenarios using combinations of multiple `-r` (record size) and `-s` (file size) parameters.

**2. Command Analysis:**

- Performance test command:  
  `iozone -e -I -a -r 4K -r 16K -r 64K -r 256K -r 1M -r 4M -r 16M -s 16K -s 1M -s 16M -s 128M -s 256M -f "$output_dir/iozone_data" -Rb "$output_dir/test_iozone_emmc_ext4_performance_${loop_num}.xls"`
- Parameter explanation:
  - `-e`: Enables extended testing.
  - `-I`: Enables direct I/O, bypassing OS caching.
  - `-a`: Executes automatic mode testing, covering multiple read/write operations across various file and record sizes.
  - `-r 4K -r 16K -r 64K -r 256K -r 1M -r 4M -r 16M`: Specifies record sizes used during testing, affecting eMMC performance under different block sizes.
  - `-s 16K -s 1M -s 16M -s 128M -s 1G`: Specifies the range of test file sizes—from small 16KB to large 1GB—to evaluate read/write performance across different file sizes.
  - `-f "$output_dir/iozone_data"`: Specifies the location for storing test files.
  - `-Rb "$output_dir/test_iozone_emmc_ext4_performance_1.xls"`: Outputs test results in Excel format.

## Preparation

**1.** Run the `lsblk -f` command to ensure the eMMC storage device is properly mounted and has sufficient free space for testing.

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

In eMMC performance testing, the `-s 256MB` command creates files of the specified size for read/write tests (e.g., `-s 256M -f "$output_dir/iozone_data"`). Please ensure sufficient free space is available under the `/app` mount path to accommodate the largest file size used in testing.

**2.** Confirm that the two test scripts `emmc_performance_test.sh` and `emmc_stability_test.sh` exist under the path `/app/chip_base_test/02_emmc`.

```shell
02_emmc/
├── emmc_performance_test.sh
└── emmc_stability_test.sh
```

## Test Procedure

The stress test scripts support the `-h` suffix to display command parameter descriptions (applies to both scripts):

```shell
./emmc_performance_test.sh -h

Usage: ./emmc_performance_test.sh [options]

Options:
  -t <time>       Set the test duration (e.g., 2h for hours, 30m for minutes; default: 48h).
  -d <seconds>    Set the sleep time between loops in seconds (default: 30).
  -o <directory>  Set the output directory for logs (default: script's '../output' folder).
  -h              Show this help message and exit.
```

Parameter explanations:

- `-t <time>`: Sets test duration (e.g., `2h` for 2 hours, `30m` for 30 minutes; default is 48 hours).
- `-d <seconds>`: Sets sleep time between loops in seconds (default: 30 seconds).
- `-o <directory>`: Sets the log output directory (default: the `../output` folder relative to the script's location).
- `-h`: Displays help information and exits the script.

**Example:**  
For instance, the command:  
`./emmc_performance_test.sh -t 2h -d 10 -o /userdata/output`  
customizes the test duration to 2 hours, sets a 10-second interval between loops, and specifies `/userdata/output` as the output directory.

### eMMC Stability Test:

After completing the preparation steps, run the test command:

```shell
cd /app/chip_base_test/02_emmc/

./emmc_stability_test.sh
```

After running for some time, the output will appear as follows:

```shell
eMMC stability test starting...
Test configuration:
  Test duration: 2880 minutes
  Sleep duration: 30 seconds
  Output directory: /app/chip_base_test/output
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

**Key Information Description:**

- `Test duration`: Test duration: This is the test duration in minutes (i.e., 48 hours).
- `Sleep duration`: Sleep duration: During each test loop execution, the script waits for 30 seconds to allow the system and storage device sufficient time to recover, thereby minimizing potential fluctuations during testing.
- `Output directory`: Output directory: `/app/chip_base_test/output`
- `Command line used`: Command used: `iozone -e -I -az -n 16m -g 2g -q 16m -f /app/chip_base_test/output/iozone_data -Rb /app/chip_base_test/output/test_iozone_emmc_stability_1.xls`, with key parameters as follows:
  - File size range: minimum 16 MB, maximum 2 GB.
  - Record size range: from 4 KB to 16384 KB (i.e., 16 MB).
- `Key performance metrics`:
  - kB: indicates file size (in KB).
  - reclen: indicates record size (in bytes), i.e., the data block size for each I/O operation.
  - random write / rewrite / read / reread: These columns represent throughput for random write, rewrite, random read, and reread operations, respectively.
  - bkwd (Backward Read): Throughput for backward (reverse-order) read operations.
  - record write / record read: Throughput for sequential record write and read operations.
  - stride read / stride write: Throughput for stride (strided) read and write operations.
  - fwrite / frewrite / fread / freread: These columns represent throughput for direct I/O operations using the O_DIRECT flag.

### eMMC Performance Test:

After ensuring all preparations are complete, run the test command:

```shell
/app/chip_base_test/02_emmc/

./emmc_performance_test.sh
```

After running for a period of time, the output appears as follows:

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

**Key Information Description:**

- `Test duration`: Test duration: This is the test duration in minutes (i.e., 48 hours).
- `Sleep duration`: Sleep duration: During each test loop execution, the script waits for 30 seconds to allow the system and storage device sufficient time to recover, thereby minimizing potential fluctuations during testing.
- `Output directory`: Output directory: `/app/chip_base_test/output`
- `Command line used`: Command used: `iozone -e -I -a -r 4K -r 16K -r 64K -r 256K -r 1M -r 4M -r 16M -s 16K -s 1M -s 16M -s 128M -s 256M -f /app/chip_base_test/output/iozone_data -Rb /app/chip_base_test/output/test_iozone_emmc_performance_1.xls`
  - Record sizes include: 4 KB, 16 KB, 64 KB, 256 KB, 1 MB, 4 MB, 16 MB.
  - File sizes are set to: 16 KB, 1 MB, 16 MB, 128 MB, 256 MB.
- `Key performance metrics`:
  - kB: indicates file size (in KB).
  - reclen: indicates record size (in bytes), i.e., the data block size for each I/O operation.
  - random write / rewrite / read / reread: These columns represent throughput for random write, rewrite, random read, and reread operations, respectively.
  - bkwd (Backward Read): Throughput for backward (reverse-order) read operations.
  - record write / record read: Throughput for sequential record write and read operations.
  - stride read / stride write: Throughput for stride (strided) read and write operations.
  - fwrite / frewrite / fread / freread: These columns represent throughput for direct I/O operations using the O_DIRECT flag.

## Test Metrics

### eMMC Stability Test

After the test program starts, the stability test generates the following files in the `/app/chip_base_test/log` directory:

- test_iozone_emmc_stability.log: Logs status information during stress testing.
- test_iozone_emmc_stability_*.xls: Records data results from stress testing.

The test objective is to ensure the system can run stably for 48 hours without rebooting or hanging. To verify stability during testing, use the following command to check log files for anomalies such as "fail", "error", or "timeout":

```shell
cd "/app/chip_base_test/output/" && grep -iE 'error|fail|timeout' test_iozone_emmc_stability*.log
```

### eMMC Stability Test Results

After running the test for 24 hours and inspecting the log files, no abnormal status messages were found, indicating the stability stress test has passed.

```shell
Test loop 1 succeeded!
Test loop 2 succeeded!
Test loop 3 succeeded!
.....
```

### eMMC Performance Test

After the test program starts, the performance test generates the following files in the `/app/chip_base_test/output` directory:

- test_iozone_emmc_performance.log: Logs status information during stress testing.
- test_iozone_emmc_performance_*.xls: Records data results from stress testing.

The test objective is to ensure the system runs stably for 48 hours without rebooting or hanging. To check for anomalies in the logs, use the following command to search for keywords like "fail", "error", or "timeout":

```shell
cd "/app/chip_base_test/output/" && grep -iE 'error|fail|timeout' test_iozone_emmc_performance*.log
```

Additionally, performance should meet general standards for real-world usage. For RDKS100 (eMMC 5.1), which supports HS400 mode at maximum, typical read speeds range from 250 MB/s to 300 MB/s, while write speeds are slightly lower, typically between 120 MB/s and 200 MB/s.

### eMMC Performance Test Results

After 48 hours of testing, log inspection using the command revealed no abnormal status messages. According to the log output, the maximum read speed reached approximately 168 MB/s, and the maximum write speed was about 102 MB/s, confirming that the performance stress test has passed.

```shell
Test loop 1 succeeded!
Test loop 2 succeeded!
Test loop 3 succeeded!
.....
```