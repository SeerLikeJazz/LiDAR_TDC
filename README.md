# LiDAR_TDC
Time-of-flight (ToF) measurement using pulse lasers

### 24.02.15

| Signal | Pin|
| -------- | --- |
| LASER_DAC          |   |
| LASER_PULSE        | E11  |
| LASER_PULSE_ENABLE | E9 |
| APD_PWM            | E13 |
| APD_FB_VOLT        |   |
| COMP_DAC           | A4 |
| \ | \ |
| TDC_INT            | C4 |
| CSN                | A15 |
| SCK                | A5 |
| MOSI               | A7 |
| MISO               | A6 |
| START              | E14 |
| EN_START           |  |
| \ | \ |
| USB_DP             | A12 |
| USB_DM             | A11 |

FIRE_UP 不使用  
定时器控制START  
stmw32F407生成3路测试信号，移植驱动  
TDC具体配置、运行模式看手册