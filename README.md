# Quadcopter

Created by Chris Wang.	
Quadcopter based on STM32F401RE.

代码风格 注释doxygen 函数名 变量名 与cube一致

## Directory

### APP
- main.c
- main.h

### DRIVER
- motor
- mpu
- remoter

### FLIGHT
- ahrs
- pid
- ano

### HAL

- hal_conf.h
- stmf4xx_it.h .c中断
- i2c
- usart
- tim
- adc
- dma
- gpio

### KEIL
the keil's project and outputs

### LIB

startup file

system_stm32f4xx.c

hal libralies 

dsp lib 

### uCOS

the core、ports and config of the embedded operating system ucos_ii

- ucos_ii.c

  include all core c file h file

- os_cpu.h 、os_cpu_c.c、os_cpu_a.asm

  ucos_ii's port, should be modified according to the mcu you choose

- os_cfg.h、app_cfg.h

  config file, can be modified according to your demand

- app_hooks.c

  hook function

- os.h

  compability with ucos_iii

- os_trace.h、os_dbg.c

  related to debug

## Task

遥控器任务

传感器任务 姿态解算 pid

数据发送任务



TODO:

- 遥控器变量 volatile
- mpu改名sensor
- 什么样的变量才需要互斥访问 只读不改需要吗？
- 正点原子 匿名 任务划分
- tim1 arr最大化减少中断次数
- 中断函数加上 ucos中断管理
- pid dsp和自己写 比较效率
- ucos pendsv 改启动文件
- mpu量程选择
- 向上位机发送什么
- 文件顺序为字母顺序
- 去掉main.h 需要什么用什么



/**

 \* @brief 

 \* @param *ch* 

 \* @param *f* 

 \* @return int 

 */

int fputc(int *ch*, FILE **f*)

{

  UNUSED(*f*);

  HAL_UART_Transmit(&huart2, (uint8_t *)&*ch*, 1, 1000);



  return *ch*;

}

UseTab: Never

IndentWidth: 4

TabWidth: 4

BreakBeforeBraces: Allman

AllowShortIfStatementsOnASingleLine: false

IndentCaseLabels: false

ColumnLimit: 0

AccessModifierOffset: -4

NamespaceIndentation: All

FixNamespaceComments: false

ReflowComments: false

AllowShortLoopsOnASingleLine: true
