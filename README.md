# Quadcopter

Created by Chris Wang.
Quadcopter based on STM32F401RE.

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
