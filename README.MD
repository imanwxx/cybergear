[English](#English-version) | [中文](#中文版)
# 中文版
# CYBERGEAR电机控制代码

## 1. 硬件

- 树莓派5
- 微雪RS485 CAN HAT
- CYBERGEAR小米电机
-- 电机参数：功率160w

## 2. 依赖

### 2.1 创建虚拟环境

```bash
python -m venv cybergear
source ~/cybergear/bin/activate
pip install python-can numpy
```

### 2.2 微雪RS485 CAN HAT

```bash
wget https://github.com/joan2937/lg/archive/master.zip
unzip master.zip
cd lg-master
sudo make install 
sudo nano /boot/firmware/config.txt
```

将下面的代码复制到`config.txt`最后，`ctrl+o`写入，`ctrl+x` 退出。

```bash
dtparam=spi=on
dtoverlay=mcp2515-can0,oscillator=12000000,interrupt=25,spimaxfrequency=2000000
```

CAN通讯详情配置请参[微雪RS485 CAN HAT](https://www.waveshare.net/wiki/RS485_CAN_HAT)

### 2.3 克隆仓库

```bash
git clone https://github.com/imanwxx/cybergear.git
cd cybergear
python cybergear.py
```

目前实现的效果有修改电机id（需要在上电但未使能电机的情况下修改id）、使能电机、失能电机、位置控制、位置插值控制、力矩控制、阻尼模式、设置零位、监听电机数据（角度、角速度、力矩、温度）。

# English-version

# CYBERGEAR Motor Control Code
## 1. Hardware
- Raspberry Pi 5
- Waveshare RS485 CAN HAT
- CYBERGEAR Xiaomi Motor
## 2. Dependencies
### 2.1 Create Virtual Environment
```bash
python -m venv cybergear
source ~/cybergear/bin/activate
pip install python-can numpy
```
### 2.2 Waveshare RS485 CAN HAT
```bash
wget https://github.com/joan2937/lg/archive/master.zip
unzip master.zip
cd lg-master
sudo make install 
sudo nano /boot/firmware/config.txt
```
Add the following code to the end of config.txt, then press ctrl+o to save and ctrl+x to exit:

```bash
dtparam=spi=on
dtoverlay=mcp2515-can0,oscillator=12000000,interrupt=25,spimaxfrequency=2000000
```
Refer to [Waveshare RS485 CAN HAT](https://www.waveshare.net/wiki/RS485_CAN_HAT) for detailed CAN communication configuration.
### 2.3 Clone Repository
```bash
git clone https://github.com/imanwxx/cybergear.git
cd cybergear
python cybergear.py
```

Current functionality includes: motor enable/disable, position control, position interpolation control, torque control, damping mode, zero position setting, and real-time monitoring of motor data (angle, angular velocity, torque, temperature).

