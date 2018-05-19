# 应用说明

此应用移植自NXP Application Note AN5219

### 端口信息
S32K144：

UART：PTA2、PTA3

BTN：PTE7（EasyARM-S32K144）、PTC12（S32K144EVB-Q100）

LED：PTD4、PTD16

CAN：PTE4、PTE5

S32K148：

UART：PTA27、PTA28

BTN：PTC13

LED：PTE22

CAN：PTB0、PTB1

注意：S32K144使用8MHz外部晶振，S32K148 S32K Start Kit使用16MHz外部晶振，时钟配置请注意！

### 适用范围
 
文档主要介绍S32K14x的BootLoader应用的说明。此文档中部分内容适用于S32K14x系列产品，使用Xmodem进行文件传输的BootLoder应用可以参考文档中提到的操作方式。其他应用由于使用通信接口和使用文件传输方式的不同仅供部分参考。
与此文档关联提供的软件例程是使用NXP的S32 Design Studio（简写作S32DS）开发，测试硬件均使用评估板进行测试（S32K144EVB-Q100、TRK-KEA128、S32 Start Kit），如需在其他产品和硬件上进行开发和测试，需要用户自行在软件和硬件上做对应的修改。

### 操作步骤

可参考AN5219文档


