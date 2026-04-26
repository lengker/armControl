# VOFA配置示例代码

本文档提供VOFA中具体的数据包配置示例。

## VOFA配置核心思路

VOFA允许你定义"数据包模板"，然后绑定到GUI控件（滑块、按钮等）。
当用户操作控件时，VOFA会自动根据模板生成CAN/串口消息。

---

## 配置实例

### 1. VOFA中创建"设置X坐标"滑块数据包

**在VOFA中的步骤：**

1. 右键左侧"数据包"列表 → "新建数据包"
2. 输入包名：`Set_X_Coord`
3. 包格式选择：**发送原始数据包**（固定）
4. 设置数据内容如下：

```
包名称：Set_X_Coord
内容（十六进制）：
  FF 10 XX XX XX XX
  ↑  ↑  └─────┬─────┘
  │  │    float(X)
  │  └─ 命令0x10
  └─ 帧头

动态替换规则：第3-6字节为X坐标的4字节float表示
```

### 2. 创建滑块控件并关联数据包

**在VOFA中的步骤：**

```
①. 右击中央编辑区 → 添加控件 → 滑块(Slider)
②. 配置滑块属性：
    - 名称：X坐标滑块
    - 范围：0 ~ 500
    - 单位：mm
    - 显示精度：1位小数
③. 绑定数据包：
    - 控件右键 → "绑定数据包"
    - 选择：Set_X_Coord
    - 数据类型：float (小端)
    - 替换位置：从第3字节开始（原始数据[FF 10]后面）
```

### 3. Python代码示例（导出为VOFA脚本）

如果VOFA支持Python脚本导出，参考以下逻辑：

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import struct
import serial
import time

class VOFAController:
    def __init__(self, port='COM6', baud=115200):
        self.ser = serial.Serial(port, baud, timeout=1)
        time.sleep(0.5)
    
    def send_frame(self, cmd, data):
        """
        发送VOFA指令帧
        cmd: 命令字节 (0x10/0x11/0x12/0x20/0x21)
        data: 数据字节或数组
        """
        if isinstance(data, float):
            # float数据
            data_bytes = struct.pack('<f', data)  # 小端float
        elif isinstance(data, int):
            # 单字节整数
            data_bytes = bytes([data])
        else:
            data_bytes = data
        
        frame = bytes([0xFF, cmd]) + data_bytes + bytes([0, 0])  # 补充到7字节
        frame = frame[:7]  # 截断到7字节
        
        print(f"Sending: {frame.hex().upper()}")
        self.ser.write(frame)
    
    def set_position(self, x, y, z):
        self.send_frame(0x10, x)
        time.sleep(0.05)
        self.send_frame(0x11, y)
        time.sleep(0.05)
        self.send_frame(0x12, z)
    
    def pump_on(self):
        self.send_frame(0x20, 1)
        print("[Pump] ON")
    
    def pump_off(self):
        self.send_frame(0x20, 0)
        print("[Pump] OFF")
    
    def get_pump_status(self):
        self.send_frame(0x21, 0)
    
    def close(self):
        self.ser.close()

# 使用示例
if __name__ == '__main__':
    vofa = VOFAController(port='COM6')
    
    # 测试1：设置坐标
    vofa.set_position(300.0, 0.0, 200.0)
    time.sleep(1.0)
    
    # 测试2：抓取
    vofa.pump_on()
    time.sleep(2.0)
    vofa.pump_off()
    
    # 测试3：移动到新位置
    vofa.set_position(350.0, 50.0, 180.0)
    time.sleep(1.0)
    
    vofa.get_pump_status()
    vofa.close()
```

---

## VOFA按钮配置

### "吸取"按钮（按下开启，松开关闭）

```
按钮配置：
  名称：吸取
  类型：按钮(Button)
  
  事件处理：
    - 按下(MouseDown)：
        发送数据包：FF 20 01 00 00 00
    - 松开(MouseUp)：
        发送数据包：FF 20 00 00 00 00
```

### 方案B：两个独立按钮

```
按钮A - 开启气泵：
  名称：吸
  类型：按钮
  点击事件：发送 FF 20 01 00 00 00

按钮B - 关闭气泵：
  名称：放
  类型：按钮  
  点击事件：发送 FF 20 00 00 00 00
```

---

## VOFA文本显示反馈

创建文本框显示气泵状态：

```
文本框配置：
  名称：气泵状态显示
  类型：文本标签(Text/Label)
  数据源：串口接收关键字
  
  搜索关键字：[PUMP_STATE]
  提取模式：关键字后的数值
  
示例：
  接收字符串：[PUMP_STATE] 1
  显示内容：气泵: 开启 ✓
  
  接收字符串：[PUMP_STATE] 0
  显示内容：气泵: 关闭 ✗
```

---

## 十六进制数据包生成工具

以下是手工生成VOFA数据包的快速查表：

### float类型坐标转换表

| 坐标 | 十六进制(小端) | VOFA包(hex) |
|------|----------------|------------|
| X=0.0 | 00 00 00 00 | FF 10 00 00 00 00 |
| X=100.0 | 00 00 C8 42 | FF 10 00 00 C8 42 |
| X=300.0 | 00 00 96 43 | FF 10 00 00 96 43 |
| X=350.0 | 00 00 B0 43 | FF 10 00 00 B0 43 |
| X=500.0 | 00 00 FA 43 | FF 10 00 00 FA 43 |

### 在线float转换工具

访问：https://www.h-schmidt.net/FloatConverter/IEEE754.html

例如转换 350.0：
1. 输入 350.0
2. 点击"To Hex"
3. 得到 `43B00000`（大端）→ `00 00 B0 43`（小端）

---

## VOFA的"数据包模板"文件结构

如果想保存和分享VOFA配置，可导出为`.vofa`格式（JSON）：

```json
{
  "project_name": "RoboArm_VOFA_Control",
  "data_packets": [
    {
      "name": "Set_X_Coord",
      "format": "raw_hex",
      "content": "FF 10 XX XX XX XX",
      "dynamic_fields": [
        {
          "name": "x_value",
          "position": 2,
          "type": "float",
          "byte_order": "little_endian"
        }
      ]
    },
    {
      "name": "Pump_On",
      "format": "raw_hex",
      "content": "FF 20 01 00 00 00"
    },
    {
      "name": "Pump_Off",
      "format": "raw_hex",
      "content": "FF 20 00 00 00 00"
    }
  ]
}
```

---

## 快速测试检查清单

- [ ] VOFA已连接到正确的COM口（查看左上角连接状态）
- [ ] 波特率设为 **115200**
- [ ] X/Y/Z滑块的数据包格式为：FF 10/11/12 + float小端 + 补零
- [ ] 吸取按钮对应：FF 20 01 00 00 00
- [ ] 放置按钮对应：FF 20 00 00 00 00
- [ ] 串口终端显示 `target_x updated to ...` 说明命令被正确接收
- [ ] 串口显示 `[PUMP] ON/OFF` 说明气泵指令有效

---

**推荐测试顺序**

1. 先用串口助手发送原始16进制帧验证硬件通信
2. 再在VOFA中构建UI并测试
3. 观察串口输出确认每个命令的接收情况

---

最后修改：2026-04-04
