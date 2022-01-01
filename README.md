# NewLegends_Darts
 NewLengends战队飞镖电控代码

操作说明：
1、右上+左上控制底盘电机转向 通过遥控器通道CH3、CH4控制电机转向
2、右中控制摩擦轮  装填逻辑与步兵相同
3、右下AUTO,完成自动发射流程

接线说明：
1、推杆电机驱动：    分配GPIO口PB14、PB15
    推杆接OUT1、OUT2
    需要接跳线帽5V使能、ENA

2、DM542说明：
    分配GPIO口PB12(DIR_YAW)、PB13(DIR_PITCH) 
    PUL给脉冲驱动(PWM)、DIR控制电机旋转方向
    电机线 红  绿  黄  蓝 
          A+  A-  B+  B-
    PUL+接PWM
    PUL-接GND
    DIR+接GPIO口
    DIR-接GND

TODO:
1、修改遥控器控制方案
2、增加推杆控制
3、测试拨盘
4、增加PITCH、YAW电机的GPIO口控制