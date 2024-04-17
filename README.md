# NewLegends_Darts
 NewLengends战队飞镖电控代码

操作说明：
1、右上控制底盘电机转向 通过遥控器通道CH3、CH4控制电机转向
2、右中控制摩擦轮  装填逻辑与步兵相同
3、换弹目前采用步进电机模式，若想改为无刷电机可以用原有注释的代码   lol

接线说明：
1、DM542说明：
    分配GPIO口PB14(DIR_YAW)、PB15(DIR_PITCH)、PF1（DIR_TRIGGER) 
    PUL给脉冲驱动、DIR控制电机旋转方向
    电机线 红  绿  黄  蓝 (实际看步进同相线序)
          A+  A-  B+  B-
    PUL+接5v
    PUL-接SPI2_CS(YAW)或SPI2_CLk(PITCH)或I2C2_SCL(TRIGGER)
    DIR+接5v
    DIR-接SPL2_MISO(YAW)或SPL2_MOSI(PITCH)或I2C2_SDA(TRIGGER)

     ______________________             ________________________
    |  YAW_DIR-       PITCH_DIR-      PITCH_PUL-      YAW_PUL-  |
    | TRIGGER_DIR-   TRIGGER_PUL-        GND             5v     |
    |___________________________________________________________|
                 
                 
                     （C板IO口接线）      
2、摩擦轮电机ID：
L1:1       R1:5        
L2:2       R2:6
L3:3       R3:7

3、3508拨盘ID：4 推杆ID：8

TODO:
1、制导飞镖
2、换弹改为竖列式发射