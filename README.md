#### 场景包含以下物体：

Cup:中间的方形玻璃杯

Cup2:右边的圆柱玻璃杯

Cup3:左边的三面玻璃杯

Paticles:多个粒子组合模拟水珠

#### 初始化:

加载100个粒子到方形玻璃杯Cup中

#### 键盘可控操作：

##### 对Cup进行操作【均是GetKey类型，需要长按】

+ WSAD——在XY平面内上下左右移动
+ Z——在XY平面内向左倾倒
+ C——在XY平面内向右倾倒

+ X——改变杯子x方向和z方向的朝向，组合效果为摇晃杯子
+ R——杯子的位置和朝向复原，粒子全部消失，即回到初始状态
+ T——只复原杯子的位置和朝向

##### 添加水

按键1，向杯子Cup中加水，防止卡顿的情况下，至多400个粒子

在Controller控件中可以看见实时加载出来的粒子数pNumActive



#### 演示示例：

初始化之后按键1，加满水；然后按键Z，将水从Cup倒入Cup2中

按键R重置，按键1，加满水；然后按键C，将水从Cup倒入Cup3中

按键R重置，按键1，加满水；按键X，摇晃Cup

#### 材料参考：

玻璃材料和粒子材料来源于https://github.com/king-mork/SPH-Project

