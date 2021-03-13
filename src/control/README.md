处理手动控制：
需要订阅/BlueRov2/Setting/arm topic, 然后，将gamepad输入转化为 /BlueRov2/Command/joy,
由commander处理

存在的问题：还是这个锁定不够灵敏，多次发送才能成功

下面处理定深、定向、定速.

定深pub的topic有:
/BlueRov2/bar30 获取深度传感器数据
/BlueRov2/Settings/set_depth depth设置，包括标志位、pwm_max、pid参数
/BlueRov2/Settings/set_target设置，包括目标深度、姿态、速度

发布的topic是:
/BlueRov2/Command/depth 通过commander处理

修改Set_depth.msg,删除Set_target.msg,将target分别放入各个msg

commander也订阅了set_depth 这里不用修改

现在修改ui

深度控制这里的问题是，手动控制和自动控制的flag无法通过界面修改，现在临时默认是自动，
另外，那个mode有什么用，manual模型下也可以自动，mode是指px4的模式，这里的自动实际上也是
通过manual模式实现的, 
这里可以自定义一些模式mymode auto和manual

速度控制尚未实现的



