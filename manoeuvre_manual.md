# 四个按钮
## Btn_ArmSwitch
## Btn_Joint1
## Btn_joint2
## Btn_Hand

# 操作流程
1. 上电前，两只机械臂都转到指定位置，然后上电
2. 按下`Btn_Joint1`/`Btn_joint2`/`Btn_Hand`中任意一个，机械臂移动到闲置位置
3. 按下`Btn_Joint1`/`Btn_joint2`，机械臂进入允许运动状态，同时移动到该状态下指定的初始位置
4. 通过`Btn_Joint1`切换90°/180°，通过`Btn_Joint2`在预设的两上下位置间切换
5. 对准后，按下`Btn_Hand`，会伸出吸盘，吸到后回到闲置位置
6. 如果没吸到要重来，此时再按`Btn_Joint1`/`Btn_joint2`回到第3步之前的状态（还需要再按下`Btn_Joint1`/`Btn_joint2`让机械臂进入允许运动状态）
7. 到达九宫格，按下`Btn_Hand`，将KFS放下，回到第2步之后的状态