#import "/template/template.typ": *

= 部分算法分析



== 舵轮底盘控制器

约定复数 $a + b iu$（$iu$ 为虚数单位）表示向量 $lr((a comma b))$。

底盘控制器接收底盘的目标水平控制速度 $dot(vb(r)) = dot(x) + iu dot(y)$ 和目标旋转控制速度 $dot(theta)$，
并输出轮电机和舵电机的目标控制力矩。

底盘的控制频率为 $1 thin "kHz"$，意味着每帧计算时间 $lt.eq 1 thin "ms"$。
因此底盘控制器需要尽可能降低运算时间，提高实时性。

=== 舵轮底盘建模

对于舵轮底盘，设其质量为 $m$，转动惯量为 $J$，轮的几何中心与底盘中心的水平距离均为 $R$。
假定底盘为刚体，质量中心位于底盘中心。

对于舵轮，设轮半径为 $r$，轮与地面的静摩擦系数为 $mu$。
假定轮为轻质刚性轮，无质量，无转动惯量，与地面不发生滑动摩擦，轮的 $z$ 方向（舵向）旋转轴穿过轮的几何中心。
忽略轮的宽度，即认为 $z$ 方向（舵向）的旋转是无摩擦的。

以舵轮底盘的中心为原点，从原点出发，逆时针依次选择两相邻轮，构成的射线为 $x comma y$ 轴，建立自然坐标系。

逆时针依次遍历位于 $x$ 正，$y$ 正，$x$ 负，$y$ 负处的轮，分别记其编号 $i$ 为 $1 comma 2 comma 3 comma 4$
。

对于任意轮 $i$，将右手拇指指向轮电机输出轴方向，定义右手四指方向为轮转速 $omega_i$ 的正方向。
令轮以速度 $omega_i > 0$ 旋转，定义轮中心正上方的的线速度方向为舵的正方向。
设轮的正方向与 $x$ 轴的夹角为 $alpha_i$，轮中心与底盘中心连线与 $x$ 轴夹角为：$ phi_i = cases(delim: "{", 0 comma & i = 1 comma, pi slash 2 comma & i = 2 comma, pi comma & i = 3 comma, 3 pi slash 2 comma & i = 4 dot.basic) $

若观测其输出轴转速为 $omega_i$。若不考虑打滑，底盘在此处相对地面的速度大小为 $u_i = omega_i r$，速度向量为 $vb(u)_i = omega_i r e^(iu alpha_i)$。

=== 观测底盘速度

以 $t = 0$ 时刻的自然坐标系为基准，建立世界坐标系。

在世界坐标系下，底盘有三个自由度 $x comma y comma theta$。显然 $t = 0$ 时刻 $lr((x comma y comma theta)) = (x_0 comma y_0 comma theta_0) = 0$。

对于轮 $i$，在任意 $t$ 时刻，其中心位置向量为：
$vb(r)_i (t) = x + iu y + R e^(iu lr((theta + phi_i)))$。

其中心速度向量为：
$vb(v)_i (t) = dot(x) + iu dot(y) + iu R e^(iu lr((theta + phi_i))) dot(theta)$。

令 $t = 0$，则轮中心速度向量为：
$vb(v)_i (0) = dot(x)_0 + iu dot(y)_0 + iu R e^(iu phi_i) dot(theta)_0$。

为了求解该时刻的底盘实际速度 $dot(x)_0 comma dot(y)_0 comma dot(theta)_0$，代入速度观测量，可得方程组：
$ cases(vb(v)_i (0) = vb(u)_i comma quad & i = 1 comma 2 comma 3 comma 4) $

展开为实数形式：
$
  cases(
    dot(x)_0 - R dot(theta)_0 sin lr((phi_i)) = r omega_i cos lr((alpha_i)) \
    dot(y)_0 + R dot(theta)_0 cos lr((phi_i)) = r omega_i sin lr((alpha_i))
  ), quad & i = 1, 2, 3, 4
$

继续展开：
$
  mat(dot(x)_0, dot(y)_0 + R dot(theta)_0; dot(x)_0 - R dot(theta)_0, dot(y)_0; dot(x)_0, dot(y)_0 - R dot(theta)_0; dot(x)_0 + R dot(theta)_0, dot(y)_0) = r mat(omega_1 cos lr((alpha_1)), omega_1 sin lr((alpha_1)); omega_2 cos lr((alpha_2)), omega_2 sin lr((alpha_2)); omega_3 cos lr((alpha_3)), omega_3 sin lr((alpha_3)); omega_4 cos lr((alpha_4)), omega_4 sin lr((alpha_4)))
$

共 $8$ 个方程，和 $3$ 个未知数，显然这是一个超定方程组，可以通过构造特征方程求最小二乘解得到近似解：
$
  dot(x)_0 & = r / 4 lr((omega_1 cos alpha_1 + omega_2 cos alpha_2 + omega_3 cos alpha_3 + omega_4 cos alpha_4)) comma\
  dot(y)_0 & = r / 4 lr((omega_1 sin alpha_1 + omega_2 sin alpha_2 + omega_3 sin alpha_3 + omega_4 sin alpha_4)) comma\
  dot(theta)_0 & = - frac(r, 4 R) lr((- omega_1 sin alpha_1 + omega_2 cos alpha_2 + omega_3 sin alpha_3 - omega_4 cos alpha_4)) dot.basic
$

得到底盘的观测速度。

=== 闭环底盘速度 <head:底盘控制器.闭环底盘速度>

通过PID（比例-积分-微分）控制器，基于底盘的目标控制速度 $lr((dot(x) comma dot(y) comma dot(theta)))$ 与观测速度 $(dot(x)_0 comma dot(y)_0 comma dot(theta)_0)$，闭环底盘的目标控制加速度 $lr((ddot(x) comma ddot(y) comma ddot(theta)))$。

$
      ddot(x) & = "PID"_"velocity" lr((dot(x) - dot(x)_0)) comma             \
      ddot(y) & = "PID"_"velocity" lr((dot(y) - dot(y)_0)) comma             \
  ddot(theta) & = "PID"_"velocity" lr((dot(theta) - dot(theta)_0)) dot.basic \
$

=== 计算电机控制力矩

==== 轮电机 <head:底盘控制器.计算电机控制力矩.轮电机>

在每个控制帧内，设底盘以观测速度为初速度，做加速度为目标控制加速度的匀加速运动，则在每个控制帧内，$lr((dot(x)_0 comma dot(y)_0 comma dot(theta)_0)) = upright("const")$，$lr((ddot(x) comma ddot(y) comma ddot(theta))) = upright("const")$，有：
$
  vec(dot(x), dot(y), dot(theta)) = vec(dot(x)_0 + ddot(x) t, dot(y)_0 + ddot(y) t, dot(theta)_0 + ddot(theta) t)
$

$
  vec(x, y, theta) = vec(dot(x)_0 t + 1 / 2 ddot(x) t^2, dot(y)_0 t + 1 / 2 ddot(y) t^2, dot(theta)_0 t + 1 / 2 ddot(theta) t^2)
$

于是 $t$ 时刻轮中心位置向量：
$
  vb(r)_i (t) = dot(x)_0 t + 1 / 2 ddot(x) t^2 + iu lr((dot(y)_0 t + 1 / 2 ddot(y) t^2)) + R e^(iu lr((phi_i + dot(theta)_0 t + 1 / 2 ddot(theta) t^2)))
$

$t$ 时刻轮中心速度向量：
$
  dot(vb(r))_i (t) = dot(x)_0 + ddot(x) t + iu lr((dot(y)_0 + ddot(y) t)) + iu R e^(iu lr((phi_i + dot(theta)_0 t + 1 / 2 ddot(theta) t^2))) lr((dot(theta)_0 + ddot(theta) t))
$

在自然坐标系下，对底盘做受力分析。

由于底盘做匀加速运动，坐标系非惯性系，由达朗贝尔原理，可将加速度设为假想的惯性力。

在 $t = 0$ 时刻，底盘受平移力：
$ vb(F) = F_x + iu F_y = - m ddot(x) - iu m ddot(y) $

底盘受旋转力矩：$ M = - J ddot(theta) $

在该坐标系下，底盘受假想的惯性力但保持静止。每个轮可提供平行于轮方向的主动力和垂直于轮方向的静摩擦力，约束底盘保持静止。

轮无论位于何方向，其主动力和静摩擦力总是互相垂直，若假定无论如何都不会发生滑动，可以将4个轮当成光滑铰链约束，提供任意方向和大小的约束反力阻止底盘移动，在解出约束反力后再分解为轮的主动力和静摩擦力。
铰链约束仅提供反力，不提供反力矩。设每个铰链约束的约束反力向量为 $F_(x i) + iu F_(y i)$，可以列出方程：
$
  cases(
    vb(F) + sum_(i = 1)^4 lr((F_(x i) + iu F_(y i))) = 0 \
    M + sum_(i = 1)^4 lr((- R F_(x i) sin lr((phi_i)) + R F_(y i) cos lr((phi_i)))) = 0
  )
$

展开方程得到：
$
  cases(
    F_(x 1) + F_(x 2) + F_(x 3) + F_(x 4) + F_x = 0 \
    F_(y 1) + F_(y 2) + F_(y 3) + F_(y 4) + F_y = 0 \
    R lr((F_(y 1) - F_(x 2) - F_(y 3) + F_(x 4))) + M = 0
  )
$

共3个实数方程，但存在8个未知量。显然该系统是超静定的，若不增加条件无法求解。
考虑引入最小范数条件，即最小化反力总平方和：
$ min sum_(i = 1)^4 lr((F_(x i)^2 + F_(y i)^2)) $

构建拉格朗日函数，引入乘数 $lambda_1 comma lambda_2 comma lambda_3$：
$
  L & = sum lr((F_(x i)^2 + F_(y i)^2)) + lambda_1 lr((sum F_(x i) + F_x))                              \
    & + lambda_2 lr((sum F_(y i) + F_y)) + lambda_3 lr((F_(y 1) - F_(x 2) - F_(y 3) + F_(x 4) + M / R))
$

对每个变量求偏导并令其为零，解得：
$
  F_(x 1) & = - lambda_1 / 2 comma space & F_(x 2) & = frac(- lambda_1 + lambda_3, 2) comma space & F_(x 3) & = - lambda_1 / 2 comma space & F_(x 4) & = frac(- lambda_1 - lambda_3, 2) comma space\
  F_(y 1) & = frac(- lambda_2 - lambda_3, 2) comma space & F_(y 2) & = - lambda_2 / 2 comma space & F_(y 3) & = frac(- lambda_2 + lambda_3, 2) comma space & F_(y 4) & = - lambda_2 / 2 dot.basic
$

将上述表达式代入平衡方程，解得：
$ lambda_1 = F_x / 2 comma quad lambda_2 = F_y / 2 comma quad lambda_3 = frac(M, 2 R) dot.basic $

代回各反力表达式，得到：
$
  mat(vb(F)_1; vb(F)_2; vb(F)_3; vb(F)_4)
  = mat(F_(x 1), F_(y 1); F_(x 2), F_(y 2); F_(x 3), F_(y 3); F_(x 4), F_(y 4))
  = 1 / 4 mat(m ddot(x), m ddot(y) + frac(J ddot(theta), R); m ddot(x) - frac(J ddot(theta), R), m ddot(y); m ddot(x), m ddot(y) - frac(J ddot(theta), R); m ddot(x) + frac(J ddot(theta), R), m ddot(y))
$

写为复数形式：
$ vb(F)_i = frac(m ddot(x) + iu m ddot(y) + iu frac(J ddot(theta), R) e^(iu phi_i), 4) $

设舵轮 $i$ 的正方向相对底盘的夹角（舵向角）的观测值为 $zeta_(i 0)$。

将 $vb(F)_i$ 投影到 $zeta_(i 0)$ 方向，乘以轮半径 $r$，得到轮的目标控制力矩：

$
  limits(tau_i)_"feedforward" = r lr((vb(F)_i dot.op e^(iu zeta_(i 0)))) = r lr((frac(m ddot(x) R cos lr((zeta_(i 0))) + m ddot(y) R sin lr((zeta_(i 0))) + J ddot(theta) sin lr((zeta_(i 0) - phi_i)), 4 R)))
$

在计算目标控制力矩时引入了较多假设（如最小范数条件），在遇到扰动（如打滑）时抗干扰能力不佳。考虑引入速度环PID控制器，将底盘观测速度 $lr((dot(x)_0 comma dot(y)_0 comma dot(theta)_0))$ 下理论计算的轮速度，在轮正方向上的投影作为控制速度，原计算得到的控制力矩作为PID控制器的前馈。
$
  tau_i
  &= limits(tau_i)_"feedforward" + "PID"_"velocity" lr((frac(dot(vb(r))_i (0) dot.op e^(iu zeta_(i 0)), r) - omega_i)) \
  &= r lr((frac(m ddot(x) R cos lr((zeta_(i 0))) + m ddot(y) R sin lr((zeta_(i 0))) + J ddot(theta) sin lr((zeta_(i 0) - phi_i)), 4 R))) + \
  & quad "PID"_"velocity" lr((frac(dot(x)_0 cos lr((zeta_(i 0))) + dot(y)_0 sin lr((zeta_(i 0))) + R dot(theta)_0 sin lr((zeta_(i 0) - phi_i)), r) - omega_i))
$


==== 舵电机

设舵轮 $i$ 的正方向相对底盘的夹角（舵向角）的目标控制值为 $zeta_i$，则#footnote[ $zeta_i (t)$ 和 $dot(zeta)_i (t)$ 完整表达式见附录（TODO）。]：
$ zeta_i (t) = arg lr((dot(vb(r))_i)) - theta $
$ dot(zeta)_i (t) = (diff zeta_i) / (diff t) $

$ddot(zeta)_i (t)$ 的表达式过于复杂，由于轮为轻质轮，简单起见可令 $zeta_ddot(i) = 0$，即直接由PID控制器闭环舵向角速度，不使用舵向角加速度作为前馈。

$t = 0$ 时刻：
$
  zeta_i (0) = tan^(- 1) lr((dot(x)_0 - R dot(theta)_0 sin lr((phi_i)) comma dot(y)_0 + R dot(theta)_0 cos lr((phi_i))))
$

$
  dot(zeta)_i (0)
  = (dot(x)_0 ddot(y) - dot(y)_0 ddot(x) - dot(theta)_0 (dot(x)_0^2 + dot(y)_0^2) \
  + R cos(phi_i)(ddot(theta) dot(x)_0 - dot(theta)_0 (ddot(x) + dot(theta)_0 dot(y)_0))
  + R sin(phi_i)(ddot(theta) dot(y)_0 - dot(theta)_0 (ddot(y) + dot(theta)_0 dot(x)_0)))
  / ((dot(x)_0 - R dot(theta)_0 sin (phi_i))^2
  + (dot(y)_0 - R dot(theta)_0 sin (phi_i))^2)
$

注意到上述表达式在轮 $i$ 处线速度 $dot(vb("r"))_i = 0$，即
$display(
  cases(
    dot(x)_0 - R dot(theta)_0 sin lr((phi_i)) = 0 \
    dot(y)_0 + R dot(theta)_0 cos lr((phi_i)) = 0
  )
)$
时无法求值。

对 $zeta_i (0)$，显然极限
$lim_(
dot(x)_0 -> R dot(theta)_0 sin lr((phi_i)) \
dot(y)_0 -> -R dot(theta)_0 cos lr((phi_i))
) zeta_i (0)$
不定，考虑将奇异点代入 $zeta_i (t)$：

$
  zeta_i (t)
  stretch(->, size: #(100% + 2em))^(
  dot(x)_0 & arrow.r.bar R dot(theta)_0 sin lr((phi_i)) \
  dot(y)_0 & arrow.r.bar - R dot(theta)_0 cos lr((phi_i))
             ) zeta_i prime (t)
$

对 $t$ 求极限得：
$
  limits(zeta_i (0))_(dot(vb(r))_i = 0) & = lim_(t arrow.r 0^+) zeta_i prime (t)\
  & = tan^(- 1) lr((ddot(x) - R lr((ddot(theta) sin lr((phi_i)) + dot(theta)_0^2 cos lr((phi_i)))) comma ddot(y) + R lr((ddot(theta) cos lr((phi_i)) - dot(theta)_0^2 sin lr((phi_i))))))
$

对 $dot(zeta)_i (0)$，显然
$lim_(
dot(x)_0 -> R dot(theta)_0 sin lr((phi_i)) \
dot(y)_0 -> -R dot(theta)_0 cos lr((phi_i))
) dot(zeta)_i (0)$
同样不定，为简单起见，定义：
$ limits(dot(zeta)_i)_(dot(vb(r))_i = 0) = 0 $

表达式 $limits(zeta_i (0))_(dot(vb(r))_i = 0)$ 在在轮 $i$ 处加速度 $ddot(vb(r))_i = 0$ 时依旧无解，为简单起见，定义此时刻舵向控制力为 $0$。

合并上述表达式，得到舵向目标控制力矩：
$
  tau_(zeta i) = cases(
    "PID"_"velocity" (
      dot(zeta)_i (0) + "PID"_"angle" (zeta_i (0) - zeta_(i 0)) - dot(zeta)_(i 0)) comma quad & dot(vb("r"))_i eq.not 0,
    "PID"_"velocity" (
      "PID"_"angle" (limits(zeta_i (0))_(dot(vb(r))_i = 0) - zeta_(i 0)) - dot(zeta)_(i 0)) comma quad & dot(vb("r"))_i = 0,
    0 comma quad & ddot(vb(r))_i = 0
  )
$

=== 增加约束

在@head:底盘控制器.闭环底盘速度 中，由PID控制器计算了底盘的目标控制加速度，但显然底盘功率并非无限，底盘在加速度过大时也会存在打滑。因此实际控制加速度不可能总与期望相符，考虑引入约束条件以保证加速度合法，通过极大化目标函数 $z lr((ddot(r) comma ddot(theta))) = a ddot(r) + b ddot(theta)$ 以确保获得最优解。

@head:底盘控制器.闭环底盘速度 计算的未约束的目标控制加速度，在本小节中用 $lr((ddot(x)_(upright("max")) comma ddot(y)_(upright("max")) comma ddot(theta)_(upright("max"))))$ 表示。

设 $alpha = tan^(- 1) lr((ddot(x)_(upright("max")) comma ddot(y)_(upright("max"))))$ 是底盘目标平移控制加速度的方向角，有：
$ ddot(x) = ddot(r) cos lr((alpha)) comma thin thin ddot(y) = ddot(r) sin lr((alpha)) $
<numbering>

设 $ddot(r)_(upright("max")) = sqrt(ddot(x)_(upright("max"))^2 + ddot(y)_(upright("max"))^2)$，有：
$
  ddot(x)_(upright("max")) = ddot(r)_(upright("max")) cos lr((alpha)) comma thin thin ddot(y)_(upright("max")) = ddot(r)_(upright("max")) sin lr((alpha))
$

在一次解算内，$alpha$ 不变，认为其是已知量。

==== 控制加速度约束

对于约束后的平移控制加速度 $ddot(r)$，它应当小于等于未约束的平移控制加速度 $ddot(r)_(upright("max"))$。

对于约束后的旋转控制加速度 $ddot(theta)$，它应当小于等于未约束的旋转控制加速度 $ddot(theta)_(upright("max"))$ #footnote[ $ddot(theta)_(upright("max"))$ 可能为负，本文只考虑其非负的情况。在实际的算法实现中，若遇到负值，将相关值取相反数处理，完成求解后，将结果反向映射回控制量。]。

于是：
$
  cases(
    ddot(r) lt.eq ddot(r)_(upright("max")) \
    ddot(theta) lt.eq ddot(theta)_(upright("max"))
  )
$

问题转化为二维线性规划 $lr((upright("LP")))$ 问题，决策变量为
$ddot(r) comma ddot(theta)$。

==== 打滑约束

要求轮不发生滑动，即满足 $lr(|vb("F")_i|) lt.eq mu F_N$。

假定车重均匀分布于每个轮，则约束不等式：
$ lr(|vb("F")_i|) lt.eq frac(mu m g, 4) comma quad i = 1 comma 2 comma 3 comma 4 $

整理得：
$
  lr((m ddot(r) R cos lr((alpha)) - J ddot(theta) sin lr((phi_i))))^2 + lr((m ddot(r) R sin lr((alpha)) + J ddot(theta) cos lr((phi_i))))^2 lt.eq mu^2 m^2 g^2 R^2 comma quad i = 1 comma 2 comma 3 comma 4
$

显然表达式是一个二次不等式组，由于其不满足线性规划问题的定义，问题转换为非线性规划 $lr((upright("NLP")))$ 问题。

$upright("NLP")$ 问题是难以求解的，考虑利用表达式的良好性质简化问题。

代入多组典型参数，绘制不等式组的可行域，发现可行域恒为对 $x,y$ 轴对称的菱形，且形状仅与 $alpha$ 相关，且在 $alpha = 1 / 2 k pi thin lr((k in bb(Z)))$ 时，可行域为菱形；在 $alpha = 1 / 2 k pi + 1 / 4 pi thin lr((k in bb(Z)))$ 时，可行域为面积最大的豆腐形；在 $alpha$ 为其余值时，可行域介于菱形与面积最大的豆腐形之间。

#figure([], caption: [
  可行域的形状仅与 $alpha$ 相关
])

由于不等式可行域即使在面积最大时，形状仍与菱形接近，考虑尽量优化计算速度，认为可行域恒为菱形。
于是问题回到线性规划 $lr((upright("LP")))$ 问题。

菱形的右顶点 $A lr((mu g comma 0))$，上顶点 $B lr((0 comma frac(m R, J) mu g))$，写为线性不等式形式：
$
  dcases(
    frac(1, mu g) lr((x + frac(J y, m R))) lt.eq 1 \
    frac(1, mu g) lr((x - frac(J y, m R))) lt.eq 1 \
    frac(1, mu g) lr((- x + frac(J y, m R))) lt.eq 1 \
    frac(1, mu g) lr((- x - frac(J y, m R))) lt.eq 1
  )
$

==== 功率约束 <head:底盘控制器.增加约束.功率约束>

显然底盘功率存在上限，需要加以限制。

单个电机预期消耗功率的经验公式：
#num_eq(
  $ P_i lr((tau_i)) = k_1 tau_i^2 + omega_i tau_i + k_2 omega_i^2 $,
) <eq:单个电机预期消耗功率的经验公式>


其中，$k_1$（电流热损耗系数）与 $k_2$（机械损耗系数）为常数，由实验拟合得出。

@head:底盘控制器.计算电机控制力矩.轮电机 计算了轮电机的控制力矩：
$ tau_i = limits(tau_i)_"feedforward" + limits(tau_i)_"PID" $

其中：
$
  limits(tau_i)_"feedforward" & = r (frac(m ddot(r) cos lr((zeta_(i 0) - alpha)), 4) + frac(J ddot(theta) sin lr((zeta_(i 0) - phi_i)), 4 R)) \
  limits(tau_i)_"PID" & = "PID"_"velocity" (frac(dot(x)_0 cos lr((zeta_(i 0))) + dot(y)_0 sin lr((zeta_(i 0))) + R dot(theta)_0 sin lr((zeta_(i 0) - phi_i)), r) - omega_i)
$

注意到 $limits(tau_i)_"PID"$ 在一次解算中恒为常数，可
将 $tau_i$ 整理为关于 $ddot(r) comma ddot(theta)$ 的多项式：
$
  tau_i = frac(m r cos lr((zeta_(i 0) - alpha)), 4) ddot(r) + frac(J r sin lr((zeta_(i 0) - phi_i)), 4 R) ddot(theta) + limits(tau_i)_"PID"
$

代入@eq:单个电机预期消耗功率的经验公式，得到：
$
  P_"total" = sum_(i = 1)^4 P_i lr((tau_i)) = A ddot(r)^2 + B ddot(r) ddot(theta) + C ddot(theta)^2 + D ddot(r) + E ddot(theta) + F lt.eq P_(upright("max"))
$

其中：
$
  A & = sum_(i = 1)^4 1 / 16 k_1 m^2 r^2 cos^2 lr((zeta_(i 0) - alpha))                                        \
  B & = sum_(i = 1)^4 1 / 8 k_1 m J r^2 R^(- 1) cos lr((zeta_(i 0) - alpha)) sin lr((zeta_(i 0) - phi_i))      \
  C & = sum_(i = 1)^4 1 / 16 k_1 J^2 r^2 R^(- 2) sin^2 lr((zeta_(i 0) - phi_i))                                \
  D & = sum_(i = 1)^4 1 / 4 m r cos lr((zeta_(i 0) - alpha)) lr((2 k_1 limits(tau_i)_"PID" + omega_i))         \
  E & = sum_(i = 1)^4 1 / 4 J r R^(- 1) sin lr((zeta_(i 0) - phi_i)) lr((2 k_1 limits(tau_i)_"PID" + omega_i)) \
  F & = sum_(i = 1)^4 k_1 limits(tau_i)_"PID" + omega_i limits(tau_i)_"PID" + k_2 omega_i^2
$

显然不等式同样是一个二次不等式，对应的约束为二次约束。由Cauchy-Schwarz不等式（柯西不等式）：
$
  lr((sum_(i = 1)^4 cos lr((zeta_(i 0) - alpha)) sin lr((zeta_(i 0) - phi_i))))^2 lt.eq lr((sum_(i = 1)^4 cos^2 lr((zeta_(i 0) - alpha)))) lr((sum_(i = 1)^4 sin^2 lr((zeta_(i 0) - phi_i))))
$

由 $k_1 comma m comma J comma r comma R > 0$，得
$display(frac(d^2 m^2 J^2, 64 R^2) > 0)$ ，两边同乘该因子，整理得：
$ B^2 lt.eq 4 A C $

可知该二次约束对应的 $Q$ 矩阵是半正定的，$P_"total"$ 是一个凸函数，相应的二次约束为凸二次约束。

=== 求解问题

问题的数学形式为：
$
    "maximize" quad & f lr((tau_t comma tau_r)) = a tau_t + b tau_r           \
  "subject to" quad & ddot(r) lt.eq ddot(r)_"max"                             \
                    & ddot(theta) lt.eq ddot(theta)_"max"                     \
                    & frac(1, mu g) lr((x + frac(J y, m R))) lt.eq 1          \
                    & frac(1, mu g) lr((x - frac(J y, m R))) lt.eq 1          \
                    & frac(1, mu g) lr((- x + frac(J y, m R))) lt.eq 1        \
                    & frac(1, mu g) lr((- x - frac(J y, m R))) lt.eq 1        \
                    & P_"total" = sum_(i = 1)^4 P_i lr((tau_i)) lt.eq P_"max"
$

除最后一项约束（功率约束）为凸二次约束外，目标函数和其余约束均为线性，线性规划的可行域一定是凸集。
因此，该问题是一个仅有一个凸二次约束的凸二次约束规划 $("QCP")$ 问题。

==== 采用商业通用求解器COPT求解

$("QCP")$ 问题通常使用内点法求解，可以选择开源或商用的通用求解器来求解 $("QCP")$ 问题，但市面上大部分求解器主要针对二次规划 $("QP")$ 问题求解，可以求解 $("QCP")$ 问题的通用求解器较少。

尝试采用国产商业通用求解器杉树 $("COPT")$ 对问题进行求解：

```
Cardinal Optimizer v7.2.7. Build date Apr 11 2025
Copyright Cardinal Operations 2025. All Rights Reserved

Setting parameter 'TimeLimit' to 0.1
Setting parameter 'Threads' to 1
Model fingerprint: 27e74570

Using Cardinal Optimizer v7.2.7 on Linux
Hardware has 11 cores and 22 threads. Using instruction set X86_AVX2
Maximizing a QCP problem

The original problem has:
    4 rows, 2 columns and 8 non-zero elements
    1 quadratic constraints
The presolved problem has:
    6 rows, 5 columns and 15 non-zero elements
    1 cones

Starting barrier solver using 1 thread

Problem info:
Range of matrix coefficients:    [1e-01,1e+00]
Range of rhs coefficients:       [5e-01,5e-01]
Range of bound coefficients:     [1e+00,1e+00]
Range of cost coefficients:      [4e-01,1e+00]

Factor info:
Number of dense columns:         0
Number of matrix entries:        2.100e+01
Number of factor entries:        2.100e+01
Number of factor flops:          7.200e+01

Iter       Primal.Obj         Dual.Obj      Compl  Primal.Inf  Dual.Inf
   0  +0.00000000e+00  -4.00000000e+00   9.00e+00    1.00e+00  1.41e+00
   1  -3.63528555e-01  -1.10418946e+00   1.53e+00    1.91e-01  2.70e-01
   2  -3.94420420e-01  -5.63866533e-01   3.35e-01    4.64e-02  6.56e-02
   3  -4.23270927e-01  -4.91048482e-01   1.25e-01    1.89e-02  2.67e-02
   4  -3.96195440e-01  -4.04062794e-01   1.33e-02    2.05e-03  2.90e-03
   5  -3.99948595e-01  -4.00449746e-01   8.47e-04    1.33e-04  1.88e-04
   6  -4.00218257e-01  -4.00225109e-01   1.15e-05    1.80e-06  2.55e-06
   7  -4.00219033e-01  -4.00220047e-01   1.70e-06    2.67e-07  3.78e-07
   8  -4.00218793e-01  -4.00218849e-01   9.49e-08    1.59e-08  1.49e-08

Barrier status:                  OPTIMAL
Primal objective:                -4.00218793e-01
Dual objective:                  -4.00218849e-01
Duality gap (abs/rel):           5.60e-08 / 5.60e-08
Primal infeasibility (abs/rel):  1.59e-08 / 1.59e-08
Dual infeasibility (abs/rel):    1.49e-08 / 1.49e-08

Postsolving

Solving finished
Status: Optimal  Objective: 4.0021879328e-01  Iterations: 8
Solution: (-0.006806, 2.035123)
Elapsed time: 0.00296087s
```

查看日志可知，求解器在 $9$ 次迭代后得出了最优解 $("Optimal")$，经验证该解是正确的。
但求解器单次计算耗时约 $3 thin "ms"$，大于许用的计算时间 $1 thin "ms"$，控制的实时性无法得到有效保证。

==== 编写求解器求解析解

由于问题是一个仅有一个凸二次约束的凸二次约束规划 $("QCP")$ 问题，且维数仅为 $2$ 维，考虑配合图解法，手动编写求解器求问题的解析解。

对于凸规划问题，局部最优解是它的全局最优解。
因此，要求问题的全局最优解，只需沿着其边缘搜寻局部最优解。

===== 求线性可行域按逆时针排序的角点序列

设 $dcases(
  x & = ddot(r)\
  y & = ddot(theta)
)$，将问题转换到二维笛卡尔坐标系下。

打滑约束的可行域是沿 $x, y$
轴对称的菱形。从其右顶点开始，逆时针记录其顶点序列为多边形
$P = { P_1, P_2, P_3, P_4 }$。

使用 Sutherland–Hodgman 算法，遍历剩余约束条件，对多边形进行裁剪。算法一次裁剪的伪代码如下：

```
for (int i = 0; i < input_points.count; i += 1) do
    Point current_point = input_points[i];
    Point prev_point = input_points[(i - 1) % input_points.count];

    Point Intersecting_point = ComputeIntersection(prev_point, current_point, clip_edge)

    if (current_point inside clip_edge) then
        if (prev_point not inside clip_edge) then
            output_points.add(Intersecting_point);
        end if
        output_points.add(current_point);

    else if (prev_point inside clip_edge) then
        output_points.add(Intersecting_point);
    end if
done
```

- 第一次裁剪：

  $P$ 作为初始序列 `input_points`，条件 $ddot(r) lt.eq ddot(r)_"max"$
  作为裁剪边 `clip_edge` 输入，执行算法。

- 第二次裁剪：

  将上一次裁剪的输出 `output_points` 作为初始序列 `input_points`，条件
  $ddot(theta) lt.eq ddot(theta)_"max"$ 作为裁剪边 `clip_edge`
  输入，执行算法，完成第二次裁剪。

===== 求凸二次约束的最优解

对于凸规划问题，局部最优解是它的全局最优解。因此，作为一个显著有效的优化，若凸二次约束的最优解在其余线性约束的可行域内，则可直接得出全局最优解为求凸二次约束的最优解。

由于二次约束是凸的，最优解一定在可行域边缘，可以把不等式转换为等式，使用拉格朗日乘数法求解最优解。

将凸二次约束写为矩阵形式：
$ 1 / 2 vb(x)^T Q vb(x) + vb(p)^T vb(x) + r = 0 $

其中 #footnote[若 $A < 0$，按@eq:矩阵形式凸二次约束的各数值量 赋值会导致 $Q$ 为负定矩阵，此时需对 $A,B,C,D,E,F$ 取相反数处理。]：
#num_eq[
  $
    vb(x) & = mat(delim: "[", ddot(r); ddot(theta)) comma \
        Q & = mat(delim: "[", 2 A, B; B, 2 C) comma       \
    vb(p) & = mat(delim: "[", D; E) comma                 \
        r & = F - P_(upright("max")) dot.basic
  $
]<eq:矩阵形式凸二次约束的各数值量>

需要极大化的目标函数：$ z lr((vb(x))) = vb(a) dot.op vb(x) $

其中：$ vb(a) = mat(delim: "[", a; b) $

写出拉格朗日函数：
$
  L lr((vb(x) comma lambda)) = vb(a)^T vb(x) - lambda lr((1 / 2 vb(x)^T Q vb(x) + vb(p)^T vb(x) + r))
$

求梯度并令其为 $0$：
$
  nabla_(vb(x)) L = vb(a) - lambda lr((Q vb(x) + vb(p))) = 0 arrow.r.double.long vb(a) = lambda lr((Q vb(x) + vb(p)))
$

求解线性方程（假设 $Q^(- 1)$ 存在）：
#num_eq($ vb(x) = Q^(- 1) lr((1 / lambda vb(a) - vb(p))) $) <eq:令梯度为0求解线性方程>

代入约束条件并化简，得到 #footnote[实际上 $lambda$ 存在正负两解，对应使目标函数最大和最小时的取值。但由于 $Q$ 为半正定矩阵，正解使目标函数取得最大值，故取正解。]：
$ lambda = sqrt(frac(vb(a)^T Q^(- 1) vb(a), vb(p)^T Q^(- 1) vb(p) - 2 r)) $

计算 $lambda$ 并代入@eq:令梯度为0求解线性方程，得到凸二次约束的最优解。若最优解在角点序列多边形内，则直接返回最优解。

在求解过程中我们假定 $Q^(- 1)$ 存在，但@head:底盘控制器.增加约束.功率约束 仅能证明 $Q$ 是半正定矩阵。当 $Q$ 不正定时，约束曲线将由椭圆退化为抛物线或分段线性结构（如两条线段），导致优化问题欠定，$Q^(- 1)$ 不存在。此时在退化方向（如抛物线开口方向）上，最优解可能趋向无穷大。

考虑引入 Moore-Penrose 伪逆 $Q^+$ 替代 $Q^(- 1)$。然而当 $Q$
奇异时，伪逆会强制给出最小范数解：一方面，该解滤除了描述系统退化特性的零空间分量；另一方面，在有解方向上产生与实际物理意义不符的数值结果。

因此，本文采用正则化方法
$lr((Q + epsilon I))^(- 1)$（$epsilon arrow.r 0^+$），当 $Q$ 奇异时，在对角线上引入微小扰动，使得矩阵可逆。该对角线扰动等效于在优化问题中引入
Tikhonov 正则项：
$ min_(vb(x)) lr(parallel vb(x) parallel)^2 quad "s.t. original constraints" $

在退化方向上，扰动诱导的特征值偏移（$lambda_i -> lambda_i + epsilon$）使解的范数发散：
$ vb(x)_"reg" approx 1 / epsilon vb(v)_"null" + O(1) $

这种可控的发散特性正确符合了求解要求，且当 $epsilon arrow.r 0^+$
时收敛于理论极限解。

===== 用二次约束裁剪多边形

来到这一步说明切点不在多边形内，因此最优解一定在多边形与二次约束的交点，或多边形的顶点上。

理解这一步很重要，意味着终于可以不把二次曲线当曲线考虑了，只需考虑交点和顶点。

遍历多边形的每条边，设其两端点分别为 $vb(x)_1 comma vb(x)_2$。

将 $vb(x)_1 comma vb(x)_2$ 依次代入二次约束方程，可判断点是否在二次约束内。
这里分情况讨论：

- 两端点均在二次约束内（含边缘）

  此时线段的两端点一定是最终可行域顶点的一部分。

- 其它情况

  此时交点数量未知，需要先求出线段与二次约束的交点。

  设 $vb(d)_vb(x) = vb(x)_2 - vb(x)_1$，有线段参数方程：
  $ vb(x) (t) = vb(x)_1 + t vb(d)_vb(x), quad t in lr([0, 1]) $

  将参数方程代入约束方程：
  $
    1 / 2 [ vb(x)_1 + t vb(d)_vb(x) ]^top Q [ vb(x)_1 + t vb(d)_vb(x) ] + vb(p)^top [ vb(x)_1 + t vb(d)_vb(x) ] + r = 0
  $

  展开并分离系数，得到标准二次形式：
  $
    underbrace(1 / 2 vb(d)_vb(x)^top Q vb(d)_vb(x), a) t^2
    + underbrace(vb(x)_1^top Q vb(d)_vb(x) + vb(p)^top vb(d)_vb(x), b) t
    + underbrace(1 / 2 vb(x)_1^top Q vb(x)_1 + vb(p)^top vb(x)_1 + r, c)
    = 0
  $

  解得 $t = frac(- b plus.minus sqrt(b^2 - 4 a c), 2 a) thin lr((t in lr([0 comma 1])))$，代入参数方程即可得到交点的坐标。

  交点和在二次约束内（含边缘）的端点一定是最终可行域顶点的一部分。

最后，将所有的最终可行域顶点整理为新的多边形。
最优解一定是这个多边形的顶点之一，简单遍历即可求解。

===== 求解效果

代码编写完成后，使用同样的输入，与杉树求解器的效果进行对比：

```
COPT solution: (-0.006806, 2.035123)
Elapsed Time: 0.002374s
Our solution: (-0.006803, 2.035111)
Elapsed Time: 0.000002s
```

查看日志可知，手动编写的解析解求解器，不仅精度高于通用求解器的迭代解，且求解速度提升约 $10^3$ 倍。
