#import "/template/template.typ": *

== 全向底盘控制器

约定复数 $a + b iu$（$iu$ 为虚数单位）表示向量 $lr((a comma b))$。

底盘控制器接收底盘的目标水平控制速度 $dot(vb(r)) = dot(x) + iu dot(y)$ 和目标旋转控制速度 $dot(theta)$，并输出轮电机的目标控制力矩。

底盘的控制频率为 $1 thin "kHz"$，意味着每帧计算时间 $lt.eq 1 thin "ms"$。
因此底盘控制器需要尽可能降低运算时间，提高实时性。

=== 麦轮底盘建模

麦轮底盘等效于长方形排布的全向底盘。

在水平面上，以麦轮底盘的中心为原点，从原点出发，平行于任意两相邻轮的连线，构造 $x$ 轴，建立平面直角坐标系，为自然坐标系。

从 $x$ 轴开始，逆时针依次遍历位于第一、第二、第三、第四象限处的轮，分别记其编号 $i$ 为 $1 comma 2 comma 3 comma 4$
。

轮电机输出轴转速的正方向，以拇指指向与原点向外的射线同向的右手四指方向决定。

对于任意轮电机 $i$，设轮半径为 $r$，观测其输出轴转速为 $omega_i$。若不考虑打滑，底盘在此处相对地面速度的可观测分量为 $v_i = minus omega_i r$（垂直于全向轮的速度分量不可观测）。

对于任意轮 $i$，不难计算 $v_i$ 的正方向：
$
  vb(p)_i = cases(
    - & 1 + iu comma quad & i = 1 \
    - & 1 - iu comma quad & i = 2 \
      & 1 - iu comma quad & i = 3 \
      & 1 + iu comma quad & i = 4
  )
$

设 $t=0$ 时刻，轮中心位置向量为 $(a, b > 0)$：
$
  vb(c)_i = cases(
      & a + b iu comma quad & i = 1 \
    - & a + b iu comma quad & i = 2 \
    - & a - b iu comma quad & i = 3 \
      & a - b iu comma quad & i = 4
  )
$

=== 全向底盘建模

全向底盘（通常为正方形排布）可认为是麦轮底盘长短边相等，即 $a = b$ 时的特例。

=== 观测底盘速度

每帧解算中，均认为当前时刻为 $t = 0$ 时刻。

以 $t = 0$ 时刻的自然坐标系为基准，建立世界坐标系。

对于底盘，其在世界坐标系下有三个自由度 $x comma y comma theta$（中心位置、旋转角），设 $vb(r) = x + iu y$。

特别地，记 $t = 0$ 时刻三自由度的值分别为 $(x_0 comma y_0 comma theta_0)$，显然此时刻 $(x_0 comma y_0 comma theta_0) = 0$。

对于轮 $i$，在任意 $t$ 时刻，其中心位置向量为：
$vb(r)_i (t) = x + iu y + vb(c)_i e^(iu theta)$

求导得任意 $t$ 时刻，轮 $i$ 的中心速度向量：
$dot(vb(r))_i (t) = dot(x) + iu dot(y) + iu dot(theta) vb(c)_i e^(iu theta)$

于是 $t=0$ 时刻：
$ dot(vb(r))_i (0) = dot(x)_0 + iu dot(y)_0 + iu dot(theta)_0 vb(c)_i $

将 $dot(vb(r))_i (0)$ 投影到 $v_i$ 的正方向，得到底盘在轮 $i$ 处的可观测速度分量：
$
  limits(dot(r)_i (0))_"observable" = dot(vb(r))_i (0) dot vb(p)_i/sqrt(2)
$

展开为实数形式：
$
  limits(dot(r)_i (0))_"observable" = 1 / sqrt(2) cases(
    - & dot(x)_0 + dot(y)_0 + (a+b)dot(theta)_0 comma quad & i = 1 \
    - & dot(x)_0 - dot(y)_0 + (a+b)dot(theta)_0 comma quad & i = 2 \
      & dot(x)_0 - dot(y)_0 + (a+b)dot(theta)_0 comma quad & i = 3 \
      & dot(x)_0 + dot(y)_0 + (a+b)dot(theta)_0 comma quad & i = 4
  )
$

构造观测方程组：

$ limits(dot(r)_i (0))_"observable" = v_i, quad i = 1, 2, 3, 4 $

共4个方程，和3个未知数，显然这是一个超定方程组，将其化为矩阵形式 $A vb(x) = vb(b)$：
$
  A = 1 / sqrt(2) mat(
    - & 1, & 1, a+b;
    - & 1, - & 1, a+b;
    & 1, - & 1, a+b;
    & 1, & 1, a+b;
  ), quad
  vb(x) = mat(
    dot(x)_0;
    dot(y)_0;
    dot(theta)_0;
  ), quad
  vb(b) = mat(
    v_1;
    v_2;
    v_3;
    v_4
  )
$

构造正规方程 $A^top A vb(x) = A^top vb(b)$，解之，得到最小二乘近似解：
$
  mat(
    dot(x)_0;
    dot(y)_0;
    dot(theta)_0;
  )
  = sqrt(2) / 4 mat(
    - & v_1 - & v_2 + & v_3 + v_4;
    & v_1 - & v_2 - & v_3 + v_4;
    1 / (a + b) ( & v_1 + & v_2 + & v_3 + v_4)
  )
$

用轮的输出轴转速 $omega_i$ 表示：
$
  mat(
    dot(x)_0;
    dot(y)_0;
    dot(theta)_0;
  )
  = - (sqrt(2) r) / 4 mat(
    - & omega_1 - & omega_2 + & omega_3 + omega_4;
    & omega_1 - & omega_2 - & omega_3 + omega_4;
    1 / (a + b) ( & omega_1 + & omega_2 + & omega_3 + omega_4)
  )
$

即得到底盘的观测速度 $(dot(x)_0 comma dot(y)_0 comma dot(theta)_0)$。

=== 简单闭环控制

假定底盘功率无限，电机能提供的扭矩无限，我们可以简单地对观测速度的平移和旋转分量进行闭环控制。

通过PID控制器，基于底盘的目标控制速度 $lr((dot(x) comma dot(y) comma dot(theta)))$ 与观测速度 $(dot(x)_0 comma dot(y)_0 comma dot(theta)_0)$，闭环底盘的目标控制加速度 $lr((ddot(x) comma ddot(y) comma ddot(theta)))$。

$
      ddot(x) & = "PID"_"velocity" lr((dot(x) - dot(x)_0)) comma             \
      ddot(y) & = "PID"_"velocity" lr((dot(y) - dot(y)_0)) comma             \
  ddot(theta) & = "PID"_"velocity" lr((dot(theta) - dot(theta)_0)) dot.basic \
$

=== 计算电机控制力矩

设轮 $i$ 在接地点处对底盘的目标作用力为 $F_i$。
对底盘，由牛顿第二定律：
$
       1/sqrt(2) (- F_1 - F_2 + F_3 + F_4) & = m ddot(x)     \
        1/sqrt(2) ( F_1 - F_2 - F_3 + F_4) & = m ddot(y)     \
  1/sqrt(2) sum_(i=1)^4 vb(c)_i times F_i vb(p)_i
  = (a+b)/sqrt(2) ( F_1 + F_2 + F_3 + F_4) & = J ddot(theta)
$

写为矩阵形式 $A vb(x) = vb(b)$：
$
  A = 1 / sqrt(2) mat(
    -1, -1, 1, 1;
    1, -1, -1, 1;
    a+b, a+b, a+b, a+b;
  ), quad
  vb(x) = mat(
    F_1;
    F_2;
    F_3;
    F_4;
  ), quad
  vb(b) = mat(
    m ddot(x);
    m ddot(y);
    J ddot(theta)
  )
$

共3个方程，和4个未知数，显然这是一个欠定方程组，若不增加条件无法求解。

要求最节省能量的解，显然需要使 $F_i$ 的范数和 $sum_(i=1)^4 ||F_i||$ 最小，即最小范数解。
这可通过 Moore-Penrose 伪逆求得：
$
  vb(x) = mat(
    F_1;
    F_2;
    F_3;
    F_4;
  ) = A^top (A A^top)^(-1) vb(b) = sqrt(2) / 4 mat(
    - & m ddot(x) + m ddot(y) + (J ddot(theta)) / (a + b);
    - & m ddot(x) - m ddot(y) + (J ddot(theta)) / (a + b);
    & m ddot(x) - m ddot(y) + (J ddot(theta)) / (a + b);
    & m ddot(x) + m ddot(y) + (J ddot(theta)) / (a + b);
  )
$

写为复数形式：
$
  F_i = sqrt(2) / 4 (m vb(p)_i dot ddot(vb(r)) + J / (a+b) ddot(theta))
$

于是轮的目标控制力矩：
$
  limits(tau_i)_"feedforward" = - r F_i = - (sqrt(2) r) / 4 (m vb(p)_i dot ddot(vb(r)) + J / (a+b) ddot(theta))
$

#v(0.2em)
不妨设 $display(vb(tau)_r = - (sqrt(2) m r) / 4 ddot(vb(r)))$，$display(tau_theta = - (sqrt(2) J r) / (4 (a + b)) ddot(theta))$，表达式可简化为：
#v(0.2em)
$
  limits(tau_i)_"feedforward" = vb(p)_i dot vb(tau)_r + tau_theta
$

设 $display(hat(vb(tau)_r) = frac(vb(tau)_r, ||vb(tau)_r||))$，$tau_r = ||vb(tau)_r||$，则 $vb(p)_i dot vb(tau)_r = vb(p)_i dot hat(vb(tau)_r) tau_r$。
再设 $lambda_i = vb(p)_i dot hat(vb(tau)_r)$，表达式可继续简化：
$
  limits(tau_i)_"feedforward" = lambda_i tau_r + tau_theta
$


在计算目标控制力矩时引入了最小范数假设，在遇到扰动（如打滑）时抗干扰能力不佳。
考虑引入速度环PID控制器，将底盘观测速度 $lr((dot(x)_0 comma dot(y)_0 comma dot(theta)_0))$ 下，理论计算的底盘在轮 $i$ 处的可观测速度分量作为控制速度，原计算得到的控制力矩作为PID控制器的前馈。
$
  limits(tau_i)_"PID"
  = "PID"_"velocity" (limits(omega_i)_"expected" - omega_i)
  = "PID"_"velocity" (-frac(limits(dot(r)_i (0))^"observable", r) - omega_i)
$

于是总的目标控制力矩：
$
  tau_i & = limits(tau_i)_"feedforward" + limits(tau_i)_"PID" \
        & = lambda_i tau_r + tau_theta + limits(tau_i)_"PID"
$

$tau_r$ 和 $tau_theta$ 为未约束的目标控制力矩。
在同一次解算中，$lambda_i$ 和 $limits(tau_i)_"PID"$ 恒不变，认为其为常数。

=== 增加约束

前文由PID控制器计算了底盘的目标控制加速度和目标控制力矩，但显然底盘功率并非无限，电机能提供的扭矩也并非无限，底盘在加速度过大时也会存在打滑，因此实际控制加速度不可能总与期望相符，考虑引入约束条件以保证控制力矩合法。



==== 控制力矩约束

不难发现 $tau_i$ 仅与 $tau_r, tau_theta$ 相关。可将问题转化为线性规划 $("LP")$ 问题，决策变量为 $tau_t comma tau_r$。
通过极大化目标函数 $z lr((ddot(r) comma ddot(theta))) = alpha ddot(r) + beta ddot(theta)$ 以确保获得最优解。

对于控制力矩，我们要求它小于等于上文计算的待约束的目标控制力矩，同时必须大于等于 $0$。
用 $tau_(r "max"), tau_(theta "max")$ 表示上文计算的未约束的目标控制力矩，有#footnote[ $tau_(theta "max")$ 可能为负，本文只考虑其非负的情况。在实际的算法实现中，若遇到负值，将相关值取相反数处理，完成求解后，将结果反向映射回控制量。]：
$
  cases(
    0 & <= tau_r     &     <= tau_(r "max") \
    0 & <= tau_theta & <= tau_(theta "max")
  )
$

==== 打滑约束

单个电机的控制力矩 $tau$ 总有上限。
同时为防止打滑而浪费功率，电机的输出力矩也应限制在一个较小范围。

由于电机的最大控制力矩几乎总是大于使轮发生滑动所需的力矩，因此仅考虑使轮不发生打滑，即满足 $display(|F| = lr(|tau / r|) <= mu F_N)$。

假定车重均匀分布于每个轮，得约束不等式：
$ lr(|lambda_i tau_r + tau_theta|) <= (mu m g r) / 4, quad i = 1, 2, 3, 4 $

由 $vb(p)_1 = -vb(p)_3$，得 $lambda_1 = vb(p)_1 dot hat(vb(tau)_r) = -vb(p)_3 dot hat(vb(tau)_r) = -lambda_3$，同理可证 $lambda_2 = -lambda_4$。

若设 #box(height: 1.2em)$dcases(
  x & = tau_t \
  y & = tau_r
)$，易证该不等式的可行域一定是是沿 $x comma y$ 轴对称的菱形，其上顶点恒为 #box(height: 1.5em)$display((0, (mu m g r) / 4))$，右顶点恒为 $display(((mu m g r) / (4 max(|lambda_1|, |lambda_2|)), 0))$。

==== 功率约束

显然底盘功率存在上限，需要加以限制。

单个电机预期消耗功率的经验公式：
$ P_i lr((tau_i)) = k_1 tau_i^2 + tau_i omega_i + k_2 omega_i^2 $

其中，$k_1$（电流热损耗系数）与 $k_2$（机械损耗系数）为常数，由实验拟合得出。

代入 $tau_i = lambda_i tau_r + tau_theta + limits(tau_i)_"PID"$，得到：
$
  P_"total"
  = sum_(i = 1)^4 P_i (tau_i) = A tau_r^2 + B tau_r tau_theta + C tau_theta^2 + D tau_r + E tau_theta + F
  <= P_"max"
$

其中：
$
  A & = 4 k_1                                                                         \
  B & = 0                                                                             \
  C & = 4 k_1                                                                         \
  D & = lambda_1(omega_1 - omega_3 + 2k_1(limits(tau_1)_"PID" - limits(tau_3)_"PID") )
      + lambda_2(omega_2 - omega_4 + 2k_1(limits(tau_2)_"PID" - limits(tau_4)_"PID")) \
  E & = 2 k_1 sum limits(tau_i)_"PID" + sum omega_i                                   \
  F & = k_1 sum limits(tau_i^2)_"PID"
      + sum limits(tau_i)_"PID" omega_i
      + k_2 sum omega_i^2
$

显然其圆锥曲线判别式 $B^2 <= 4 A C$，该不等式的可行域为一个圆。

由于圆不满足线性规划问题的定义，该问题转化为非线性规划问题 $("NLP")$。
$"NLP"$ 问题是难以求解的 $("NP-Hard")$，考虑利用表达式的良好性质简化问题。

首先，注意到约束表达式是一个二次不等式组，即二次约束，且除该约束外，其它约束均为线性。
又注意到 $P_"total"$ 是一个圆，显然圆为凸函数，于是相应的二次约束为凸二次约束，因此问题是一个仅有一个凸二次约束的凸二次约束规划 $("QCP")$ 问题。

=== 问题求解

问题的数学形式为：
$
    "maximize" quad & z (tau_r, tau_theta) = alpha tau_t + beta tau_r         \
  "subject to" quad & 0 <= tau_r <= tau_(r "max")                             \
                    & 0 <= tau_theta <= tau_(theta "max")                     \
                    & lr(|lambda_i tau_r + tau_theta|)
                      <= (mu m g r) / 4 space (i = 1, 2, 3, 4)                \
                    & P_"total" = sum_(i = 1)^4 P_i lr((tau_i)) lt.eq P_"max"
$

该规划问题的形式，与舵轮底盘控制器的规划问题，形式上完全相同（且更简单）。
因此两算法使用同一求解器，具体求解过程可见@head:舵轮底盘控制器.问题求解.编写求解器求解析解。
