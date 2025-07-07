#import "@preview/fletcher:0.5.8" as fletcher: edge, node
#let diagram(..args) = {
  show raw: set text(
    font: "libertinus serif",
    size: 1.25em,
  )
  fletcher.diagram(..args)
}

#figure(
  diagram(
    spacing: 4em,

    // debug: 3,
    label-size: 0.8em,
    label-wrapper: edge => box(
      align(center)[#edge.label],
      inset: .3em,
      fill: edge.label-fill,
    ),

    node((0, -1), [启动脚本\ `launch-rmcs`]),
    edge(
      "d",
      "-|>",
      [
        #v(-0.5em) #text(size: 0.8em)[I.]
        加载环境变量\ `source ~/env_setup.bash`
      ],
      label-side: right,
    ),
    edge(
      "d",
      "-|>",
      [
        #v(0.5em) #text(size: 0.8em)[II.]
        `ros2 launch rmcs_bringup` \
        `rmcs.launch.py robot:=<robot_type>`
      ],
      label-side: left,
    ),

    node((0, 0), [启动描述文件\ `rmcs.launch.py`]),
    edge(
      "d",
      "-|>",
      [配置文件路径\ `<robot_name>.yaml`],
      label-side: right,
    ),

    node((0, 1), [核心进程\ `rmcs_executor/main.cpp`]),
    edge(
      "d",
      "-|>",
      [
        #v(-0.5em) #text(size: 0.8em)[I.]
        从配置文件中读取组件列表\ `vector<string>`
      ],
      label-side: right,
    ),
    edge(
      "d",
      "-|>",
      [
        #v(0.5em) #text(size: 0.8em)[II.]
        由组件名依次加载并构造组件\ `vector<shared_ptr<Component>>`
      ],
      label-side: left,
    ),

    node((0, 2), [执行器\ `class Executor`]),
    edge(
      "d",
      "-|>",
      [
        #v(-0.5em) #text(size: 0.8em)[I.]
        匹配各组件的输入/输出\ `input.pointer = &output.data;`
      ],
      label-side: right,
    ),
    edge(
      "d",
      "-|>",
      [
        #v(0.5em) #text(size: 0.8em)[II.]
        按输入/输出进行依赖分析\ 构建有向无环图得到组件更新顺序
      ],
      label-side: left,
    ),

    node((0, 3), [
      依次调用组件的update函数\
      #v(-0.25em)
      #text(size: 0.8em)[
        ```
        for (component : updating_order)
              component->update();
        ```
      ]
    ]),
    edge(
      "d",
      "-|>",
      [
        等待1ms后的下一帧\
        `sleep_until(next_iteration_time);`],
      label-side: right,
    ),


    node(
      (0, 4),
      [
        #v(-0.2em)
        `rclcpp::ok()` ?
      ],
      width: 5em,
      height: 2.3em,
      stroke: 0.5pt,
      shape: fletcher.shapes.diamond.with(fit: 0.65),
    ),
    edge("r,u,l", "-|>", [true], label-pos: 0.1),
  ),
  caption: [RMCS启动流程],
)