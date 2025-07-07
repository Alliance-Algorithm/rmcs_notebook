#import "/template/template.typ": ilm

#show: ilm.with(
  title: [RMCS \ 思想、架构和指南],
  author: "Zihan Qin",
  date: datetime(year: 2025, month: 06, day: 25),
  abstract: [
    'Ilm is the Urdu term for knowledge. In its general usage, 'ilm may refer to knowledge of any specific thing or any form of "learning". Subsequently, the term came to be used to refer to various categories of "sciences", especially when used in its plural form ('ulum).
  ],
  preface: [
    #align(center + horizon)[
      Thank you for using this book #emoji.heart,\ I hope you like it #emoji.face.smile
    ]
  ],
  table-of-contents: outline(depth: 3),
  figure-index: (enabled: true),
  table-index: (enabled: true),
  listing-index: (enabled: true),
)

= 无下位机，RMCS与libRMCS
#include "chapters/introduction/introduction.typ"

= 环境配置
#include "chapters/environment/environment.typ"

= 使用libRMCS
#include "chapters/librmcs/librmcs.typ"

= RMCS架构分析
#include "chapters/rmcs/rmcs.typ"

= RMCS部分算法分析
#include "chapters/algorithm/steering_wheel.typ"

= 通讯架构分析
#include "chapters/communication/communication.typ"
