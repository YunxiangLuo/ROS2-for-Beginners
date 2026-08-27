---
marp: true
theme: default
paginate: true
---

# 第39章 全局路径规划与地图导航
## Global Path Planning & HD Map Navigation

---

# 幻灯片 1：本章大纲

1. **OpenDRIVE 地图格式** — HD Map 概念与 XML 描述
2. **全局路径规划算法** — Dijkstra vs A\*
3. **CARLA Waypoint API** — 道路网络路径生成
4. **Lanelet2 地图** — Autoware 兼容高精地图

---

# 幻灯片 2：高精度地图 (HD Map)

| 特性 | HD Map | 传统地图 |
|------|--------|---------|
| 精度 | 10-20cm | 5-10m |
| 车道级 | 每条车道独立建模 | 道路级聚合 |
| 语义 | 停止线/限速/转向 | 仅有路名 |
| 高程 | 横坡/纵坡/超高 | 无 |
| 更新 | 云端实时 | 季度更新 |

> HD Map 是自动驾驶感知与规划的"超感官"——能"看见"遮挡后的道路

---

# 幻灯片 3：OpenDRIVE 道路模型

```
 Road (道路)
 ├── PlanView (水平几何)：直线、螺旋线、回旋曲线
 ├── ElevationProfile (纵断面)
 ├── LateralProfile (横断面)
 └── Lanes (车道)
      ├── LaneSection (车道段)
      │    ├── Left (左车道, ID > 0)
      │    ├── Center (中心线, ID = 0)
      │    └── Right (右车道, ID < 0)
      └── Lane properties: width, roadMark, speed
```

**车道编号规则：** 沿道路参考线方向，左侧车道为正 (1,2,3...)，右侧车道为负 (-1,-2,-3...)

---

# 幻灯片 4：OpenDRIVE XML 结构

```xml
<road name="Road_0" length="256.78" id="0" junction="-1">
  <planView>
    <geometry s="0.0" x="12.34" y="56.78" hdg="0.523" length="256.78">
      <line/>
    </geometry>
  </planView>
  <lanes>
    <laneSection s="0.0">
      <left>
        <lane id="1" type="driving">
          <width s="0.0" a="3.5"/>
        </lane>
      </left>
    </laneSection>
  </lanes>
</road>
```

---

# 幻灯片 5：Dijkstra 算法原理

**核心思想：** 贪心 + 广度优先，逐步确定最短路径

```
初始化: dist[S]=0, dist[others]=∞
每次从未确定节点中选最小 dist → 松弛邻边

  (2)──B──(3)          dist[A]=0
  ╱       ╲           dist[B]=2  (S→B)
 S         D──(1)─T   dist[C]=5  (S→C)
  ╲       ╱           dist[D]=6  (S→B→D)
  (5)──C──(1)         dist[T]=7  (S→B→D→T)
```

- 时间复杂度: O((V+E)logV) 使用优先队列
- 缺点：无目标导向，全图扩展

---

# 幻灯片 6：A\* 算法原理

**核心思想：** f(n) = g(n) + h(n)，启发式导向搜索

```
f(n) = 实际代价 + 估计代价
        ↓          ↓
    起点→n   n→目标的估计距离
```

| 启发函数 | 公式 | 特点 |
|---------|------|------|
| 曼哈顿距离 | \|dx\| + \|dy\| | 4方向网格 |
| 欧几里得距离 | √(dx²+dy²) | 任意方向 |
| 对角线距离 | max(\|dx\|,\|dy\|) | 8方向网格 |

**可采纳条件：** h(n) ≤ 实际代价 ⇒ A\* 保证最优

---

# 幻灯片 7：Dijkstra vs A\* 搜索对比

```
  Dijkstra                      A*
  ┌──────────┐                 ┌──────────┐
  │ S  *  *  │                 │ S  *  *  │
  │ *  *  *  │                 │ *  *  G  │
  │ *  *  G  │                 │ *  G     │
  │ *  *     │                 │ G        │
  └──────────┘                 └──────────┘
  均匀向外扩散                  定向目标扩展
```

| 对比项 | Dijkstra | A\* |
|--------|----------|-----|
| 启发函数 | 无 | f=g+h |
| 扩展方向 | 各向均匀 | 目标导向 |
| 效率 | O(V²) 或 O(ElogV) | 通常快 2-10 倍 |
| 最优性 | 保证 | h 可采纳时保证 |

---

# 幻灯片 8：A\* 算法伪代码

```
1:  open_set ← {start}
2:  g[start] ← 0
3:  f[start] ← h(start, goal)
4:
5:  while open_set ≠ ∅:
6:      current ← open_set中f最小的节点
7:      if current == goal:
8:          return reconstruct_path(current)
9:
10:     for each neighbor of current:
11:         tentative_g = g[current] + dist(current, neighbor)
12:         if tentative_g < g[neighbor]:
13:             came_from[neighbor] = current
14:             g[neighbor] = tentative_g
15:             f[neighbor] = g[neighbor] + h(neighbor, goal)
16:             open_set.add_or_update(neighbor)
17:
18: return failure
```

---

# 幻灯片 9：CARLA Waypoint API

Waypoint = 车道中心线上的离散采样点

```
╔══════════════════════════════════════╗
║  ┌────┐  ┌────┐  ┌────┐  ┌────┐    ║
║  │WP0 │→│WP1 │→│WP2 │→│WP3 │→... ║
║  └────┘  └────┘  └────┘  └────┘    ║
║    ↓       ↓       ↓       ↓       ║
║ 车道中心线采样点 (间距 2m)          ║
╚══════════════════════════════════════╝
```

```python
waypoint = map.get_waypoint(location)
next_wps = waypoint.next(2.0)      # 前方 2m
left_wp  = waypoint.get_left_lane() # 左侧车道
```

---

# 幻灯片 10：路径生成策略

```python
def build_route(map, start_loc, step=2.0, max_steps=500):
    wp = map.get_waypoint(start_loc)
    route = [wp]

    for _ in range(max_steps):
        candidates = wp.next(step)
        if not candidates:
            break
        # 选择朝向变化最小的后继 waypoint
        wp = min(candidates,
            key=lambda w: abs(w.transform.rotation.yaw
                              - wp.transform.rotation.yaw))
        route.append(wp)

    return route
```

**路口处理：** 在 Junction 处根据目标方向选择对应出口车道

---

# 幻灯片 11：Lanelet2 地图结构

```
Lanelet2 核心概念:
┌──────────────────────────────────────┐
│  LaneletMap                          │
│  ├── LaneletLayer (Lanelet 集合)     │
│  │   ├── Lanelet A                   │
│  │   │   ├── leftBound (Linestring)  │
│  │   │   └── rightBound (Linestring) │
│  │   ├── Lanelet B                   │
│  │   └── ...                         │
│  ├── LinestringLayer                 │
│  └── RegulatoryElementLayer          │
└──────────────────────────────────────┘
```

每个 Lanelet 代表一个方向上车道的**最小通行单元**，包含限速、转向等语义属性。

---

# 幻灯片 12：Lanelet2 OSM 格式

```xml
<relation id="3001">
  <member type="way" ref="2001" role="left"/>
  <member type="way" ref="2002" role="right"/>
  <tag k="type" v="lanelet"/>
  <tag k="subtype" v="road"/>
  <tag k="speed_limit" v="60"/>
  <tag k="turn_direction" v="straight"/>
  <tag k="one_way" v="yes"/>
</relation>
```

**Autoware 常用工具：**
- `lanelet2_io` — 加载/保存 Lanelet2 地图
- `lanelet2_routing` — 路由规划
- `lanelet2_traffic_rules` — 交通规则

---

# 幻灯片 13：OpenDRIVE → Lanelet2 互转

```
 OpenDRIVE (.xodr)
      │
      │ lanelet2_io::load()
      ▼
 Lanelet2 Map (.osm)
      │
      │ lanelet2_io::write()
      ▼
 OpenDRIVE (.xodr) 或  Autoware OSM (.osm)
```

**转换注意事项：**
- OpenDRIVE 的 laneSection 映射为多个 Lanelet
- Junction 映射为 RegulatoryElement
- 不支持的所有 OpenDRIVE 几何类型 (spiral, paramPoly3) 可能丢失精度

---

# 幻灯片 14：本章总结

| 主题 | 核心内容 |
|------|---------|
| OpenDRIVE | XML 格式描述道路网络，CARLA 原生支持 |
| Dijkstra | 广度优先均匀扩展，保证最优 |
| A\* | 启发式导向搜索，效率更高 |
| Waypoint | 车道中心线采样点，路径生成基本单元 |
| Lanelet2 | Autoware 高精地图格式，OSM 存储 |

**课后实践：** 动手完成 Lab 39，在 CARLA 中实现 A\* 路径规划与 Waypoint 导航
