# Transformation Matrices - robotic_arm

Homogeneous transformation matrices between consecutive frames.
Convention: URDF RPY (XYZ extrinsic / ZYX intrinsic).

## Notation

### Frames

| Index | Link |
|-------|------|
| $L_{0}$ | base_link |
| $L_{1}$ | JAW |
| $L_{2}$ | JAW_2 |
| $L_{3}$ | JAW_3 |
| $L_{4}$ | JAW_4 |
| $L_{5}$ | JAW_5 |
| $L_{6}$ | JAW_6 |
| $L_{7}$ | JAW_7 |
| $L_{8}$ | JAW_8 |

### Joint Variables

| Variable | Joint | Type | From | To |
|----------|-------|------|------|----|
| $q_{1}$ | Revolute_1 | revolute (rad) | $L_{0}$ | $L_{1}$ |
| $q_{2}$ | Revolute_2 | continuous (rad) | $L_{1}$ | $L_{2}$ |
| $q_{3}$ | Revolute_3 | continuous (rad) | $L_{2}$ | $L_{3}$ |
| $q_{4}$ | Revolute_4 | revolute (rad) | $L_{3}$ | $L_{4}$ |
| $q_{5}$ | Revolute_5 | continuous (rad) | $L_{4}$ | $L_{5}$ |
| $q_{6}$ | Revolute_6 | revolute (rad) | $L_{5}$ | $L_{6}$ |
| $q_{7}$ | Revolute_7 | continuous (rad) | $L_{6}$ | $L_{7}$ |
| $q_{8}$ | Revolute_8 | continuous (rad) | $L_{6}$ | $L_{8}$ |

Shorthand: $c_i = \cos(q_i)$, $s_i = \sin(q_i)$

### Kinematic Tree

```
L0: base_link
  +-- [revolute] Revolute_1 (q1)
      L1: JAW
        +-- [continuous] Revolute_2 (q2)
            L2: JAW_2
              +-- [continuous] Revolute_3 (q3)
                  L3: JAW_3
                    +-- [revolute] Revolute_4 (q4)
                        L4: JAW_4
                          +-- [continuous] Revolute_5 (q5)
                              L5: JAW_5
                                +-- [revolute] Revolute_6 (q6)
                                    L6: JAW_6
                                      |-- [continuous] Revolute_7 (q7)
                                      |   L7: JAW_7
                                      +-- [continuous] Revolute_8 (q8)
                                          L8: JAW_8
```

## Transforms

## Revolute_1

$L_{0}$ **base_link** -> $L_{1}$ **JAW** (revolute)
  Variable: $q_{1}$

- **origin xyz**: (0.794021, 0.612882, -0.042003) m
- **origin rpy**: (1.570796, 0, -0.788864) rad
- **axis**: (0, -1, 0)
- **limits**: [0, 6.283185] rad ([0deg, 360deg])

### Local Transform

$T^{0}_{1}(q_{1}) = T_{fixed} \cdot R_{axis}(q_{1})$ where:

$$
T_{fixed} = \begin{bmatrix}
0.704652 & 0 & -0.709553 & 0.794021 \\
-0.709553 & 0 & -0.704652 & 0.612882 \\
0 & 1 & 0 & -0.042003 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{1}) = \begin{bmatrix}
c_{1} & 0 & -s_{1} & 0 \\
0 & 1 & 0 & 0 \\
s_{1} & 0 & c_{1} & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Revolute_2

$L_{1}$ **JAW** -> $L_{2}$ **JAW_2** (continuous)
  Variable: $q_{2}$

- **origin xyz**: (0.15, 0.16, 0.02) m
- **origin rpy**: (0, 0, -1.939557) rad
- **axis**: (0, 0, -1)

### Local Transform

$T^{1}_{2}(q_{2}) = T_{fixed} \cdot R_{axis}(q_{2})$ where:

$$
T_{fixed} = \begin{bmatrix}
-0.36046 & 0.932775 & 0 & 0.15 \\
-0.932775 & -0.36046 & 0 & 0.16 \\
0 & 0 & 1 & 0.02 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{2}) = \begin{bmatrix}
c_{2} & s_{2} & 0 & 0 \\
-s_{2} & c_{2} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Revolute_3

$L_{2}$ **JAW_2** -> $L_{3}$ **JAW_3** (continuous)
  Variable: $q_{3}$

- **origin xyz**: (-0.365, 0, 0) m
- **origin rpy**: (3.014035, 1.570796, 0) rad
- **axis**: (-1, 0, 0)

### Local Transform

$T^{2}_{3}(q_{3}) = T_{fixed} \cdot R_{axis}(q_{3})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & 0.127212 & -0.991876 & -0.365 \\
0 & -0.991876 & -0.127212 & 0 \\
-1 & 0 & 0 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{3}) = \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & c_{3} & s_{3} & 0 \\
0 & -s_{3} & c_{3} & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Revolute_4

$L_{3}$ **JAW_3** -> $L_{4}$ **JAW_4** (revolute)
  Variable: $q_{4}$

- **origin xyz**: (0.048, -0.246, 0.045) m
- **origin rpy**: (0, 0, 3.141593) rad
- **axis**: (0, -1, 0)
- **limits**: [0, 0] rad ([0deg, 0deg])

### Local Transform

$T^{3}_{4}(q_{4}) = T_{fixed} \cdot R_{axis}(q_{4})$ where:

$$
T_{fixed} = \begin{bmatrix}
-1 & 0 & 0 & 0.048 \\
0 & -1 & 0 & -0.246 \\
0 & 0 & 1 & 0.045 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{4}) = \begin{bmatrix}
c_{4} & 0 & -s_{4} & 0 \\
0 & 1 & 0 & 0 \\
s_{4} & 0 & c_{4} & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Revolute_5

$L_{4}$ **JAW_4** -> $L_{5}$ **JAW_5** (continuous)
  Variable: $q_{5}$

- **origin xyz**: (0.022, 0.115, 0) m
- **origin rpy**: (0, 0.523599, 1.570796) rad
- **axis**: (0, -1, 0)

### Local Transform

$T^{4}_{5}(q_{5}) = T_{fixed} \cdot R_{axis}(q_{5})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & -1 & 0 & 0.022 \\
0.866025 & 0 & 0.5 & 0.115 \\
-0.5 & 0 & 0.866025 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{5}) = \begin{bmatrix}
c_{5} & 0 & -s_{5} & 0 \\
0 & 1 & 0 & 0 \\
s_{5} & 0 & c_{5} & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Revolute_6

$L_{5}$ **JAW_5** -> $L_{6}$ **JAW_6** (revolute)
  Variable: $q_{6}$

- **origin xyz**: (0.094, 0.022, 0) m
- **origin rpy**: (0, 0, -1.570796) rad
- **axis**: (0, -1, 0)
- **limits**: [0, 0] rad ([0deg, 0deg])

### Local Transform

$T^{5}_{6}(q_{6}) = T_{fixed} \cdot R_{axis}(q_{6})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & 1 & 0 & 0.094 \\
-1 & 0 & 0 & 0.022 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{6}) = \begin{bmatrix}
c_{6} & 0 & -s_{6} & 0 \\
0 & 1 & 0 & 0 \\
s_{6} & 0 & c_{6} & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Revolute_7

$L_{6}$ **JAW_6** -> $L_{7}$ **JAW_7** (continuous)
  Variable: $q_{7}$

- **origin xyz**: (-0.04231, 0.032892, 0.009) m
- **origin rpy**: (1.570796, 1.570796, 0) rad
- **axis**: (-1, 0, 0)

### Local Transform

$T^{6}_{7}(q_{7}) = T_{fixed} \cdot R_{axis}(q_{7})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & 1 & 0 & -0.04231 \\
0 & 0 & -1 & 0.032892 \\
-1 & 0 & 0 & 0.009 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{7}) = \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & c_{7} & s_{7} & 0 \\
0 & -s_{7} & c_{7} & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Revolute_8

$L_{6}$ **JAW_6** -> $L_{8}$ **JAW_8** (continuous)
  Variable: $q_{8}$

- **origin xyz**: (0.04081, 0.032892, 0.009) m
- **origin rpy**: (1.570796, -1.570796, 0) rad
- **axis**: (1, 0, 0)

### Local Transform

$T^{6}_{8}(q_{8}) = T_{fixed} \cdot R_{axis}(q_{8})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & -1 & 0 & 0.04081 \\
0 & 0 & -1 & 0.032892 \\
1 & 0 & 0 & 0.009 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{8}) = \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & c_{8} & -s_{8} & 0 \\
0 & s_{8} & c_{8} & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Global Transform Chains

Transform from root $L_0$ to any link, as product of local transforms along the kinematic chain.

$$T^{0}_{2} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2})\quad (L_0 \to L_{2}: \text{JAW_2})$$

$$T^{0}_{3} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2}) \cdot T^{2}_{3}(q_{3})\quad (L_0 \to L_{3}: \text{JAW_3})$$

$$T^{0}_{4} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2}) \cdot T^{2}_{3}(q_{3}) \cdot T^{3}_{4}(q_{4})\quad (L_0 \to L_{4}: \text{JAW_4})$$

$$T^{0}_{5} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2}) \cdot T^{2}_{3}(q_{3}) \cdot T^{3}_{4}(q_{4}) \cdot T^{4}_{5}(q_{5})\quad (L_0 \to L_{5}: \text{JAW_5})$$

$$T^{0}_{6} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2}) \cdot T^{2}_{3}(q_{3}) \cdot T^{3}_{4}(q_{4}) \cdot T^{4}_{5}(q_{5}) \cdot T^{5}_{6}(q_{6})\quad (L_0 \to L_{6}: \text{JAW_6})$$

$$T^{0}_{7} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2}) \cdot T^{2}_{3}(q_{3}) \cdot T^{3}_{4}(q_{4}) \cdot T^{4}_{5}(q_{5}) \cdot T^{5}_{6}(q_{6}) \cdot T^{6}_{7}(q_{7})\quad (L_0 \to L_{7}: \text{JAW_7})$$

$$T^{0}_{8} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2}) \cdot T^{2}_{3}(q_{3}) \cdot T^{3}_{4}(q_{4}) \cdot T^{4}_{5}(q_{5}) \cdot T^{5}_{6}(q_{6}) \cdot T^{6}_{8}(q_{8})\quad (L_0 \to L_{8}: \text{JAW_8})$$

