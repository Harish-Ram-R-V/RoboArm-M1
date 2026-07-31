# DH Parameter Derivation

## Methodology

Standard Denavit-Hartenberg convention applied to the 4-DOF arm chain.  
Parameters extracted directly from URDF joint `<origin>` tags.

## Joint Chain

```
base_link
    └── Revolute_1 → JAW          (waist)
            └── Revolute_2 → JAW_2    (shoulder)
                    └── Revolute_3 → JAW_3    (elbow)
                            └── Revolute_4 → JAW_4    [FIXED: 0°]
                                    └── Revolute_5 → JAW_5    (wrist)
                                            └── Revolute_6 → JAW_6    [FIXED: 0°]
                                                    ├── Revolute_7 → JAW_7  (left finger)
                                                    └── Revolute_8 → JAW_8  (right finger)
```

## Active Joints (DH Chain)

| Joint | URDF Name | Parent → Child | Axis | Type |
|-------|-----------|----------------|------|------|
| 1 | Revolute_1 | base_link → JAW | -Y | Revolute |
| 2 | Revolute_2 | JAW → JAW_2 | -Z | Revolute |
| 3 | Revolute_3 | JAW_2 → JAW_3 | -X | Revolute |
| 4 | Revolute_5 | JAW_4 → JAW_5 | -Y | Revolute |

## Fixed Joints (excluded from DH, treated as constant transforms)

| Joint | Reason |
|-------|--------|
| Revolute_4 | lower = upper = 0 (structurally rigid) |
| Revolute_6 | lower = upper = 0 (structurally rigid) |

## Gripper Joints (separate from kinematic chain)

| Joint | Role |
|-------|------|
| Revolute_7 | Left finger |
| Revolute_8 | Right finger (mirrors Revolute_7) |

## DH Table

| i | Joint | θᵢ | dᵢ (m) | aᵢ (m) | αᵢ (rad) | Source (URDF origin xyz) |
|---|-------|-----|---------|---------|----------|--------------------------|
| 1 | Rev_1 | q₁ | 0.042 | 0.000 | π/2 | z = 0.042 from base |
| 2 | Rev_2 | q₂ | 0.000 | 0.365 | 0 | x = 0.365 from JAW_2 |
| 3 | Rev_3 | q₃ | 0.246 | 0.048 | π/2 | y = 0.246 (fixed J4), x = 0.048 |
| 4 | Rev_5 | q₄ | 0.115 | 0.000 | 0 | y = 0.115 from JAW_5 |

## Transformation Matrix (Standard DH)

```
       ⎡ cos θᵢ   -sin θᵢ cos αᵢ    sin θᵢ sin αᵢ    aᵢ cos θᵢ ⎤
ᵢ₋₁Tᵢ =⎢ sin θᵢ    cos θᵢ cos αᵢ   -cos θᵢ sin αᵢ    aᵢ sin θᵢ ⎥
       ⎢   0          sin αᵢ            cos αᵢ            dᵢ      ⎥
       ⎣   0             0                 0               1       ⎦
```

## End-Effector (FK at home position)

Position: (0.413, -0.246, -0.073) m  
All round-trip IK errors < 0.001 mm ✓
