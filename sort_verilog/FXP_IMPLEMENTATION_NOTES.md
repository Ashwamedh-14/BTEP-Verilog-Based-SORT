# Fixed-Point Notation Implementation for SORT Hardware

## Overview
All Verilog modules in the `sort_verilog` folder have been updated to incorporate proper fixed-point (FXP) notation according to the specifications in `Specs/main.tex`. This document details the fixed-point formats used and the bit shift strategies employed throughout the design.

---

## Fixed-Point Notation Summary

### State Vector **x** (7 elements, 112 bits total)
- **cx, cy, h**: FXP(16,0) - Unsigned, 16 bits, 0 fractional bits
- **r**: FXP(8,8) - Unsigned, 8 integer bits, 8 fractional bits  
- **vx, vy, vh**: FXP(8,8) - Signed, 8 integer bits, 8 fractional bits

**Bit Layout**: `[cx(111:96), cy(95:80), h(79:64), r(63:48), vx(47:32), vy(31:16), vh(15:0)]`

### Covariance Matrix **P** (10 elements, 160 bits total)
- **Diagonal (σ²_cx, σ²_cy, σ²_h, σ²_r, σ²_vx, σ²_vy, σ²_vh)**: FXP(12,4) - Unsigned, 12 integer bits, 4 fractional bits
- **Cross-covariance (σ_vx, σ_vy, σ_vh)**: FXP(8,8) - Signed, 8 integer bits, 8 fractional bits

**Bit Layout**: `[σ²_cx(159:144), σ²_cy(143:128), σ²_h(127:112), σ²_r(111:96), σ²_vx(95:80), σ²_vy(79:64), σ²_vh(63:48), σ_vx(47:32), σ_vy(31:16), σ_vh(15:0)]`

### Measurement Vector **z** (4 elements, 64 bits)
- **x, y, h**: FXP(16,0) - Unsigned
- **r**: FXP(8,8) - Unsigned

### Innovation Vector **y** (4 elements, 64 bits)
- **y_x, y_y, y_h**: FXP(16,0) - Signed
- **y_r**: FXP(8,8) - Signed

### Process & Measurement Noise (Q, R)
- All elements: FXP(8,8) - Unsigned

### Innovation Covariance **S** (4 elements, 64 bits)
- All elements: FXP(12,4) - Unsigned

---

## Bit Shift Strategy

### Multiplication Bit Shifts

When multiplying two fixed-point numbers, the result has more fractional bits than needed. Right shifts are applied to normalize:

| Operation | Format | Temp Format | Shift Right | Result Format |
|-----------|--------|-------------|-------------|---------------|
| FXP(16,0) × FXP(8,8) | 0 + 8 | 8 frac bits | 8 | FXP(16,0) |
| FXP(12,4) × FXP(8,8) | 4 + 8 | 12 frac bits | 8 | FXP(12,4) |
| FXP(12,4) × FXP(12,4) | 4 + 4 | 8 frac bits | 4 | FXP(12,4) |

### Division Bit Shifts

When dividing FXP numbers, maintain precision by left-shifting the numerator before division:

**K Calculation** (σ²/ρ where both are FXP(12,4)):
- No shift needed: both have 4 frac bits, result is also FXP(12,4)
- K_i = (σ²_i_fxp) / (ρ_i_fxp) → result already in correct format

**Covariance Update** (σ⁴/ρ where σ² is FXP(12,4)):
- temp = σ²_fxp × σ²_fxp → 8 fractional bits
- Right shift by 4: (temp >> 4) / ρ_fxp → FXP(12,4)

**Cross-Covariance Update** (σ² × σ_cross / ρ):
- temp = σ²_fxp × σ_cross_fxp → 12 fractional bits
- Right shift by 8: (temp >> 8) / ρ_fxp → FXP(8,8)

---

## Module Updates

### 1. **arithmetic.v** - Enhanced Documentation
- Updated `Adder_Subtractor_16bit` module documentation
- Added clear explanation of mixed fixed-point format support

### 2. **KalmanBlock.v** - Core Fixed-Point Implementation

#### Mult_Div_16bit (Utility Module)
- Basic multiplication-division: Y = (A × B) / C
- Does NOT include automatic bit shifting
- Caller must apply post-division shifts

#### Mult_Div_Shift_16bit (New Module)
- Includes automatic left/right shift control
- Useful for format conversion operations

#### Predict Module
- **State Prediction**: Adds FXP(8,8) velocities to FXP(16,0) positions
  - Bit shift right by 8: `x_out = cx + (vx >>> 8)`
- **Covariance Prediction**: Updates P matrix using Equation from spec
  - Cross-covariance term: `((p_cov_vx << 1) >>> 8)` for 2× multiplication and format conversion

#### Innovate Module
- Computes residual: y = z - x (element-wise, preserves formats)
- Computes innovation covariance: S = P + R (diagonal addition)
- Documentation explains format compatibility

#### Update Module (Most Complex)
- **State Update**: K×y where K = σ²/ρ
  - Multiply: σ²_fxp(FXP12.4) × y_fxp(FXP16.0) → 4 fractional bits
  - Divide: temp / S_fxp(FXP12.4) 
  - Post-division right shift by 4 normalizes to FXP(16,0)

- **Covariance Update**: P = σ² - (σ⁴/ρ)
  - Square: σ²_fxp × σ²_fxp → 8 fractional bits
  - Right shift by 4: temp >> 4
  - Divide: (temp >> 4) / S_fxp → FXP(12,4)
  
- **Cross-Covariance Update**: σ - (σ² × σ_cross / ρ)
  - Multiply: σ²_fxp × σ_cross_fxp → 12 fractional bits
  - Right shift by 8: temp >> 8
  - Divide: (temp >> 8) / S_fxp → FXP(8,8)

### 3. **memory.v** - Cleaned Up Utilities
- Removed duplicate modules (now in KalmanBlock.v)
- Kept `Mult_Div_16bit` as utility
- Added comprehensive FXP documentation header

### 4. **IoU_Block.v** - Enhanced Coordinate Conversion

#### ST_to_BBox Module
- Converts state vector to bounding box
- **Width calculation**: w = (h × r) >> 8
  - FXP(16,0) × FXP(8,8) → right shift by 8 for FXP(16,0)

#### BBox_to_ST Module
- Converts bounding box back to state vector
- **Aspect ratio calculation**: r = (w << 8) / h
  - Left shift w by 8 before division
  - Result stored as FXP(8,8)

#### IOU Module
- Uses 32-bit arithmetic for area calculations
- Avoids division: 10×area_i > threshold×area_u

### 5. **SORT_Controller.v** - Enhanced Documentation
- Added comprehensive module header with FXP notation reference
- Documented state and covariance storage bit layouts
- Added state machine flow documentation
- Clarified module instantiation purposes

---

## Key Implementation Details

### Signed vs. Unsigned Arithmetic
- Use `$signed()` cast when multiplying signed values (e.g., velocities)
- Example: `ky_cx_temp = sigma2_cx * $signed(y[63:48])`

### Overflow Protection
- All intermediate calculations use 32-bit values
- Post-division shifts prevent overflow in final results
- Division by zero returns 0 safely

### Precision Maintenance
- Each operation carefully tracks fractional bit position
- Right shifts applied only when necessary to prevent precision loss
- Left shifts applied before division to maintain precision

---

## Validation Checklist

- [x] All state vector elements use correct FXP formats
- [x] All covariance matrix elements use correct FXP formats
- [x] Multiplication operations include appropriate right shifts
- [x] Division operations preserve precision through shifts
- [x] Signed/unsigned operations correctly handled
- [x] Overflow conditions protected
- [x] Division by zero handled gracefully

---

## Testing Notes

When testing the hardware implementation:

1. **Small value preservation**: FXP(8,8) format should preserve values < 2^8 with 8-bit fractional precision
2. **Aspect ratio**: FXP(8,8) aspect ratios should maintain values in range [0, 256)
3. **Position accuracy**: FXP(16,0) should handle image coordinates up to 640×640
4. **Variance dynamics**: FXP(12,4) covariances should decrease after measurements (update step)
5. **Cross-covariance sign**: Should match the sign of velocity updates

---

## References

- Specifications: `/home/ayush/Desktop/Ahmedabad University/Year 4/Winter Semester/BTEP/Specs/main.tex`
- SORT Algorithm: Section 3 & 5 (Theory, Mathematical Equations)
- Register Sizing: Eq. (12) - justification for 16-bit registers
- State Prediction: Eq. (State_Predict) - state evolution
- Covariance Prediction: Eq. (Cov_Predict) - simplified matrix operations
- Update Equations: Eq. (X_update), Eq. (P-update) - Kalman gain and covariance updates

---

*Last Updated: May 2, 2026*
*Implementation: Fixed-Point SORT Hardware Tracker*
