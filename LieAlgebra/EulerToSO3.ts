import { SO3 } from "./SO3";

export class SO3Conversions {
  private static EPS = 1e-9;

  /**
   * Converts [roll, pitch, yaw] (radians) -> 3x3 SO(3) matrix (row-major Array)
   * Convention: ZYX intrinsic (applies Z-yaw, then Y-pitch, then X-roll)
   */
  public static eulerToSO3(euler: [number, number, number]): number[] {
    const [roll, pitch, yaw] = euler;
    const cr = Math.cos(roll), sr = Math.sin(roll);
    const cp = Math.cos(pitch), sp = Math.sin(pitch);
    const cy = Math.cos(yaw), sy = Math.sin(yaw);

    return [
      cy * cp,
      cy * sp * sr - sy * cr,
      cy * sp * cr + sy * sr,
      sy * cp,
      sy * sp * sr + cy * cr,
      sy * sp * cr - cy * sr,
      -sp,
      cp * sr,
      cp * cr
    ];
  }

  /**
   * Converts 3x3 SO(3) matrix (row-major Array) -> [roll, pitch, yaw] (radians)
   * Handles gimbal lock at pitch appr +-pi/2 gracefully.
   */
  public static so3ToEuler(rotationMatrix: SO3): [number, number, number] {
    const matrix = rotationMatrix.matrix 
    const m00 = matrix[0], m01 = matrix[1], m02 = matrix[2];
    const m10 = matrix[3], m11 = matrix[4], m12 = matrix[5];
    const m20 = matrix[6], m21 = matrix[7], m22 = matrix[8];

    let pitch: number, roll: number, yaw: number;

    if (m20 > 1 - this.EPS) {
      // Gimbal lock: pitch appr -pi/2
      pitch = -Math.PI / 2;
      roll = 0; // Arbitrary
      yaw = Math.atan2(m01, m11);
    } else if (m20 < -1 + this.EPS) {
      // Gimbal lock: pitch appr +pi/2
      pitch = Math.PI / 2;
      roll = 0; // Arbitrary
      yaw = Math.atan2(-m01, m11);
    } else {
      pitch = -Math.asin(m20);
      roll = Math.atan2(m21, m22);
      yaw = Math.atan2(m10, m00);
    }

    return [roll, pitch, yaw];
  }

  /**
 * Creates an SO(3) matrix from a unit quaternion [w, x, y, z].
 */
  public static fromQuaternion(q: [number, number, number, number]): SO3 {
    const [w, x, y, z] = q;
    const wx = w * x, wy = w * y, wz = w * z;
    const xx = x * x, xy = x * y, xz = x * z;
    const yy = y * y, yz = y * z, zz = z * z;

    const mat = [
      1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy),
      2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx),
      2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)
    ];
    return new SO3(mat);
  }

  /**
 * Converts the SO(3) rotation matrix to a unit quaternion [w, x, y, z].
 * Uses the robust trace-based method to avoid singularities.
 */
  public static toQuaternion(rotationMatrix: SO3): [number, number, number, number] {
    const m = rotationMatrix.matrix;
    const trace = m[0] + m[4] + m[8];
    let w, x, y, z;

    if (trace > 0) {
      const S = Math.sqrt(trace + 1.0) * 2; // S = 4 * w
      w = 0.25 * S;
      x = (m[7] - m[5]) / S;
      y = (m[2] - m[6]) / S;
      z = (m[3] - m[1]) / S;
    } else if (m[0] > m[4] && m[0] > m[8]) {
      const S = Math.sqrt(1.0 + m[0] - m[4] - m[8]) * 2; // S = 4 * x
      w = (m[7] - m[5]) / S;
      x = 0.25 * S;
      y = (m[3] + m[1]) / S;
      z = (m[2] + m[6]) / S;
    } else if (m[4] > m[8]) {
      const S = Math.sqrt(1.0 + m[4] - m[0] - m[8]) * 2; // S = 4 * y
      w = (m[2] - m[6]) / S;
      x = (m[3] + m[1]) / S;
      y = 0.25 * S;
      z = (m[7] + m[5]) / S;
    } else {
      const S = Math.sqrt(1.0 + m[8] - m[0] - m[4]) * 2; // S = 4 * z
      w = (m[3] - m[1]) / S;
      x = (m[2] + m[6]) / S;
      y = (m[7] + m[5]) / S;
      z = 0.25 * S;
    }

    return [w, x, y, z];
  }
}