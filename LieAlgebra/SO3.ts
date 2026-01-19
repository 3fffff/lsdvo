import { Vec } from "./Vec";

export class SO3 {
  public static M_SQRT1_2: number = Math.sqrt(0.5);
  public matrix: Float32Array;

  public constructor(mat33?: any) {
    if (mat33 != null && mat33 instanceof Float32Array) {
      this.set33(mat33);
    } else if ((mat33 != null && mat33 instanceof SO3)) {
      let __args = arguments;
      let rotation: any = __args[0];
      this.matrix = rotation.matrix;
    } else {
      this.matrix = Vec.matrixEye(3);
    }
  }

  public set31(vec3: Float32Array) {
    this.set33(SO3.exp(vec3));
  }

  public set33(mat33: Float32Array) {
    this.matrix = mat33;
    this.coerce();
  }

  /**
   * Performs exponential, returns SO3 matrix.
   */
  public static exp(vec3: Float32Array): Float32Array {
    let one_6th: number = 1.0 / 6.0;
    let one_20th: number = 1.0 / 20.0;
    let theta_sq: number = Vec.dot(vec3, vec3);
    let theta: number = Math.sqrt(theta_sq);
    let A: number;
    let B: number;
    if (theta_sq < 1.0E-12) {
      A = 1.0 - one_6th * theta_sq;
      B = 0.5;
    } else {
      if (theta_sq < 1.0E-12) {
        B = 0.5 - 0.25 * one_6th * theta_sq;
        A = 1.0 - theta_sq * one_6th * (1.0 - one_20th * theta_sq);
      } else {
        let inv_theta: number = 1.0 / theta;
        A = Math.sin(theta) * inv_theta;
        B = (1.0 - Math.cos(theta)) * (inv_theta * inv_theta);
      }
    }
    let result: Float32Array = SO3.rodrigues_so3_exp(vec3, A, B);
    return result;
  }

  /**
   * Compute a rotation exponential using the Rodrigues Formula.
   */
  public static rodrigues_so3_exp(w: Float32Array, A: number, B: number): Float32Array {
    let R: Float32Array = new Float32Array(3 * 3);
      let wx2: number = <number>w[0] * w[0];
      let wy2: number = <number>w[1] * w[1];
      let wz2: number = <number>w[2] * w[2];
      R[0 * 3 + 0] = 1.0 - B * (wy2 + wz2);
      R[1 * 3 + 1] = 1.0 - B * (wx2 + wz2);
      R[2 * 3 + 2] = 1.0 - B * (wx2 + wy2);
      let a2: number = A * w[2];
      let b01: number = B * (w[0] * w[1]);
      R[0 * 3 + 1] = b01 - a2;
      R[1 * 3 + 0] = b01 + a2;
      let a1: number = A * w[1];
      let b02: number = B * (w[0] * w[2]);
      R[0 * 3 + 2] = b02 + a1;
      R[2 * 3 + 0] = b02 - a1;
      let a0: number = A * w[0];
      let b12: number = B * (w[1] * w[2]);
      R[1 * 3 + 2] = b12 - a0;
      R[2 * 3 + 1] = b12 + a0;
    return R;
  }

  /**
   * Take the logarithm of the matrix, generating the corresponding vector in the
   * Lie Algebra.
   */
  public ln(): Float32Array {
    this.coerce();
    let result: Float32Array = new Float32Array([0, 0, 0]);
    let cos_angle: number = (this.matrix[0 * 3 + 0] + this.matrix[1 * 3 + 1] + this.matrix[2 * 3 + 2] - 1.0) * 0.5;
    result[0] = (this.matrix[2 * 3 + 1] - this.matrix[1 * 3 + 2]) / 2.0;
    result[1] = (this.matrix[0 * 3 + 2] - this.matrix[2 * 3 + 0]) / 2.0;
    result[2] = (this.matrix[1 * 3 + 0] - this.matrix[0 * 3 + 1]) / 2.0;
    let sin_angle_abs: number = Math.sqrt(Vec.dot(result, result));
    if (cos_angle > SO3.M_SQRT1_2) {
      if (sin_angle_abs > 0) {
        let s: number = Math.asin(sin_angle_abs) / sin_angle_abs;
        Vec.scalarMult(result, s);
      }
    } else if (cos_angle > -SO3.M_SQRT1_2) {
      let angle_s: number = Math.acos(cos_angle) / sin_angle_abs;
      Vec.scalarMult(result, angle_s);
    } else {
      let angle: number = Math.PI - Math.asin(sin_angle_abs);
      let d0: number = this.matrix[0 * 3 + 0] - cos_angle;
      let d1: number = this.matrix[1 * 3 + 1] - cos_angle;
      let d2: number = this.matrix[2 * 3 + 2] - cos_angle;
      let r2: Float32Array = new Float32Array([0, 0, 0]);
      if (d0 * d0 > d1 * d1 && d0 * d0 > d2 * d2) {
        r2[0] = d0;
        r2[1] = (this.matrix[1 * 3 + 0] + this.matrix[0 * 3 + 1]) / 2.0;
        r2[2] = (this.matrix[0 * 3 + 2] + this.matrix[2 * 3 + 0]) / 2.0;
      } else if (d1 * d1 > d2 * d2) {
        r2[0] = (this.matrix[1 * 3 + 0] + this.matrix[0 * 3 + 1]) / 2.0;
        r2[1] = d1;
        r2[2] = (this.matrix[2 * 3 + 1] + this.matrix[1 * 3 + 2]) / 2.0;
      } else {
        r2[0] = (this.matrix[0 * 3 + 2] + this.matrix[2 * 3 + 0]) / 2.0;
        r2[1] = (this.matrix[2 * 3 + 1] + this.matrix[1 * 3 + 2]) / 2.0;
        r2[2] = d2;
      }
      if (Vec.dot(r2, result) < 0) Vec.scalarMult(r2, -1);
      Vec.unit(r2);
      Vec.scalarMult(r2, angle);
      result = r2;
    }
    for (let i = 0; i < result.length; i++)
      if (isNaN(result[i])) throw new Error("isNaN(d)");
    return result;
  }

  public static inverse(so3: SO3): SO3 {
    let inverse: SO3 = new SO3(Vec.matrixTranspose(so3.matrix, 3));
    return inverse;
  }

  /**
   * Right multiply by given SO3
   * @param {SO3} so3
   */
  public mulEq(so3: SO3) {
    this.matrix = Vec.multMatrix(this.matrix, so3.matrix, 3, 3, 3, 3);
  }

  public mul(so3: SO3): SO3 {
    return new SO3(Vec.multMatrix(this.matrix, so3.matrix, 3, 3, 3, 3));
  }

  public coerce() {
    let vec0: Float32Array = Vec.getRow(this.matrix, 0, 3);
    let vec1: Float32Array = Vec.getRow(this.matrix, 1, 3);
    let vec2: Float32Array = Vec.getRow(this.matrix, 2, 3);
    Vec.unit(vec0);
    Vec.vecMinus(vec1, Vec.cross(vec0, Vec.cross(vec0, vec1)));
    Vec.unit(vec1);
    Vec.vecMinus(vec2, Vec.cross(vec0, Vec.cross(vec0, vec2)));
    Vec.vecMinus(vec2, Vec.cross(vec1, Vec.cross(vec1, vec2)));
    Vec.unit(vec2);
    if (!((Vec.dot(Vec.cross(vec0, vec1), vec2) > 0))) throw new Error("(Vec.dot(Vec.cross(vec0, vec1), vec2) > 0);");
    Vec.setRow(this.matrix, vec0, 0);
    Vec.setRow(this.matrix, vec1, 1);
    Vec.setRow(this.matrix, vec2, 2);
  }
}
