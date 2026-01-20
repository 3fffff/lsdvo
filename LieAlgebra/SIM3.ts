import { SE3 } from "./SE3";
import { SO3 } from "./SO3";
import { Vec } from "./Vec";

export class SIM3 {
  public se3: SE3;

  public scale: number = 0;

  public constructor(se3?: any, scale?: any) {
    if (se3 != null && se3 instanceof SE3) {
      this.se3 = se3;
      this.scale = scale ?? 1;
    } else {
      this.se3 = new SE3();
      this.scale = 1;
    }
  }

  public getSE3(): SE3 {
    return this.se3;
  }

  public getScale(): number {
    return this.scale;
  }

  public getTranslation(): Array<number> {
    return this.se3.getTranslation();
  }

  getTranslationMat(): Array<number> {
    return this.se3.getTranslation();
  }

  getRotation(): SO3 {
    return this.se3.getRotation();
  }

  public getRotationMatrix(): Array<number> {
    return this.se3.getRotationMatrix();
  }

  public SIM3(sim3: SIM3): SIM3 {
    let newSim3: SIM3 = new SIM3(this);
    newSim3.se3.translation = Vec.vecAdd2(newSim3.se3.translation, Vec.matVecMultiplySqr(this.getRotationMatrix(), Vec.scalarMul2(sim3.getTranslationMat(), this.getScale()), 3));
    newSim3.se3.rotation.mulEq(sim3.getRotation());
    newSim3.scale *= sim3.getScale();
    return newSim3;
  }

  public multiply(sim3: SIM3): SIM3 {
    let newSim3: SIM3 = new SIM3(this);
    newSim3.se3.translation = Vec.vecAdd2(
      newSim3.se3.translation,
      Vec.matVecMultiplySqr(this.getRotationMatrix(), Vec.scalarMul2(sim3.getTranslationMat(), this.getScale()), 3)
    );
    newSim3.se3.rotation.mulEq(sim3.getRotation());
    newSim3.scale *= sim3.getScale();
    return newSim3;
  }

  public mul(sim3?: any): any {
    if (sim3 instanceof SIM3) {
      return this.multiply(sim3);
    } else if (sim3 instanceof Array) {
      return this.mulFloat(sim3);
    } else throw new Error('invalid overload');
  }

  public mulFloat(point: Array<number>): Array<number> {
    return Vec.vecAdd2(this.getTranslationMat(), Vec.matVecMultiplySqr(this.getRotationMatrix(), Vec.scalarMul2(point, this.getScale()), 3));
  }

  public static inverse(sim3: SIM3): SIM3 {
    let inverse: SIM3 = new SIM3();
    inverse.se3.rotation = SO3.inverse(sim3.se3.rotation);
    inverse.scale = 1.0 / sim3.scale;
    inverse.se3.translation = Vec.scalarMul2((Vec.matVecMultiplySqr(inverse.getRotationMatrix(), sim3.se3.translation, 3)), -inverse.scale);
    return inverse;
  }

  public inverse(): SIM3 {
    return SIM3.inverse(this);
  }

  public static ln(sim3: SIM3): Array<number> {
    let result: Array<number> = [0, 0, 0, 0, 0, 0, 0];
    let rotResult: Array<number> = sim3.getRotation().ln();
    let theta: number = Vec.magnitude(rotResult);
    let s: number = Math.log(sim3.getScale());
    result[6] = s;
    let coeff: Array<number> = SIM3.compute_rodrigues_coefficients_sim3(s, theta);
    let cross: Array<number> = SIM3.cross_product_matrix(rotResult);
    let W: Array<number> = Vec.matrixAdd2(Vec.matrixAdd2((Vec.matrixMul(Vec.matrixEye(3), coeff[0])), (Vec.matrixMul(cross, coeff[1]))), (Vec.matrixMul(Vec.multMatrix(cross, cross, 3, 3, 3, 3), coeff[2])));
    let transResultMat: Array<number> = Vec.solveSystem(W, sim3.getTranslation());
    result[0] = transResultMat[0];
    result[1] = transResultMat[1];
    result[2] = transResultMat[2];
    result[3] = rotResult[0];
    result[4] = rotResult[1];
    result[5] = rotResult[2];
    return result;
  }

  /**
   * Exponentiate a Vector in the Lie Algebra to generate a new SIM3.
   * 
   * @param {Array} vec7 assumes size 7
   * @return
   * @return {SIM3}
   */
  public static exp(vec7: Array<number>): SIM3 {
    let transVec: Array<number> = [vec7[0], vec7[1], vec7[2]];
    let rotVec: Array<number> = [vec7[3], vec7[4], vec7[5]];
    let scale: number = Math.exp(vec7[6]);
    let rotation: SO3 = new SO3(SO3.exp(rotVec));
    let t: number = Vec.magnitude(rotVec);
    let coeff: Array<number> = SIM3.compute_rodrigues_coefficients_sim3(vec7[6], t);
    let cross: Array<number> = Vec.cross(rotVec, transVec);
    let trans: Array<number> = Vec.vecAdd2(Vec.vecAdd2(Vec.scalarMul2(transVec, coeff[0]), Vec.scalarMul2(cross, coeff[1])), Vec.scalarMul2(Vec.cross(rotVec, cross), coeff[2]));
    let se3: SE3 = new SE3();
    se3.setTranslation(trans);
    se3.rotation = rotation;
    let result: SIM3 = new SIM3(se3, scale);
    return result;
  }

  static cross_product_matrix(vec: Array<number>): Array<number> {
    let result: Array<number> = [0, -vec[2], vec[1], vec[2], 0, -vec[0], -vec[1], vec[0], 0];
    return result;
  }

  static compute_rodrigues_coefficients_sim3(s: number, t: number): Array<number> {
    let coeff: Array<number> = [0, 0, 0];
    let es: number = Math.exp(s);
    let eps: number = 1.0E-6;
    if (Math.abs(s) < eps && Math.abs(t) < eps) {
      coeff[0] = 1.0 + s / 2.0 + s * s / 6.0;
      coeff[1] = 1.0 / 2.0 + s / 3.0 - t * t / 24.0 + s * s / 8.0;
      coeff[2] = 1.0 / 6.0 + s / 8.0 - t * t / 120.0 + s * s / 20.0;
    } else if (Math.abs(s) < eps) {
      coeff[0] = 1.0 + s / 2.0 + s * s / 6.0;
      coeff[1] = (1.0 - Math.cos(t)) / (t * t) + (Math.sin(t) - Math.cos(t) * t) * s / (t * t * t) + (2.0 * Math.sin(t) * t - t * t * Math.cos(t) - 2.0 + 2.0 * Math.cos(t)) * s * s / (2.0 * t * t * t * t);
      coeff[2] = (t - Math.sin(t)) / (t * t * t) - (-t * t - 2.0 + 2.0 * Math.cos(t) + 2.0 * Math.sin(t) * t) * s / (2.0 * t * t * t * t) - (-t * t * t + 6.0 * Math.cos(t) * t + 3.0 * Math.sin(t) * t * t - 6.0 * Math.sin(t)) * s * s / (6.0 * t * t * t * t * t);
    } else if (Math.abs(t) < eps) {
      coeff[0] = (es - 1.0) / s;
      coeff[1] = (s * es + 1.0 - es) / (s * s) - (6.0 * s * es + 6.0 - 6.0 * es + es * s * s * s - 3.0 * es * s * s) * t * t / (6.0 * s * s * s * s);
      coeff[2] = (es * s * s - 2.0 * s * es + 2.0 * es - 2.0) / (2.0 * s * s * s) - (es * s * s * s * s - 4.0 * es * s * s * s + 12.0 * es * s * s - 24.0 * s * es + 24.0 * es - 24.0) * t * t / (24.0 * s * s * s * s * s);
    } else {
      let a: number = es * Math.sin(t);
      let b: number = es * Math.cos(t);
      let inv_s_theta: number = 1.0 / (s * s + t * t);
      coeff[0] = (es - 1.0) / s;
      coeff[1] = (a * s + (1.0 - b) * t) * inv_s_theta / t;
      coeff[2] = (coeff[0] - ((b - 1.0) * s + a * t) * inv_s_theta) / (t * t);
    }
    return coeff;
  }
}
