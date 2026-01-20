import { SO3 } from "./SO3";
import { Vec } from "./Vec";

export class SE3 {
  public rotation: SO3;

  public translation: Array<number>;

  public constructor(rotation?: any, translation?: any) {
    if (rotation != null && rotation instanceof SO3) {
      this.rotation = rotation;
      this.translation = translation ?? [0, 0, 0];
    } else {
      this.rotation = new SO3();
      this.setTranslation([0, 0, 0]);
    }
  }

  public getRotation(): SO3 {
    return this.rotation;
  }

  public getRotationMatrix(): Array<number> {
    return this.rotation.matrix;
  }

  public getTranslation(): Array<number> {
    return this.translation;
  }

  /**
   * Exponentiate a Vector in the Lie Algebra to generate a new SE3.
   * 
   * @param {Array} vec6 assumes size 6
   * @return
   * @return {SE3}
   */
  public static exp(vec6: Array<number>): SE3 {
    let one_6th: number = 1.0 / 6.0;
    let one_20th: number = 1.0 / 20.0;
    let result: SE3 = new SE3();
    let t: Array<number> = [vec6[0], vec6[1], vec6[2]];
    let w: Array<number> = [vec6[3], vec6[4], vec6[5]];
    let theta_sq: number = Vec.dot(w, w);
    let theta: number = Math.sqrt(theta_sq);
    let A: number;
    let B: number;
    let cross: Array<number> = Vec.cross(w, t);
    if (theta_sq < 1.0E-12) {
      A = 1.0 - one_6th * theta_sq;
      B = 0.5;
      result.setTranslation(Vec.vecAdd2(t, Vec.scalarMul2(cross, 0.5)));
    } else {
      let C: number;
      if (theta_sq < 1.0E-12) {
        C = one_6th * (1.0 - one_20th * theta_sq);
        A = 1.0 - theta_sq * C;
        B = 0.5 - 0.25 * one_6th * theta_sq;
      } else {
        let inv_theta: number = 1.0 / theta;
        A = Math.sin(theta) * inv_theta;
        B = (1.0 - Math.cos(theta)) * (inv_theta * inv_theta);
        C = (1.0 - A) * (inv_theta * inv_theta);
      }
      result.setTranslation(Vec.vecAdd2(Vec.vecAdd2(t, Vec.scalarMul2(cross, B)), Vec.scalarMul2(Vec.cross(w, cross), C)));
    }
    result.rotation.matrix = SO3.rodrigues_so3_exp(w, A, B);
    return result;
  }

  public static ln(se3: SE3): Array<number> {
    let rot: Array<number> = se3.getRotation().ln();
    let theta: number = Math.sqrt(Vec.dot(rot, rot));
    let shtot: number = 0.5;
    if (theta > 1.0E-12) {
      shtot = Math.sin(theta / 2.0) / theta;
    }
    let halfrotator: SO3 = new SO3(SO3.exp(Vec.scalarMul2(rot, -0.5)));
    let rottrans: Array<number> = Vec.matVecMultiplySqr(halfrotator.matrix, se3.getTranslation(), 3);

    if (theta > 1.0E-12) {
      let correction = Vec.scalarMul2(
        rot,
        (Vec.dot(se3.getTranslation(), rot) * (1.0 - 2.0 * shtot) / Vec.dot(rot, rot))
      );
      rottrans = Vec.vecMinus2(rottrans, correction);
    } else {
      let correction = Vec.scalarMul2(rot, Vec.dot(se3.getTranslation(), rot) / 24.0);
      rottrans = Vec.vecMinus2(rottrans, correction);
    }

    rottrans = Vec.scalarMul2(rottrans, 1.0 / (2.0 * shtot));
    return [rottrans[0], rottrans[1], rottrans[2], rot[0], rot[1], rot[2]];
  }

  public static inverse(se3: SE3): SE3 {
    let inverse: SE3 = new SE3();
    inverse.rotation = SO3.inverse(se3.rotation);
    inverse.translation = Vec.scalarMul2((Vec.matVecMultiplySqr(inverse.getRotationMatrix(), se3.translation, 3)), -1);
    return inverse;
  }

  public inverse(): SE3 {
    return SE3.inverse(this);
  }

  public setTranslation(vec3: Array<number>) {
    let translationVec3: Array<number> = [vec3[0], vec3[1], vec3[2]];
    this.translation = translationVec3;
  }
  /**
   * Right multiply by given SE3
   * @param {SE3} se3
   */
  public mulEq(se3: SE3) {
    this.translation = Vec.vecAdd2(this.translation, Vec.matVecMultiplySqr(this.rotation.matrix, se3.translation, 3));
    this.rotation.matrix = Vec.multMatrix(this.rotation.matrix, se3.rotation.matrix, 3, 3, 3, 3);
  }

  /**
   * Right multiply by given SE3
   * @param {SE3} se3
   * @return {SE3}
   */
  public mulSE3(se3: SE3): SE3 {
    let translation: Array<number> = Vec.vecAdd2(this.translation, Vec.matVecMultiplySqr(this.rotation.matrix, se3.translation, 3));
    let rotation: Array<number> = Vec.multMatrix(this.rotation.matrix, se3.rotation.matrix, 3, 3, 3, 3);
    return new SE3(new SO3(rotation), translation);
  }

  public mulFloat(point: Array<number>): Array<number> {
    return Vec.vecAdd2(this.getTranslation(), Vec.matVecMultiplySqr(this.getRotationMatrix(), point, 3));
  }
}