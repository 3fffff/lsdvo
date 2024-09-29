import { SO3 } from "./SO3";
import { Vec } from "./Vec";

export class SE3 {
  public rotation: SO3;

  public translation: Float32Array;

  public constructor(rotation?: any, translation?: any) {
    if (((rotation != null && rotation instanceof <any>SO3) || rotation === null) && ((translation != null && translation instanceof <any>Float32Array && (translation.length == 0 || translation[0] == null || (typeof translation[0] === 'number'))) || translation === null)) {
      this.rotation = rotation;
      this.translation = translation;
      this.assertNotNaN();
    } else if (((rotation != null && rotation instanceof <any>SE3) || rotation === null) && translation === undefined) {
      let __args = arguments;
      let se3: any = __args[0];
      this.translation = se3.translation;
      this.rotation = new SO3(se3.rotation);
      this.assertNotNaN();
    } else{
      this.rotation = new SO3();
      this.setTranslation3(0, 0, 0);
      this.assertNotNaN();
    }
  }

  public getRotation(): SO3 {
    return this.rotation;
  }

  public getRotationMatrix(): Float32Array {
    return this.rotation.matrix;
  }

  public getTranslation(): Float32Array {
    return this.translation;
  }

  /**
   * Exponentiate a Vector in the Lie Algebra to generate a new SE3.
   * 
   * @param {Array} vec6 assumes size 6
   * @return
   * @return {SE3}
   */
  public static exp(vec6: Float32Array): SE3 {
    let one_6th: number = 1.0 / 6.0;
    let one_20th: number = 1.0 / 20.0;
    let result: SE3 = new SE3();
    let t: Float32Array = new Float32Array([vec6[0], vec6[1], vec6[2]]);
    let w: Float32Array = new Float32Array([vec6[3], vec6[4], vec6[5]]);
    let theta_sq: number = Vec.dot(w, w);
    let theta: number = Math.sqrt(theta_sq);
    let A: number;
    let B: number;
    let cross: Float32Array = Vec.cross(w, t);
    if (theta_sq < 1.0E-12) {
      A = 1.0 - one_6th * theta_sq;
      B = 0.5;
      result.setTranslation1(Vec.vecAdd2(t, Vec.scalarMult2(cross, 0.5)));
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
      result.setTranslation1(Vec.vecAdd2(Vec.vecAdd2(t, Vec.scalarMult2(cross, B)), Vec.scalarMult2(Vec.cross(w, cross), C)));
    }
    result.rotation.matrix = SO3.rodrigues_so3_exp(w, A, B);
    return result;
  }

  public static ln(se3: SE3): Float32Array {
    let rot: Float32Array = se3.getRotation().ln();
    let theta: number = Math.sqrt(Vec.dot(rot, rot));
    let shtot: number = 0.5;
    if (theta > 1.0E-12) {
      shtot = Math.sin(theta / 2.0) / theta;
    }
    let halfrotator: SO3 = new SO3(SO3.exp(Vec.scalarMult2(rot, -0.5)));
    let rottrans: Float32Array = Vec.matVecMultiplySqr(halfrotator.matrix, se3.getTranslation(), 3);
    if (theta > 1.0E-12) {
      Vec.vecMinus(rottrans, Vec.scalarMult2(rot, (Vec.dot(se3.getTranslation(), rot) * (1.0 - 2.0 * shtot) / Vec.dot(rot, rot))));
    } else {
      Vec.vecMinus(rottrans, Vec.scalarMult2(rot, (Vec.dot(se3.getTranslation(), rot) / 24.0)));
    }
    Vec.scalarMult(rottrans, 1.0 / (2.0 * shtot));
    let result: Float32Array = new Float32Array([rottrans[0], rottrans[1], rottrans[2], rot[0], rot[1], rot[2]]);
    for (let i = 0; i < result.length; i++)
      if (isNaN(result[i])) throw new Error("isNaN(d)");
    return result;
  }

  public static inverse(se3: SE3): SE3 {
    let inverse: SE3 = new SE3();
    inverse.rotation = SO3.inverse(se3.rotation);
    inverse.translation = Vec.scalarMult2((Vec.matVecMultiplySqr(inverse.getRotationMatrix(), se3.translation, 3)), -1);
    inverse.assertNotNaN();
    return inverse;
  }

  public inverse(): SE3 {
    return SE3.inverse(this);
  }

  public setTranslation1(vec3: Float32Array) {
    let translationVec3: Float32Array = new Float32Array([vec3[0], vec3[1], vec3[2]]);
    this.translation = translationVec3;
  }

  public setTranslation3(x: number, y: number, z: number) {
    let translationVec3: Float32Array = new Float32Array([x, y, z]);
    this.translation = translationVec3;
  }

  public setTranslation(x?: any, y?: any, z?: any): any {
    if (((typeof x === 'number') || x === null) && ((typeof y === 'number') || y === null) && ((typeof z === 'number') || z === null)) {
      return this.setTranslation3(x, y, z);
    } else if (((x != null && x instanceof <any>Array && (x.length == 0 || x[0] == null || (typeof x[0] === 'number'))) || x === null) && y === undefined && z === undefined) {
      return this.setTranslation1(x);
    } else throw new Error('invalid overload');
  }

  /**
   * Right multiply by given SE3
   * @param {SE3} se3
   */
  public mulEq(se3: SE3) {
    this.translation = Vec.vecAdd2(this.translation, Vec.matVecMultiplySqr(this.rotation.matrix, se3.translation, 3));
    this.rotation.matrix = Vec.multMatrix(this.rotation.matrix, se3.rotation.matrix, 3, 3, 3, 3);
    this.assertNotNaN();
  }

  /**
   * Right multiply by given SE3
   * @param {SE3} se3
   * @return {SE3}
   */
  public mul(se3: SE3): SE3 {
    let translation: Float32Array = Vec.vecAdd2(this.translation, Vec.matVecMultiplySqr(this.rotation.matrix, se3.translation, 3));
    let rotation: Float32Array = Vec.multMatrix(this.rotation.matrix, se3.rotation.matrix, 3, 3, 3, 3);
    return new SE3(new SO3(rotation), translation);
  }

  
  public mulFloat(point: Float32Array): Float32Array {
    return Vec.vecAdd2(this.getRotation().matrix, Vec.matVecMultiplySqr(this.getRotationMatrix(), point, 3));
  }

  public assertNotNaN() {
    if (isNaN(this.translation[0])) throw new Error("(isNaN(this.translation[0]));");
    if (isNaN(this.translation[1])) throw new Error("(isNaN(this.translation[1]));");
    if (isNaN(this.translation[2])) throw new Error("(isNaN(this.translation[2]));");
    this.rotation.assertNotNaN();
  }
}