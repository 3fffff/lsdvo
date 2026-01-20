import { Vec } from "../LieAlgebra/Vec";
import { Frame } from "../DataStructures/Frame";
import { SE3 } from "../LieAlgebra/SE3";

export class Constants {
  public static DIVISION_EPS: number = 1.0E-10;

  public static MAX_VAR: number = 0.5 * 0.5;

  public static VAR_RANDOM_INIT_INITIAL: number = 0.5 * 0.5;
  public static PYRAMID_LEVELS: number = 5;
  public static SE3TRACKING_MAX_LEVEL: number = 5;
  public static SE3TRACKING_MIN_LEVEL: number = 1;
  static MIN_USE_GRAD: number = 5;
  public static MIN_ABS_GRAD_CREATE: number = Constants.MIN_USE_GRAD;
  public static MIN_ABS_GRAD_DECREASE: number = Constants.MIN_USE_GRAD;

  /**
   * ============== constants for validity handling =======================
   */
  public static VALIDITY_COUNTER_MAX: number = 50;
  public static VALIDITY_COUNTER_MAX_VARIABLE: number = 250.0;
  public static VALIDITY_COUNTER_INC: number = 5;
  public static VALIDITY_COUNTER_DEC: number = 5;
  public static VALIDITY_COUNTER_INITIAL_OBSERVE: number = 5;
  public static VAL_SUM_MIN_FOR_CREATE: number = 30;
  public static VAL_SUM_MIN_FOR_KEEP: number = 24;
  public static VAL_SUM_MIN_FOR_UNBLACKLIST: number = 50;
  public static MIN_BLACKLIST: number = -1;
  public static MIN_EPL_GRAD_SQUARED: number = (2.0 * 2.0);
  public static MIN_EPL_LENGTH_SQUARED: number = (1.0 * 1.0);
  public static MIN_EPL_ANGLE_SQUARED: number = (0.3 * 0.3);
  public static MIN_DEPTH: number = 0.05;
  public static useSubpixelStereo: boolean = true;
  public static MAX_EPL_LENGTH_CROP: number = 30.0;
  public static MIN_EPL_LENGTH_CROP: number = 3.0;
  public static GRADIENT_SAMPLE_DIST: number = 1.0;
  public static SAMPLE_POINT_TO_BORDER: number = 7;
  public static MAX_ERROR_STEREO: number = 1300.0;
  public static MIN_DISTANCE_ERROR_STEREO: number = 1.5;
  public static STEREO_EPL_VAR_FAC: number = 2.0;
  public static MAX_DIFF_CONSTANT: number = (40.0 * 40.0);
  public static MAX_DIFF_GRAD_MULT: number = (0.5 * 0.5);
  public static MIN_GOODPERGOODBAD_PIXEL: number = 0.5;
  public static MIN_GOODPERALL_PIXEL: number = 0.04;
  public static MIN_GOODPERALL_PIXEL_ABSMIN: number = 0.01;
  public static INITIALIZATION_PHASE_COUNT: number = 5;
  public static MIN_NUM_MAPPED: number = 5;
  static depthSmoothingFactor: number = 1;
  public static REG_DIST_VAR: number = 0.075 * 0.075 * Constants.depthSmoothingFactor * Constants.depthSmoothingFactor;
  public static DIFF_FAC_SMOOTHING: number = (1.0 * 1.0);
  public static DIFF_FAC_OBSERVE: number = (1.0 * 1.0);
  public static DIFF_FAC_PROP_MERGE: number = (1.0 * 1.0);
  public static debugDisplay: number = 1;
  public static scaledDepthVarTH: number = Math.pow(10, -3.0);
  public static absDepthVarTH: number = Math.pow(10, -1.0);
  public static K: Array<Array<number>> = Array(Constants.PYRAMID_LEVELS);
  public static fx: Array<number> = Array(Constants.PYRAMID_LEVELS);
  public static fy: Array<number> = Array(Constants.PYRAMID_LEVELS);
  public static cx: Array<number> = Array(Constants.PYRAMID_LEVELS);
  public static cy: Array<number> = Array(Constants.PYRAMID_LEVELS);
  public static KInv: Array<Array<number>> = Array(Constants.PYRAMID_LEVELS);
  public static fxInv: Array<number> = Array(Constants.PYRAMID_LEVELS);
  public static fyInv: Array<number> = Array(Constants.PYRAMID_LEVELS);
  public static cxInv: Array<number> = Array(Constants.PYRAMID_LEVELS);
  public static cyInv: Array<number> = Array(Constants.PYRAMID_LEVELS);
  public static frameScore: number = 12
  /**
   * Sets camera matrix for all pyramid levels. Pass in parameters for level 0.
   */
  public static setK(fx: number, fy: number, cx: number, cy: number) {
    for (let level = 0; level < Constants.PYRAMID_LEVELS; level++) {
      if (level === 0) {
        Constants.fx[0] = fx;
        Constants.fy[0] = fy;
        Constants.cx[0] = cx;
        Constants.cy[0] = cy;
      } else {
        Constants.fx[level] = Constants.fx[level - 1] * 0.5;
        Constants.fy[level] = Constants.fy[level - 1] * 0.5;
        Constants.cx[level] = (Constants.cx[0] + 0.5) / (1 << level) - 0.5;
        Constants.cy[level] = (Constants.cy[0] + 0.5) / (1 << level) - 0.5;
      }
      Constants.K[level] = [
        Constants.fx[level], 0, Constants.cx[level],
        0, Constants.fy[level], Constants.cy[level],
        0, 0, 1
      ];
      Constants.KInv[level] = Vec.invert(Constants.K[level], 3);
      Constants.fxInv[level] = Constants.KInv[level][0];
      Constants.fyInv[level] = Constants.KInv[level][4];
      Constants.cxInv[level] = Constants.KInv[level][2];
      Constants.cyInv[level] = Constants.KInv[level][5];
    }
  }
  public static writePointCloudToFile(keyframe: Frame): void {
    console.log("WRITING KF " + keyframe.id);
    const posData = Constants.getPointCloud(keyframe, 0)
    const cameraPoints: Array<number>[] = Constants.generateCameraPosePoints(keyframe);
    const allPoints: Array<number>[] = [...cameraPoints, ...posData]
    let header: string = "ply\n format ascii 1.0\n element vertex " + allPoints.length + "\n property float x\n"
      + "property float y\n property float z\n end_header\n";
    for (let i = 0; i < allPoints.length; i++)
      header += " " + allPoints[i][0] + " " + allPoints[i][1] + " " + allPoints[i][2] + " \n";
    let link = document.createElement('a');
    link.download = 'graphPOINTCLOUD-' + keyframe.id + '.ply';
    let blob = new Blob([header], { type: 'text/plain' });
    link.href = window.URL.createObjectURL(blob);
    link.click();
    keyframe.clearData()
  }

  static getPointCloud(keyframe: Frame, level: number): Array<number>[] {
    const posData: Array<number>[] = [];
    const scaledTH: number = Constants.scaledDepthVarTH;
    const absTH: number = Constants.absDepthVarTH;
    for (let i = 0; i < keyframe.posDataLvl[level].length; i++) {
      // Get idepth, variance
      const idepth: number = keyframe.colorAndVarData[level][i][0];
      const var1: number = keyframe.colorAndVarData[level][i][1];
      const depth: number = 1 / idepth;
      let depth4: number = depth * depth * depth * depth;
      // Skip if depth/variance is not valid
      if (var1 * depth4 > scaledTH || var1 * depth4 > absTH)
        continue;
      posData.push(keyframe.camToWorld.mulFloat(keyframe.posDataLvl[level][i]));
    }
    return posData
  }

  static generateCameraPosePoints(keyframe: Frame): Array<number>[] {
    let cameraPose: Array<SE3> = keyframe.trackedOnPoses;
    let cameraPoints: Array<number>[] = Array();
    for (let i = 0; i < cameraPose.length; i++) {
      let pt = cameraPose[i].getTranslation();
      pt = keyframe.camToWorld.mulFloat(pt);
      let point: Array<number> = [...pt, 255, 0, 0];
      cameraPoints.push(point);
    }
    return cameraPoints;
  }
}