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
  public static VALIDITY_COUNTER_MAX: number = 0.5;
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
  public static K: Array<Float32Array> = Array(Constants.PYRAMID_LEVELS);
  public static fx: Float32Array = new Float32Array(Constants.PYRAMID_LEVELS);
  public static fy: Float32Array = new Float32Array(Constants.PYRAMID_LEVELS);
  public static cx: Float32Array = new Float32Array(Constants.PYRAMID_LEVELS);
  public static cy: Float32Array = new Float32Array(Constants.PYRAMID_LEVELS);
  public static KInv: Array<Float32Array> = Array(Constants.PYRAMID_LEVELS);
  public static fxInv: Float32Array = new Float32Array(Constants.PYRAMID_LEVELS);
  public static fyInv: Float32Array = new Float32Array(Constants.PYRAMID_LEVELS);
  public static cxInv: Float32Array = new Float32Array(Constants.PYRAMID_LEVELS);
  public static cyInv: Float32Array = new Float32Array(Constants.PYRAMID_LEVELS);
  public static frameScore: number = 12
  /**
   * Sets camera matrix for all pyramid levels. Pass in parameters for level 0.
   */
  public static setK(fx: number, fy: number, cx: number, cy: number) {
    Constants.fx[0] = fx;
    Constants.fy[0] = fy;
    Constants.cx[0] = cx;
    Constants.cy[0] = cy;
    Constants.K.splice(0, 0, new Float32Array([Constants.fx[0], 0, Constants.cx[0], 0, Constants.fy[0], Constants.cy[0], 0, 0, 1]));
    Constants.KInv.splice(0, 0, Vec.invert(Constants.K[0], 3));
    Constants.fxInv[0] = Constants.KInv[0][0 * 3 + 0];
    Constants.fyInv[0] = Constants.KInv[0][1 * 3 + 1];
    Constants.cxInv[0] = Constants.KInv[0][0 * 3 + 2];
    Constants.cyInv[0] = Constants.KInv[0][1 * 3 + 2];
    for (let level: number = 1; level < Constants.PYRAMID_LEVELS; level++) {
      Constants.fx[level] = Constants.fx[level - 1] * 0.5;
      Constants.fy[level] = Constants.fy[level - 1] * 0.5;
      Constants.cx[level] = (Constants.cx[0] + 0.5) / (1 << level) - 0.5;
      Constants.cy[level] = (Constants.cy[0] + 0.5) / (1 << level) - 0.5;
      Constants.K.splice(level, 0, new Float32Array([Constants.fx[level], 0, Constants.cx[level], 0, Constants.fy[level], Constants.cy[level], 0, 0, 1]));
      Constants.KInv.splice(level, 0, Vec.invert(Constants.K[level], 3));
      Constants.fxInv[level] = Constants.KInv[level][0 * 3 + 0];
      Constants.fyInv[level] = Constants.KInv[level][1 * 3 + 1];
      Constants.cxInv[level] = Constants.KInv[level][0 * 3 + 2];
      Constants.cyInv[level] = Constants.KInv[level][1 * 3 + 2];
    }
  }
  public static writePointCloudToFile(keyframe: Frame): void {
    console.log("WRITING KF " + keyframe.id);
    const posData = Constants.getPointCloud(keyframe, 0)
    const cameraPoints: Float32Array[] = Constants.generateCameraPosePoints(keyframe);
    const allPoints: Float32Array[] = [...cameraPoints, ...posData].filter(p => p !== undefined && p.length > 0 && !isNaN(p[0]));
    // Write to file
    let header: string = "ply\n" + "format ascii 1.0\n" + "element vertex " + allPoints.length + "\n" + "property float x\n"
      + "property float y\n" + "property float z\n" + "end_header\n";
    for (let i = 0; i < allPoints.length; i++)
      header += " " + allPoints[i][0] + " " + allPoints[i][1] + " " + allPoints[i][2] + " \n";
    let link = document.createElement('a');
    link.download = 'graphPOINTCLOUD-' + keyframe.id + '.ply';
    let blob = new Blob([header], { type: 'text/plain' });
    link.href = window.URL.createObjectURL(blob);
    link.click();
    keyframe.clearData()
  }

  static getPointCloud(keyframe: Frame, level: number): Float32Array[] {
    const width: number = keyframe.width(level);
    const height: number = keyframe.height(level);
    const inverseDepth: Float32Array = keyframe.inverseDepthLvl[level];
    const inverseDepthVariance: Float32Array = keyframe.inverseDepthVarianceLvl[level];
    const posData: Float32Array[] = Array(width * height);
    const scaledTH: number = Constants.scaledDepthVarTH;
    const absTH: number = Constants.absDepthVarTH;
    for (let x = 1; x < width - 1; x++) {
      for (let y = 1; y < height - 1; y++) {
        // Index to reference pixel
        const idx: number = x + y * width;
        // Get idepth, variance
        const idepth: number = inverseDepth[idx];
        const var1: number = inverseDepthVariance[idx];
        const depth: number = 1 / idepth;
        let depth4: number = depth * depth;
        depth4 *= depth4;
        // Skip if depth/variance is not valid
        if (idepth == 0 || var1 <= 0 || var1 * depth4 > scaledTH || var1 * depth4 > absTH)
          continue;
        // Transform
        posData[idx] = keyframe.camToWorld.mulFloat(keyframe.posDataLvl[level][idx]);
      }
    }
    return posData
  }

  static generateCameraPosePoints(keyframe: Frame): Float32Array[] {
    let cameraPose: Array<SE3> = keyframe.trackedOnPoses;
    let cameraPoints: Float32Array[] = Array();
    for (let i = 0; i < cameraPose.length; i++) {
      let pt = cameraPose[i].getTranslation();
      pt = keyframe.camToWorld.mulFloat(pt);
      let point: Float32Array = new Float32Array([...pt, 255, 0, 0]);
      cameraPoints.push(point);
    }
    return cameraPoints;
  }
}