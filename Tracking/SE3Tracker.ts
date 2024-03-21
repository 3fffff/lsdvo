import { Constants } from "../Utils/Constants";
import { Frame } from "../DataStructures/Frame";
import { SE3 } from "../LieAlgebra/SE3";
import { Vec } from "../LieAlgebra/Vec";
import { SIM3 } from "../LieAlgebra/SIM3";

export class SE3Tracker {
  // Settings variables
  maxItsPerLvl: Float32Array = new Float32Array([5, 20, 50, 100, 100]);
  lambdaInitial: Float32Array;
  convergenceEps: Float32Array;
  varWeight: number = 1.0;
  huberD: number = 3.0;
  cameraPixelNoise2: number = 4 * 4;

  lambdaSuccessFac: number = 0.5;
  lambdaFailFac: number = 2.0;
  stepSizeMin: Float32Array = new Float32Array([1e-8, 1e-8, 1e-8, 1e-8, 1e-8]);

  // Variables set when tracking
  warpedCount: number = 0; // Number of pixels warped into new image bounds

  public pointUsage: number = 0;
  lastGoodCount: number = 0;
  lastMeanRes: number = 0;
  lastBadCount: number = 0;

  public trackingWasGood: boolean = false;
  public diverged: boolean = false;

  // Buffers for holding data of pixels warped into new image bounds
  // Set in calculateResidualAndBuffers()
  // Maximum size of buffers is width(0)*height(0)
  // Current size is warpedCount
  bufWarpedResidual: Float32Array;
  bufWarpedDx: Float32Array;
  bufWarpedDy: Float32Array;
  bufWarpedX: Float32Array;
  bufWarpedY: Float32Array;
  bufWarpedZ: Float32Array;
  bufInvDepth: Float32Array;
  bufInvDepthVariance: Float32Array;
  bufWeightP: Float32Array;

  constructor(width: number, height: number) {
    // Set lambdaInitial values to 0
    this.lambdaInitial = new Float32Array(Constants.PYRAMID_LEVELS);
    // Set convergence epsilon
    this.convergenceEps = new Float32Array(Constants.PYRAMID_LEVELS);
    for (let i: number = 0; i < this.convergenceEps.length; i++)
      this.convergenceEps[i] = 0.999
    // Create buffer arrays
    let size: number = width * height;
    this.bufWarpedResidual = new Float32Array(size);
    this.bufWarpedDx = new Float32Array(size);
    this.bufWarpedDy = new Float32Array(size);
    this.bufWarpedX = new Float32Array(size);
    this.bufWarpedY = new Float32Array(size);
    this.bufWarpedZ = new Float32Array(size);
    this.bufInvDepth = new Float32Array(size);
    this.bufInvDepthVariance = new Float32Array(size);
    this.bufWeightP = new Float32Array(size);
  }

  /**
   * Estimates the pose between a reference frame and a frame.
   */
  public trackFrame(referenceFrame: Frame, frame: Frame, frameToRefInitialEstimate: SE3): SE3 {

    this.diverged = false;
    this.trackingWasGood = true;

    // Initial estimate
    let refToFrame: SE3 = SE3.inverse(frameToRefInitialEstimate);
    console.log("Init frameToRefInitialEstimate: " + SE3.ln(frameToRefInitialEstimate));

    console.log("Init refToFrame: " + (SE3.ln(refToFrame)));

    // LS
    let ls: LGS6 = new LGS6();

    let lastResidual: number = 0;

    // For each pyramid level, coarse to fine
    for (let level = Constants.SE3TRACKING_MAX_LEVEL - 1; level >= Constants.SE3TRACKING_MIN_LEVEL; level -= 1) {
      const [posData, colorAndVarData] = referenceFrame.createPointCloud(level);
      this.calculateResidualAndBuffers(posData, colorAndVarData, frame, refToFrame, level);

      // Diverge when amount of pixels successfully warped into new frame < some
      // amount
      if (this.warpedCount < Constants.MIN_GOODPERALL_PIXEL_ABSMIN * frame.width(level) * frame.height(level)) {
        // Diverge
        this.diverged = true;
        this.trackingWasGood = false;
        console.error("Diverged.(1)")
        return new SE3()
      }

      // Weighted SSD
      let lastError: number = this.calculateWeightsAndResidual(refToFrame);
      let LM_lambda: number = this.lambdaInitial[level];

      // For a maximum number of iterations
      for (let iteration = 0; iteration < this.maxItsPerLvl[level]; iteration++) {
        // Calculate/update LS
        this.calculateWarpUpdate(ls);

        let incTry: number = 0;
        while (true) {
          incTry++;

          // Solve LS to get increment
          let inc: Float32Array = this.calcIncrement(ls, LM_lambda);
          // console.log(incTry + " : " + LM_lambda + inc);
          // Apply increment
          let newRefToFrame: SE3 = SE3.exp(inc);
          newRefToFrame.mulEq(refToFrame);

          // Re-evaluate residual
          this.calculateResidualAndBuffers(posData, colorAndVarData, frame, newRefToFrame, level);

          // Check for divergence
          if (this.warpedCount < Constants.MIN_GOODPERALL_PIXEL_ABSMIN * frame.width(level)
            * frame.height(level)) {
            console.log("warpedCount: " + this.warpedCount);
            this.diverged = true;
            this.trackingWasGood = false;
            console.error("Diverged.(2)")
            return new SE3()
          }

          // Calculate weighted residual/error
          let error: number = this.calculateWeightsAndResidual(newRefToFrame);
          if (error < lastError) {
            // Accept increment
            refToFrame = newRefToFrame;

            // Check for convergence
            if (error / lastError > this.convergenceEps[level]) {
              // Stop iteration
              iteration = this.maxItsPerLvl[level];
            }
            lastError = error;
            lastResidual = error;

            // Update lambda
            if (LM_lambda <= 0.2) {
              LM_lambda = 0;
            } else {
              LM_lambda *= this.lambdaSuccessFac;
            }
            break;
          } else {
            let incVecDot: number = Vec.dot(inc, inc);
            if (!(incVecDot > this.stepSizeMin[level])) {
              // Stop iteration
              iteration = this.maxItsPerLvl[level];
              // console.log("Step size below min");
              break;
            }

            // Update lambda
            if (LM_lambda == 0) LM_lambda = 0.2;
            else LM_lambda *= Math.pow(this.lambdaFailFac, incTry);
          }
        }
      }
    }

    this.trackingWasGood = !this.diverged
      && this.lastGoodCount / (frame.width(Constants.SE3TRACKING_MIN_LEVEL)
        * frame.height(Constants.SE3TRACKING_MIN_LEVEL)) > Constants.MIN_GOODPERALL_PIXEL
      && this.lastGoodCount / (this.lastGoodCount + this.lastBadCount) > Constants.MIN_GOODPERGOODBAD_PIXEL;

    if (this.trackingWasGood) referenceFrame.numFramesTrackedOnThis++;

    let frameToRef: SE3 = SE3.inverse(refToFrame);

    frame.initialTrackedResidual = lastResidual / this.pointUsage;
    frame.thisToParent = new SIM3(frameToRef, 1.0);
    frame.camToWorld = referenceFrame.camToWorld.mul(frame.thisToParent);
    frame.kfID = referenceFrame.id;

    // Add frameToRef to reference frame
    referenceFrame.trackedOnPoses.push(frameToRef);

    console.log("Final frameToRef: " + SE3.ln(frameToRef));

    return frameToRef;
  }

  calcIncrement(ls: LGS6, LM_lambda: number): Float32Array {
    let b: Float32Array = Vec.vecNeg(ls.b);
    let A: Float32Array = ls.A;
    for (let i = 0; i < b.length; i++)
      A[i * b.length + i] = A[i * b.length + i] * (1 + LM_lambda); // A(i,i) *= 1+LM_lambda;
    return Vec.solveSystem(A, b);
  }

  /**
   * Calculate residual and buffers
   * 
   * @return sum of un-weighted residuals, divided by good pixel count.
   *
   */
  calculateResidualAndBuffers(posData: Array<Float32Array>, colorAndVarData: Array<Float32Array>, frame: Frame,
    frameToRefPose: SE3, level: number): number {

    const fx: number = Constants.fx[level];
    const fy: number = Constants.fy[level];
    const cx: number = Constants.cx[level];
    const cy: number = Constants.cy[level];

    // Get rotation, translation matrix
    const rotationMat: Float32Array = frameToRefPose.getRotationMatrix();
    const translationVec: Float32Array = frameToRefPose.getTranslation();
    let sumResUnweighted: number = 0;
    let goodCount: number = 0;
    let badCount: number = 0;
    let sumSignedRes: number = 0;
    // what?
    let usageCount: number = 0;
    let warpInd = 0;
    let numValidPoints: number = 0;
    // int inImage = 0;
    // For each point in point cloud
    for (let i = 0; i < posData.length; i++) {
      // 3D position
      let point: Float32Array = posData[i];

      // Skip if point is not valid
      if (!point) continue;
      else numValidPoints++;

      // Warp to 2D image by estimate
      let warpedPoint: Float32Array = Vec.vecAdd2(Vec.matVecMultiplySqr(rotationMat, point, 3), translationVec);

      // Image points
      let u: number = (warpedPoint[0] / warpedPoint[2]) * fx + cx;
      let v: number = (warpedPoint[1] / warpedPoint[2]) * fy + cy;

      // Check image points within bounds
      if (!(u > 1 && v > 1 && u < frame.width(level) - 2 && v < frame.height(level) - 2)) {
        // Skip this pixel
        continue;
      }

      // Interpolated intensity, gradient X,Y.
      const interpolatedIntensity: number = Vec.interpolatedValue(frame.imageArrayLvl[level], u, v,
        frame.width(level));
      const interpolatedGradientX: number = Vec.interpolatedValue(frame.imageGradientXArrayLvl[level], u, v,
        frame.width(level));
      const interpolatedGradientY: number = Vec.interpolatedValue(frame.imageGradientYArrayLvl[level], u, v,
        frame.width(level));

      const residual: number = colorAndVarData[i][0] - interpolatedIntensity;
      const squaredResidual: number = residual * residual;

      // Set buffers
      this.bufWarpedResidual[warpInd] = residual;

      this.bufWarpedDx[warpInd] = (fx * interpolatedGradientX);
      this.bufWarpedDy[warpInd] = (fy * interpolatedGradientY);

      this.bufWarpedX[warpInd] = warpedPoint[0];
      this.bufWarpedY[warpInd] = warpedPoint[1];
      this.bufWarpedZ[warpInd] = warpedPoint[2];
      this.bufInvDepth[warpInd] = (1.0 / point[2]);
      this.bufInvDepthVariance[warpInd] = colorAndVarData[i][1];

      // Increase warpCount
      warpInd++;

      // Condition related to gradient and residual, to determine if to
      // use the residual from this pixel or not.
      const isGood: boolean = squaredResidual / (Constants.MAX_DIFF_CONSTANT
        + Constants.MAX_DIFF_GRAD_MULT * (interpolatedGradientX * interpolatedGradientX
          + interpolatedGradientY * interpolatedGradientY)) < 1;

      if (isGood) {
        sumResUnweighted += squaredResidual;
        sumSignedRes += residual;
        goodCount++;
      } else badCount++;

      // Change in depth
      const depthChange: number = (point[2] / warpedPoint[2]); // if depth becomes larger: pixel becomes
      // "smaller", hence count it less.
      // Pixels used?
      usageCount += depthChange < 1 ? depthChange : 1;

    }
    this.warpedCount = warpInd;
    this.pointUsage = usageCount / numValidPoints;
    this.lastGoodCount = goodCount;
    this.lastBadCount = badCount;
    this.lastMeanRes = sumSignedRes / goodCount;
    return sumResUnweighted / goodCount;
  }

  /**
   * calcWeightsAndResidual
   *
   * @return sum of weighted residuals divided by warpedCount
   */
  calculateWeightsAndResidual(referenceToFrame: SE3): number {
    const [tx, ty, tz] = referenceToFrame.getTranslation();
    let sumRes: number = 0;

    for (let i = 0; i < this.warpedCount; i++) {
      const px: number = this.bufWarpedX[i]; // x'
      const py: number = this.bufWarpedY[i]; // y'
      const pz: number = this.bufWarpedZ[i]; // z'
      const d: number = this.bufInvDepth[i]; // d
      const rp: number = this.bufWarpedResidual[i]; // r_p
      const gx: number = this.bufWarpedDx[i]; // \delta_x I
      const gy: number = this.bufWarpedDy[i]; // \delta_y I
      const s: number = this.varWeight * this.bufInvDepthVariance[i]; // \sigma_d^2

      // calc dw/dd (first 2 components):
      const g0: number = (tx * pz - tz * px) / (pz * pz * d);
      const g1: number = (ty * pz - tz * py) / (pz * pz * d);

      // calc w_p
      const drpdd: number = gx * g0 + gy * g1; // ommitting the minus
      const w_p: number = 1.0 / ((this.cameraPixelNoise2) + s * drpdd * drpdd);
      const weighted_rp: number = Math.abs(rp * Math.sqrt(w_p));
      const wh: number = Math.abs(weighted_rp < (this.huberD / 2) ? 1 : (this.huberD / 2) / weighted_rp);
      sumRes += wh * w_p * rp * rp;

      // Set weight into buffer
      this.bufWeightP[i] = wh * w_p;
    }

    return sumRes / this.warpedCount;
  }

  /**
   * calculateWarpUpdate
   */
  calculateWarpUpdate(ls: LGS6): void {
    ls.initialize();
    // For each warped pixel
    for (let i = 0; i < this.warpedCount; i++) {
      // x,y,z
      let px: number = this.bufWarpedX[i];
      let py: number = this.bufWarpedY[i];
      let pz: number = this.bufWarpedZ[i];
      // Residual
      let r: number = this.bufWarpedResidual[i];
      // Gradient
      let gx: number = this.bufWarpedDx[i];
      let gy: number = this.bufWarpedDy[i];

      // inverse depth
      let z: number = 1.0 / pz;
      let z_sqr: number = 1.0 / (pz * pz);

      // Vector6
      let v: Float32Array = new Float32Array([z * gx, z * gy, (-px * z_sqr) * gx + (-py * z_sqr) * gy,
      (-px * py * z_sqr) * gx + (-(1.0 + py * py * z_sqr)) * gy,
      (1.0 + px * px * z_sqr) * gx + (px * py * z_sqr) * gy, (-py * z) * gx + (px * z) * gy]);

      // Integrate into A and b
      ls.update(v, r, this.bufWeightP[i]);
    }
    // Solve LS
    ls.finish();
  }
}

class LGS6 {
  // 6x6 matrix
  A: Float32Array;
  // 6x1 vector
  b: Float32Array;
  error: number = 0;
  numConstraints: number = 0;
  initialize() {
    this.A = new Float32Array(36);
    this.b = new Float32Array(6);
    this.error = 0;
    this.numConstraints = 0
  }

  // J is a vec6
  update(J: Float32Array, res: number, weight: number): void {
    this.A = Vec.matrixAdd(this.A, Vec.matrixMul(Vec.vecT(J, J), weight));
    this.b = Vec.vecMinus2(this.b, (Vec.scalarMult2(J, res * weight)));
    this.error += res * res * weight;
    this.numConstraints += 1;
  }

  finish(): void {
    this.A = Vec.matrixDiv(this.A, this.numConstraints);
    this.b = Vec.scalarDel(this.b, this.numConstraints);
    this.error /= this.numConstraints;
  }
}
