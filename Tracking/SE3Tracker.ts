import { Constants } from "../Utils/Constants";
import { Frame } from "../DataStructures/Frame";
import { SE3 } from "../LieAlgebra/SE3";
import { Vec } from "../LieAlgebra/Vec";

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
  calculateResidualAndBuffersCount: number = 0;

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

    let lastResidual: number = 0;

    // For each pyramid level, coarse to fine
    for (let level = Constants.SE3TRACKING_MAX_LEVEL - 1; level >= Constants.SE3TRACKING_MIN_LEVEL; level -= 1) {
      const posData = referenceFrame.posDataLvl[level]
      const colorAndVarData = referenceFrame.colorAndVarDataLvl[level]
      this.#calculateResidualAndBuffers(posData, colorAndVarData, frame, refToFrame, level);

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
      let lastError: number = this.#calculateWeightsAndResidual(refToFrame);
      let LM_lambda: number = this.lambdaInitial[level];

      // For a maximum number of iterations
      for (let iteration = 0; iteration < this.maxItsPerLvl[level]; iteration++) {
        // Calculate/update LS
        let [A, b] = this.#calculateWarpUpdate();

        let incTry: number = 0;
        while (true) {
          incTry++;

          // Solve LS to get increment
          let inc: Float32Array = this.#calcIncrement(A, b, LM_lambda);
          // console.log(incTry + " : " + LM_lambda + inc);
          // Apply increment
          let newRefToFrame: SE3 = SE3.exp(inc);
          newRefToFrame.mulEq(refToFrame);

          // Re-evaluate residual
          this.#calculateResidualAndBuffers(posData, colorAndVarData, frame, newRefToFrame, level);

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
          let error: number = this.#calculateWeightsAndResidual(newRefToFrame);
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
    frame.thisToParent = frameToRef;
    frame.camToWorld = referenceFrame.camToWorld.mulSE3(frame.thisToParent);
    frame.kfID = referenceFrame.id;

    // Add frameToRef to reference frame
    referenceFrame.trackedOnPoses.push(frameToRef);

    console.log("Final frameToRef: " + SE3.ln(frameToRef));

    return frameToRef;
  }

  #calcIncrement(A: Float32Array, b: Float32Array, LM_lambda: number): Float32Array {
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
  #calculateResidualAndBuffers(posData: Array<Float32Array>, colorAndVarData: Array<Float32Array>, frame: Frame,
    frameToRefPose: SE3, level: number): number {

    this.calculateResidualAndBuffersCount++;

    let fx: number = Constants.fx[level];
    let fy: number = Constants.fy[level];
    let cx: number = Constants.cx[level];
    let cy: number = Constants.cy[level];

    // Get rotation, translation matrix
    let rotationMat: Float32Array = frameToRefPose.getRotationMatrix();
    let translationVec: Float32Array = frameToRefPose.getTranslation();
    let sumResUnweighted: number = 0;
    let goodCount: number = 0;
    let badCount: number = 0;
    let sumSignedRes: number = 0;
    // what?
    let usageCount: number = 0;
    this.warpedCount = 0;
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
      let interpolatedIntensity: number = Vec.interpolatedValue(frame.imageArrayLvl[level], u, v,
        frame.width(level));
      let interpolatedGradientX: number = Vec.interpolatedValue(frame.imageGradientXArrayLvl[level], u, v,
        frame.width(level));
      let interpolatedGradientY: number = Vec.interpolatedValue(frame.imageGradientYArrayLvl[level], u, v,
        frame.width(level));

      let residual: number = colorAndVarData[i][0] - interpolatedIntensity;
      let squaredResidual: number = residual * residual;

      // Set buffers
      this.bufWarpedResidual[this.warpedCount] = residual;

      this.bufWarpedDx[this.warpedCount] = (fx * interpolatedGradientX);
      this.bufWarpedDy[this.warpedCount] = (fy * interpolatedGradientY);

      this.bufWarpedX[this.warpedCount] = warpedPoint[0];
      this.bufWarpedY[this.warpedCount] = warpedPoint[1];
      this.bufWarpedZ[this.warpedCount] = warpedPoint[2];
      this.bufInvDepth[this.warpedCount] = (1.0 / point[2]);
      this.bufInvDepthVariance[this.warpedCount] = colorAndVarData[i][1];

      // Increase warpCount
      this.warpedCount += 1;

      // Condition related to gradient and residual, to determine if to
      // use the residual from this pixel or not.
      let isGood: boolean = squaredResidual / (Constants.MAX_DIFF_CONSTANT
        + Constants.MAX_DIFF_GRAD_MULT * (interpolatedGradientX * interpolatedGradientX
          + interpolatedGradientY * interpolatedGradientY)) < 1;

      if (isGood) {
        sumResUnweighted += squaredResidual;
        sumSignedRes += residual;
        goodCount++;
      } else badCount++;

      // Change in depth
      let depthChange: number = (point[2] / warpedPoint[2]); // if depth becomes larger: pixel becomes
      // "smaller", hence count it less.
      // Pixels used?
      usageCount += depthChange < 1 ? depthChange : 1;

    }

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
  #calculateWeightsAndResidual(referenceToFrame: SE3): number {
    let [tx, ty, tz] = referenceToFrame.getTranslation();
    let sumRes: number = 0;

    for (let i = 0; i < this.warpedCount; i++) {
      let px: number = this.bufWarpedX[i]; // x'
      let py: number = this.bufWarpedY[i]; // y'
      let pz: number = this.bufWarpedZ[i]; // z'
      let d: number = this.bufInvDepth[i]; // d
      let rp: number = this.bufWarpedResidual[i]; // r_p
      let gx: number = this.bufWarpedDx[i]; // \delta_x I
      let gy: number = this.bufWarpedDy[i]; // \delta_y I
      let s: number = this.varWeight * this.bufInvDepthVariance[i]; // \sigma_d^2

      // calc dw/dd (first 2 components):
      let g0: number = (tx * pz - tz * px) / (pz * pz * d);
      let g1: number = (ty * pz - tz * py) / (pz * pz * d);

      // calc w_p
      let drpdd: number = gx * g0 + gy * g1; // ommitting the minus
      let w_p: number = 1.0 / ((this.cameraPixelNoise2) + s * drpdd * drpdd);
      let weighted_rp: number = Math.abs(rp * Math.sqrt(w_p));
      let wh: number = Math.abs(weighted_rp < (this.huberD / 2) ? 1 : (this.huberD / 2) / weighted_rp);
      sumRes += wh * w_p * rp * rp;

      // Set weight into buffer
      this.bufWeightP[i] = wh * w_p;
    }

    return sumRes / this.warpedCount;
  }

  #calculateWarpUpdate(): [Float32Array, Float32Array] {
    const A = new Float32Array(36);
    const b = new Float32Array(6)
    //let error = 0;
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
      Vec.matrixAdd(A, Vec.matrixMul(Vec.vecTransMul(v, v), this.bufWeightP[i]));
      Vec.vecMinus(b, (Vec.scalarMul2(v, r * this.bufWeightP[i])));
      // error += r * r * this.bufWeightP[i];
    }
    // Solve LS
    Vec.matrixDiv(A, this.warpedCount);
    Vec.vectorDiv0(b, this.warpedCount);
    Vec.vecNeg(b);
    // error /= this.warpedCount;
    return [A, b]
  }
}
