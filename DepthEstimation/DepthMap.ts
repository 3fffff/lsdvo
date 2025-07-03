import { DepthMapPixelHypothesis } from "./DepthMapPixelHypothesis";
import { Frame } from "../DataStructures/Frame";
import { Constants } from "../Utils/Constants";
import { Vec } from "../LieAlgebra/Vec";
import { SE3 } from "../LieAlgebra/SE3";
import { TestDepth } from "./TestDepth";

/**
 * Keeps a detailed depth map (consisting of DepthMapPixelHypothesis) and does
 * stereo comparisons and regularization to update it.
 */
export class DepthMap {

  /** ============== Depth Variance Handling ======================= */
  static SUCC_VAR_INC_FAC: number = (1.01); // before an
  // ekf-update, the variance is increased by this factor.
  static FAIL_VAR_INC_FAC: number = 1.1; // after a failed stereo observation, the
  // variance is increased by this factor.
  static MAX_VAR: number = (0.5 * 0.5); // initial variance on
  // creation - if variance becomeslarger than this, hypothesis is removed.
  static VAR_RANDOM_INIT_INITIAL: number = (0.5 * DepthMap.MAX_VAR); // initial
  // variance for Random Initialization

  width: number;
  height: number;

  otherDepthMap: DepthMapPixelHypothesis[];
  currentDepthMap: DepthMapPixelHypothesis[];
  validityIntegralBuffer: Float32Array;

  activeKeyFrame: Frame;

  //oldest_referenceFrame: Frame;
  referenceFrame: Frame;
  //referenceFrameByID: Array<Frame>;
  //referenceFrameByID_offset: number;

  // Camera matrix
  fx: number; fy: number; cx: number; cy: number;
  fxi: number; fyi: number; cxi: number; cyi: number;
  K_otherToThis_R: Float32Array;
  K_otherToThis_t: Float32Array;
  otherToThis_t: Float32Array;
  thisToOther_R: Float32Array;
  thisToOther_t: Float32Array;
  debugDepth: TestDepth | null;

  constructor(w: number, h: number, debug: boolean) {
    this.width = w;
    this.height = h;

    this.debugDepth = debug ? new TestDepth(this) : null

    //this.activeKeyFrame = null;
    this.otherDepthMap = Array(this.width * this.height);
    this.currentDepthMap = Array(this.width * this.height);

    this.validityIntegralBuffer = new Float32Array(this.width * this.height);

    this.fx = Constants.K[0][0 * 3 + 0];
    this.fy = Constants.K[0][1 * 3 + 1];
    this.cx = Constants.K[0][0 * 3 + 2];
    this.cy = Constants.K[0][1 * 3 + 2];

    this.fxi = Constants.KInv[0][0 * 3 + 0];
    this.fyi = Constants.KInv[0][1 * 3 + 1];
    this.cxi = Constants.KInv[0][0 * 3 + 2];
    this.cyi = Constants.KInv[0][1 * 3 + 2];

    for (let i = 0; i < this.width * this.height; i++) {
      if (this.otherDepthMap[i] == null)
        this.otherDepthMap[i] = new DepthMapPixelHypothesis();
      // Default hypothesis for all pixels
      if (this.currentDepthMap[i] == null)
        this.currentDepthMap[i] = new DepthMapPixelHypothesis();
    }
  }


  public isValid(): boolean {
    return this.activeKeyFrame != null;
  }

  public initializeRandomly(newFrame: Frame): void {
    this.activeKeyFrame = newFrame;

    let maxGradients: Float32Array = newFrame.imageGradientMaxArrayLvl[0];

    let goodGrad: number = 0;

    // For each pixel
    for (let y = 1; y < this.height - 1; y++) {
      for (let x = 1; x < this.width - 1; x++) {

        // For pixels with significant gradient
        if (maxGradients[x + y * this.width] > Constants.MIN_ABS_GRAD_CREATE) {
          // TODO: Get random idepth
          let idepth: number = 0.5 + 1.0 * (Math.random() * (100001) / 100000.0);

          // Set fixed initial depth
          // let idepth = 0.5f + 1.0f * 0.5f;

          // Set hypothesis, random idepth and initial variance.
          this.currentDepthMap[x + y * this.width] = new DepthMapPixelHypothesis(idepth, idepth,
            Constants.VAR_RANDOM_INIT_INITIAL, Constants.VAR_RANDOM_INIT_INITIAL, 20);
          goodGrad++;
        } else {
          // Mark as invalid
          this.currentDepthMap[x + y * this.width].isValid = false;
          this.currentDepthMap[x + y * this.width].blacklisted = 0;
        }
      }
    }
    console.log("Good grad: " + goodGrad);
    // Set depth hypothesis depth values to keyframe
    this.activeKeyFrame.setDepth(this.currentDepthMap);
  }

  /**
 * Updates depth map with observations from deque of frames
 */
  public updateKeyframe(frame: Frame) {
    this.isValid()

    // Get oldest/newest frames
    //  this.oldest_referenceFrame = referenceFrames[referenceFrames.length - 1];
    this.referenceFrame = frame

   // this.referenceFrameByID = [];
   // this.referenceFrameByID_offset = this.oldest_referenceFrame.id;

    // For each frame
    // console.log("Updating keyframe " + activeKeyFrame.id + " with " +
    // frame.id + ".");

    // Checks that tracking parent is valid
    //		assert (frame.id != 0);
    if (frame.kfID != this.activeKeyFrame.id) {
      console.log(
        "WARNING: updating frame %d with %d," + " which was tracked on a different frame (%d)."
        + "\nWhile this should work, it is not recommended.",
        this.activeKeyFrame.id, frame.id, frame.kfID);
    }

    let refToKf: SE3;
    // Get SIM3 from frame to keyframe
    if (this.activeKeyFrame.id === 0) {
      refToKf = frame.thisToParent;
    } else {
      refToKf = this.activeKeyFrame.camToWorld.inverse().mulSE3(frame.camToWorld);
    }

    // prepare frame for stereo with keyframe, SE3, K, level
    //frame.prepareForStereoWith(refToKf);

    let otherToThis: SE3 = refToKf.inverse();

    this.K_otherToThis_R = Vec.multMatrix(Constants.K[0], otherToThis.getRotationMatrix(), 3, 3, 3, 3);
    this.otherToThis_t = otherToThis.getTranslation();
    this.K_otherToThis_t = Vec.matVecMultiplySqr(Constants.K[0], this.otherToThis_t, 3);

    this.thisToOther_t = refToKf.getTranslation();
    this.thisToOther_R = refToKf.getRotationMatrix();

    // *** OBSERVE DEPTH HERE
    this.observeDepth();

    // Regularize, fill holes?
    this.regularizeDepthMapFillHoles();

    // Regularize
    this.regularizeDepthMap(false, Constants.VAL_SUM_MIN_FOR_KEEP);

    // Update depth in keyframe
    if (!this.activeKeyFrame.IDepthBeenSet) {
      // Update keyframe with updated depth?
      this.activeKeyFrame.setDepth(this.currentDepthMap);
    }

    this.activeKeyFrame.numMappedOnThis++;
  }

  observeDepth(): void {
    // TODO: make multithreaded
    this.observeDepthRow(3, this.height - 3);
  }

  /**
   * ObserveDepth for specified rows
   *
   * @param yMin
   * @param yMax
   */
  observeDepthRow(yMin: number, yMax: number): void {
    let keyFrameMaxGradBuf: Float32Array = this.activeKeyFrame.imageGradientMaxArrayLvl[0];

    // For each row assigned
    for (let y = yMin; y < yMax; y++) {
      // For x 3 to width-3
      for (let x = 3; x < this.width - 3; x++) {
        // For each pixel

        let idx: number = x + y * this.width;
        let target: DepthMapPixelHypothesis = this.currentDepthMap[idx];
        let hasHypothesis: boolean = target.isValid;

        // ======== 1. check absolute grad =========
        if (hasHypothesis && keyFrameMaxGradBuf[idx] < Constants.MIN_ABS_GRAD_DECREASE) {
          target.isValid = false;
          continue;
        }

        if (keyFrameMaxGradBuf[idx] < Constants.MIN_ABS_GRAD_CREATE
          || target.blacklisted < Constants.MIN_BLACKLIST) {
          continue;
        }

        // Gradient is significant, pixel not blacklisted
        if (!hasHypothesis) {
          // First time
          this.observeDepthCreate(x, y, idx);
        } else {
          // ***Observe depth***
          this.observeDepthUpdate(x, y, idx, keyFrameMaxGradBuf);
        }
      }
    }
  }
  observeDepthCreate(x: number, y: number, idx: number): boolean {
    let target: DepthMapPixelHypothesis = this.currentDepthMap[idx];

    // ???
    // What is activeKeyFrameIsReactivated?
    // Key frame was used before?
    let refFrame: Frame = this.referenceFrame;

    // Get epipolar line??
    let epx: number, epy: number;
    // x, y pixel coordinate, refFrame
    let epl: Float32Array | null = this.makeAndCheckEPL(x, y, refFrame);

    if (epl == null) {
      return false;
    } else {
      epx = epl[0];
      epy = epl[1];
    }

    let result_idepth: number = 0.0;
    let result_var: number = 0.0;
    let result_eplLength: number = 0.0;

    // Do line stereo, get error, ^ results
    let lineStereoResult: Float32Array = this.doLineStereo(x, y, epx, epy, 0.0, 1.0, 1.0 / Constants.MIN_DEPTH, refFrame,
      refFrame.imageArrayLvl[0], result_idepth, result_var, result_eplLength);

    let error: number = lineStereoResult[0];
    result_idepth = lineStereoResult[1];
    result_var = lineStereoResult[2];
    result_eplLength = lineStereoResult[3];

    if (error == -3 || error == -2) {
      target.blacklisted--;
    }

    if (error < 0 || result_var > Constants.MAX_VAR) {
      return false;
    }

    result_idepth = this.UNZERO(result_idepth);

    // add hypothesis
    // Set/change the hypothesis
    target = new DepthMapPixelHypothesis(result_idepth, result_var, Constants.VALIDITY_COUNTER_INITIAL_OBSERVE);

    return true;
  }

  observeDepthUpdate(x: number, y: number, idx: number, keyFrameMaxGradBuf: Float32Array): boolean {
    let target: DepthMapPixelHypothesis = this.currentDepthMap[idx];
    let refFrame: Frame = this.referenceFrame;

    // Get epipolar line
    let epx: number, epy: number;
    // x, y pixel coordinate, refFrame
    let epl: Float32Array | null = this.makeAndCheckEPL(x, y, refFrame);

    if (epl == null) {
      return false;
    } else {
      epx = epl[0];
      epy = epl[1];
    }

    // Limits for search?
    // which exact point to track, and where from.
    let sv: number = Math.sqrt(target.idepth_var_smoothed);
    let min_idepth: number = target.idepth_smoothed - sv * Constants.STEREO_EPL_VAR_FAC;
    let max_idepth: number = target.idepth_smoothed + sv * Constants.STEREO_EPL_VAR_FAC;
    if (min_idepth < 0)
      min_idepth = 0;
    if (max_idepth > 1 / Constants.MIN_DEPTH)
      max_idepth = 1 / Constants.MIN_DEPTH;

    let result_idepth: number = 0.0;
    let result_var: number = 0.0;
    let result_eplLength: number = 0.0;

    // Do stereo
    let lineStereoResult: Float32Array = this.doLineStereo(x, y, epx, epy, min_idepth, target.idepth_smoothed, max_idepth,
      refFrame, refFrame.imageArrayLvl[0], result_idepth, result_var, result_eplLength);

    let error: number = lineStereoResult[0];
    result_idepth = lineStereoResult[1];
    result_var = lineStereoResult[2];
    result_eplLength = lineStereoResult[3];

    let diff: number = result_idepth - target.idepth_smoothed;

    // if oob: (really out of bounds)
    if (error == -1) {
      // do nothing, pixel got oob, but is still in bounds in original. I
      // will want to try again.
      return false;
    } else if (error == -2) {
      // if just not good for stereo (e.g. some inf / nan occured; has
      // inconsistent minimum; ..)

      target.validity_counter -= Constants.VALIDITY_COUNTER_DEC;
      if (target.validity_counter < 0)
        target.validity_counter = 0;

      target.idepth_var *= DepthMap.FAIL_VAR_INC_FAC;
      if (target.idepth_var > Constants.MAX_VAR) {
        target.isValid = false;
        target.blacklisted--;
      }
      return false;
    } else if (error == -3) {
      // if not found (error too high)
      return false;
    } else if (error == -4) {
      return false;
    } else if (Constants.DIFF_FAC_OBSERVE * diff * diff > result_var + target.idepth_var_smoothed) {
      // if inconsistent
      target.idepth_var *= DepthMap.FAIL_VAR_INC_FAC;
      if (target.idepth_var > Constants.MAX_VAR)
        target.isValid = false;

      return false;
    } else {
      // one more successful observation!

      // do textbook ekf update:
      // increase var by a little (prediction-uncertainty)
      let id_var: number = target.idepth_var * DepthMap.SUCC_VAR_INC_FAC;

      // update var with observation
      let w: number = result_var / (result_var + id_var);
      let new_idepth: number = (1.0 - w) * result_idepth + w * target.idepth;

      target.idepth = this.UNZERO(new_idepth);

      // variance can only decrease from observation; never increase.
      id_var = id_var * w;
      if (id_var < target.idepth_var) {
        target.idepth_var = id_var;
      }

      // increase validity!
      target.validity_counter += Constants.VALIDITY_COUNTER_INC;
      let absGrad: number = keyFrameMaxGradBuf[idx];
      if (target.validity_counter > Constants.VALIDITY_COUNTER_MAX
        + absGrad * (Constants.VALIDITY_COUNTER_MAX_VARIABLE) / 255.0)
        target.validity_counter = (Constants.VALIDITY_COUNTER_MAX
          + absGrad * (Constants.VALIDITY_COUNTER_MAX_VARIABLE) / 255.0);

      // increase Skip!
      if (result_eplLength < Constants.MIN_EPL_LENGTH_CROP) {
        let inc: number = this.activeKeyFrame.numFramesTrackedOnThis / (this.activeKeyFrame.numMappedOnThis + 5);
        if (inc < 3)
          inc = 3;

        inc += Math.floor((result_eplLength * 10000) % 2);

        if (result_eplLength < 0.5 * Constants.MIN_EPL_LENGTH_CROP)
          inc *= 3;
      }

      return true;
    }
  }

  /**
 *
 * Return null if failed, return let[] {epx, epy} if found.
 */
  makeAndCheckEPL(x: number, y: number, ref: Frame): Float32Array | null {
    let idx: number = x + y * this.width;

    // ======= make epl ========
    // calculate the plane spanned by the two camera centers and the point
    // (x,y,1)
    // intersect it with the keyframe's image plane (at depth=1)
    let epx: number = -this.fx * this.thisToOther_t[0] + this.thisToOther_t[2] * (x - this.cx);
    let epy: number = -this.fy * this.thisToOther_t[1] + this.thisToOther_t[2] * (y - this.cy);

    if (isNaN(epx + epy)) {
      return null;
    }

    // ======== check epl length =========
    let eplLengthSquared: number = (epx * epx + epy * epy);
    if (eplLengthSquared < Constants.MIN_EPL_LENGTH_SQUARED) {
      return null;
    }

    // ===== check epl-grad magnitude ======

    let gx: number = this.activeKeyFrame.imageArrayLvl[0][idx + 1] - this.activeKeyFrame.imageArrayLvl[0][idx - 1];
    let gy: number = this.activeKeyFrame.imageArrayLvl[0][idx + this.width] - this.activeKeyFrame.imageArrayLvl[0][idx - this.width];
    let eplGradSquared: number = (gx * epx + gy * epy);
    eplGradSquared = eplGradSquared * eplGradSquared / eplLengthSquared; // square and norm with epl-length

    if (eplGradSquared < Constants.MIN_EPL_GRAD_SQUARED) {
      return null;
    }

    // ===== check epl-grad angle ======
    if (eplGradSquared / (gx * gx + gy * gy) < Constants.MIN_EPL_ANGLE_SQUARED) {
      return null;
    }

    // ===== DONE - return "normalized" epl =====
    let fac: number = (Constants.GRADIENT_SAMPLE_DIST / Math.sqrt(eplLengthSquared));

    let pepx: number = (epx * fac);
    let pepy: number = (epy * fac);

    return new Float32Array([pepx, pepy]);
  }

  // find pixel in image (do stereo along epipolar line).
  // mat: NEW image
  // KinvP: point in OLD image (Kinv * (u_old, v_old, 1)), projected
  // trafo: x_old = trafo * x_new; (from new to old image)
  // realVal: descriptor in OLD image.
  // returns: result_idepth : point depth in new camera's coordinate system
  // returns: result_u/v : point's coordinates in new camera's coordinate
  // system
  // returns: idepth_var: (approximated) measurement variance of inverse depth
  // of result_point_NEW
  // returns error if sucessful; -1 if out of bounds, -2 if not found.

  /**
   * Returns let[4] array, {error, result_idepth, result_var, result_eplLength}
   */
  doLineStereo(u: number, v: number, epxn: number, epyn: number, min_idepth: number, prior_idepth: number,
    max_idepth: number, referenceFrame: Frame, referenceFrameImage: Float32Array, result_idepth: number, result_var: number,
    result_eplLength: number): Float32Array {

    // calculate epipolar line start and end point in old image
    let KinvP: Float32Array = new Float32Array([this.fxi * u + this.cxi, this.fyi * v + this.cyi, 1.0]);
    let pInf: Float32Array = Vec.matVecMultiplySqr(this.K_otherToThis_R, KinvP, 3);
    let pReal: Float32Array = Vec.vecAdd2(Vec.vectorDiv(pInf, prior_idepth), this.K_otherToThis_t);

    let rescaleFactor: number = (pReal[2] * prior_idepth);

    // Start/end points of epipolar line on old image?
    let firstX: number = u - 2 * epxn * rescaleFactor;
    let firstY: number = v - 2 * epyn * rescaleFactor;
    let lastX: number = u + 2 * epxn * rescaleFactor;
    let lastY: number = v + 2 * epyn * rescaleFactor;
    // width - 2 and height - 2 comes from the one-sided gradient
    // calculation at the bottom
    if (firstX <= 0 || firstX >= this.width - 2 || firstY <= 0 || firstY >= this.height - 2 || lastX <= 0
      || lastX >= this.width - 2 || lastY <= 0 || lastY >= this.height - 2) {
      return new Float32Array([-1, result_idepth, result_var, result_eplLength]);
    }

    if (!(rescaleFactor > 0.7 && rescaleFactor < 1.4)) {
      return new Float32Array([-1, result_idepth, result_var, result_eplLength]);
    }

    let activeKeyFrameImageData: Float32Array = this.activeKeyFrame.imageArrayLvl[0];

    // calculate values to search for
    let realVal_p1: number = Vec.interpolatedValue(activeKeyFrameImageData, u + epxn * rescaleFactor,
      v + epyn * rescaleFactor, this.width);
    let realVal_m1: number = Vec.interpolatedValue(activeKeyFrameImageData, u - epxn * rescaleFactor,
      v - epyn * rescaleFactor, this.width);
    let realVal: number = Vec.interpolatedValue(activeKeyFrameImageData, u, v, this.width);
    let realVal_m2: number = Vec.interpolatedValue(activeKeyFrameImageData, u - 2 * epxn * rescaleFactor,
      v - 2 * epyn * rescaleFactor, this.width);
    let realVal_p2: number = Vec.interpolatedValue(activeKeyFrameImageData, u + 2 * epxn * rescaleFactor,
      v + 2 * epyn * rescaleFactor, this.width);

    let pClose: Float32Array = Vec.vecAdd2(pInf, Vec.scalarMul2(this.K_otherToThis_t, max_idepth));
    // if the assumed close-point lies behind the
    // image, have to change that.
    if (pClose[2] < 0.001) {
      max_idepth = ((0.001 - pInf[2]) / this.K_otherToThis_t[2]);
      pClose = Vec.vecAdd2(Vec.scalarMul2(this.K_otherToThis_t, max_idepth), pInf);
    }
    pClose = Vec.vectorDiv(pClose, pClose[2]); // pos in new image of point
    // (xy), assuming max_idepth

    let pFar: Float32Array = Vec.vecAdd2(pInf, Vec.scalarMul2(this.K_otherToThis_t, min_idepth));
    // if the assumed far-point lies behind the image or closter than the
    // near-point,
    // we moved past the Point it and should stop.
    if (pFar[2] < 0.001 || max_idepth < min_idepth) {
      return new Float32Array([-1, result_idepth, result_var, result_eplLength]);
    }
    pFar = Vec.vectorDiv(pFar, pFar[2]); // pos in new image of point (xy),
    // assuming min_idepth

    // check for nan due to eg division by zero.
    if (isNaN((pFar[0] + pClose[0]))) {
      return new Float32Array([-4, result_idepth, result_var, result_eplLength]);
    }

    // calculate increments in which we will step through the epipolar line.
    // they are sampleDist (or half sample dist) long
    let incx: number = (pClose[0] - pFar[0]);
    let incy: number = (pClose[1] - pFar[1]);
    let eplLength: number = Math.sqrt(incx * incx + incy * incy);
    if (!(eplLength > 0) || !isFinite(eplLength)) {
      return new Float32Array([-4, result_idepth, result_var, result_eplLength]);
    }

    if (eplLength > Constants.MAX_EPL_LENGTH_CROP) {
      pClose[0] = pFar[0] + incx * Constants.MAX_EPL_LENGTH_CROP / eplLength;
      pClose[1] = pFar[1] + incy * Constants.MAX_EPL_LENGTH_CROP / eplLength;
    }

    incx *= Constants.GRADIENT_SAMPLE_DIST / eplLength;
    incy *= Constants.GRADIENT_SAMPLE_DIST / eplLength;

    // extend one sample_dist to left & right.
    pFar[0] = pFar[0] - incx;
    pFar[1] = pFar[1] - incy;
    pClose[0] = pClose[0] + incx;
    pClose[1] = pClose[1] + incy;

    // make epl long enough (pad a little bit).
    if (eplLength < Constants.MIN_EPL_LENGTH_CROP) {
      let pad: number = (Constants.MIN_EPL_LENGTH_CROP - (eplLength)) / 2.0;
      pFar[0] = pFar[0] - incx * pad;
      pFar[1] = pFar[1] - incy * pad;

      pClose[0] = pClose[0] + incx * pad;
      pClose[1] = pClose[1] + incy * pad;
    }

    // if inf point is outside of image: skip pixel.
    if (pFar[0] <= Constants.SAMPLE_POINT_TO_BORDER || pFar[0] >= this.width - Constants.SAMPLE_POINT_TO_BORDER
      || pFar[1] <= Constants.SAMPLE_POINT_TO_BORDER
      || pFar[1] >= this.height - Constants.SAMPLE_POINT_TO_BORDER) {
      return new Float32Array([-1, result_idepth, result_var, result_eplLength]);
    }

    // if near point is outside: move inside, and test length again.
    if (pClose[0] <= Constants.SAMPLE_POINT_TO_BORDER || pClose[0] >= this.width - Constants.SAMPLE_POINT_TO_BORDER
      || pClose[1] <= Constants.SAMPLE_POINT_TO_BORDER
      || pClose[1] >= this.height - Constants.SAMPLE_POINT_TO_BORDER) {
      if (pClose[0] <= Constants.SAMPLE_POINT_TO_BORDER) {
        let toAdd: number = ((Constants.SAMPLE_POINT_TO_BORDER - pClose[0]) / incx);

        pClose[0] = pClose[0] + incx * toAdd;
        pClose[1] = pClose[1] + incy * toAdd;
      } else if (pClose[0] >= this.width - Constants.SAMPLE_POINT_TO_BORDER) {
        let toAdd: number = ((this.width - Constants.SAMPLE_POINT_TO_BORDER - pClose[0]) / incx);

        pClose[0] = pClose[0] + incx * toAdd;
        pClose[1] = pClose[1] + incy * toAdd;
      }

      if (pClose[1] <= Constants.SAMPLE_POINT_TO_BORDER) {
        let toAdd: number = ((Constants.SAMPLE_POINT_TO_BORDER - pClose[1]) / incy);

        pClose[0] = pClose[0] + incx * toAdd;
        pClose[1] = pClose[1] + incy * toAdd;
      } else if (pClose[1] >= this.height - Constants.SAMPLE_POINT_TO_BORDER) {
        let toAdd: number = ((this.height - Constants.SAMPLE_POINT_TO_BORDER - pClose[1]) / incy);

        pClose[0] = pClose[0] + incx * toAdd;
        pClose[1] = pClose[1] + incy * toAdd;
      }

      // get new epl length
      let fincx: number = (pClose[0] - pFar[0]);
      let fincy: number = (pClose[1] - pFar[1]);
      let newEplLength: number = Math.sqrt(fincx * fincx + fincy * fincy);

      // test again
      if (pClose[0] <= Constants.SAMPLE_POINT_TO_BORDER || pClose[0] >= this.width - Constants.SAMPLE_POINT_TO_BORDER
        || pClose[1] <= Constants.SAMPLE_POINT_TO_BORDER
        || pClose[1] >= this.height - Constants.SAMPLE_POINT_TO_BORDER || newEplLength < 8.0) {
        return new Float32Array([-1, result_idepth, result_var, result_eplLength]);
      }

    }

    // from here on:
    // - pInf: search start-point
    // - p0: search end-point
    // - incx, incy: search steps in pixel
    // - eplLength, min_idepth, max_idepth: determines search-resolution,
    // i.e. the result's variance.

    let cpx: number = pFar[0];
    let cpy: number = pFar[1];

    let val_cp_m2: number = Vec.interpolatedValue(referenceFrameImage, cpx - 2.0 * incx, cpy - 2.0 * incy, this.width);
    let val_cp_m1: number = Vec.interpolatedValue(referenceFrameImage, cpx - incx, cpy - incy, this.width);
    let val_cp: number = Vec.interpolatedValue(referenceFrameImage, cpx, cpy, this.width);
    let val_cp_p1: number = Vec.interpolatedValue(referenceFrameImage, cpx + incx, cpy + incy, this.width);
    let val_cp_p2: number;

    /*
     * Subsequent exact minimum is found the following way: - assuming lin.
     * interpolation, the gradient of Error at p1 (towards p2) is given by dE1 =
     * -2sum(e1*e1 - e1*e2) where e1 and e2 are summed over, and are the residuals
     * (not squared).
     *
     * - the gradient at p2 (coming from p1) is given by dE2 = +2sum(e2*e2 - e1*e2)
     *
     * - linear interpolation => gradient changes linearely; zero-crossing is hence
     * given by p1 + d*(p2-p1) with d = -dE1 / (-dE1 + dE2).
     *
     *
     *
     * => I for later exact min calculation, I need
     * sum(e_i*e_i),sum(e_{i-1}*e_{i-1}),sum(e_{i+1}*e_{i+1}) and sum(e_i * e_{i-1})
     * and sum(e_i * e_{i+1}), where i is the respective winning index.
     */

    // walk in equally sized steps, starting at depth=infinity.
    let loopCounter: number = 0;
    let best_match_x: number = -1;
    let best_match_y: number = -1;
    let best_match_err: number = 1e50;
    let second_best_match_err: number = 1e50;

    // best pre and post errors.
    let best_match_errPre: number = NaN;
    let best_match_errPost: number = NaN;
    let best_match_DiffErrPre: number = NaN;
    let best_match_DiffErrPost: number = NaN;
    let bestWasLastLoop: boolean = false;

    let eeLast: number = -1; // final error of last comp.

    // alternating intermediate vars
    let e1A: number = NaN, e1B: number = NaN;
    let e2A: number = NaN, e2B: number = NaN;
    let e3A: number = NaN, e3B: number = NaN;
    let e4A: number = NaN, e4B: number = NaN;
    let e5A: number = NaN, e5B: number = NaN;

    let loopCBest: number = -1, loopCSecond: number = -1;
    while (((incx < 0) == (cpx > pClose[0]) && (incy < 0) == (cpy > pClose[1])) || loopCounter == 0) {
      // interpolate one new point
      val_cp_p2 = Vec.interpolatedValue(referenceFrameImage, cpx + 2 * incx, cpy + 2 * incy, this.width);

      // hacky but fast way to get error and differential error: switch
      // buffer variables for last loop.
      let ee: number = 0;
      if (loopCounter % 2 == 0) {
        // calc error and accumulate sums.
        e1A = val_cp_p2 - realVal_p2;
        ee += e1A * e1A;
        e2A = val_cp_p1 - realVal_p1;
        ee += e2A * e2A;
        e3A = val_cp - realVal;
        ee += e3A * e3A;
        e4A = val_cp_m1 - realVal_m1;
        ee += e4A * e4A;
        e5A = val_cp_m2 - realVal_m2;
        ee += e5A * e5A;
      } else {
        // calc error and accumulate sums.
        e1B = val_cp_p2 - realVal_p2;
        ee += e1B * e1B;
        e2B = val_cp_p1 - realVal_p1;
        ee += e2B * e2B;
        e3B = val_cp - realVal;
        ee += e3B * e3B;
        e4B = val_cp_m1 - realVal_m1;
        ee += e4B * e4B;
        e5B = val_cp_m2 - realVal_m2;
        ee += e5B * e5B;
      }

      // do I have a new winner??
      // if so: set.
      if (ee < best_match_err) {
        // put to second-best
        second_best_match_err = best_match_err;
        loopCSecond = loopCBest;

        // set best.
        best_match_err = ee;
        loopCBest = loopCounter;

        best_match_errPre = eeLast;
        best_match_DiffErrPre = e1A * e1B + e2A * e2B + e3A * e3B + e4A * e4B + e5A * e5B;
        best_match_errPost = -1;
        best_match_DiffErrPost = -1;

        best_match_x = cpx;
        best_match_y = cpy;
        bestWasLastLoop = true;

      }
      // otherwise: the last might be the current winner, in which case i
      // have to save these values.
      else {
        if (bestWasLastLoop) {
          best_match_errPost = ee;
          best_match_DiffErrPost = e1A * e1B + e2A * e2B + e3A * e3B + e4A * e4B + e5A * e5B;
          bestWasLastLoop = false;
        }

        // collect second-best:
        // just take the best of all that are NOT equal to current best.
        if (ee < second_best_match_err) {
          second_best_match_err = ee;
          loopCSecond = loopCounter;
        }
      }

      // shift everything one further.
      eeLast = ee;
      val_cp_m2 = val_cp_m1;
      val_cp_m1 = val_cp;
      val_cp = val_cp_p1;
      val_cp_p1 = val_cp_p2;

      cpx += incx;
      cpy += incy;

      loopCounter++;
    }

    // if error too big, will return -3, otherwise -2.
    if (best_match_err > 4.0 * Constants.MAX_ERROR_STEREO) {
      return new Float32Array([-3, result_idepth, result_var, result_eplLength]);
    }

    // check if clear enough winner
    if (Math.abs(loopCBest - loopCSecond) > 1.0
      && Constants.MIN_DISTANCE_ERROR_STEREO * best_match_err > second_best_match_err) {
      return new Float32Array([-2, result_idepth, result_var, result_eplLength]);
    }

    let didSubpixel: boolean = false;
    if (Constants.useSubpixelStereo) {
      // ================== compute exact match =========================
      // compute gradients (they are actually only half the real gradient)
      let gradPre_pre: number = -(best_match_errPre - best_match_DiffErrPre);
      let gradPre_this: number = (best_match_err - best_match_DiffErrPre);
      let gradPost_this: number = -(best_match_err - best_match_DiffErrPost);
      let gradPost_post: number = (best_match_errPost - best_match_DiffErrPost);

      // final decisions here.
      let interpPost: boolean = false;
      let interpPre: boolean = false;

      // if one is oob: return false.
      if (best_match_errPre < 0 || best_match_errPost < 0) {
      }
      // - if zero-crossing occurs exactly in between (gradient
      // Inconsistent),
      else if ((gradPost_this < 0) !== (gradPre_this < 0)) {
      }

      // if pre has zero-crossing
      else if ((gradPre_pre < 0) !== (gradPre_this < 0)) {
        // if post has zero-crossing
        if ((gradPost_post < 0) !== (gradPost_this < 0)) {
        } else
          interpPre = true;
      }

      // if post has zero-crossing
      else if ((gradPost_post < 0) !== (gradPost_this < 0)) {
        interpPost = true;
      }
      // DO interpolation!
      // minimum occurs at zero-crossing of gradient, which is a straight
      // line => easy to compute.
      // the error at that point is also computed by just integrating.
      if (interpPre) {
        let d: number = gradPre_this / (gradPre_this - gradPre_pre);
        best_match_x -= d * incx;
        best_match_y -= d * incy;
        best_match_err = best_match_err - 2 * d * gradPre_this - (gradPre_pre - gradPre_this) * d * d;
        // if(enablePrintDebugInfo) stats->num_stereo_interpPre++;
        didSubpixel = true;

      } else if (interpPost) {
        let d: number = gradPost_this / (gradPost_this - gradPost_post);
        best_match_x += d * incx;
        best_match_y += d * incy;
        best_match_err = best_match_err + 2 * d * gradPost_this + (gradPost_post - gradPost_this) * d * d;
        // if(enablePrintDebugInfo) stats->num_stereo_interpPost++;
        didSubpixel = true;
      }
    }

    // sampleDist is the distance in pixel at which the realVal's were
    // sampled
    let sampleDist: number = Constants.GRADIENT_SAMPLE_DIST * rescaleFactor;

    let gradAlongLine: number = 0;
    let tmp: number = realVal_p2 - realVal_p1;
    gradAlongLine += tmp * tmp;
    tmp = realVal_p1 - realVal;
    gradAlongLine += tmp * tmp;
    tmp = realVal - realVal_m1;
    gradAlongLine += tmp * tmp;
    tmp = realVal_m1 - realVal_m2;
    gradAlongLine += tmp * tmp;

    gradAlongLine /= sampleDist * sampleDist;

    // check if interpolated error is OK. use evil hack to allow more error
    // if there is a lot of gradient.
    if (best_match_err > Constants.MAX_ERROR_STEREO + Math.sqrt(gradAlongLine) * 20) {
      return new Float32Array([-3, result_idepth, result_var, result_eplLength]);
    }

    // ================= calc depth (in KF) ====================
    // * KinvP = Kinv * (x,y,1); where x,y are pixel coordinates of point we
    // search for, in the KF.
    // * best_match_x = x-coordinate of found correspondence in the
    // reference frame.

    let idnew_best_match: number; // depth in the new image
    let alpha: number; // d(idnew_best_match) / d(disparity in pixel) == conputed
    // inverse depth derived by the pixel-disparity.
    if (incx * incx > incy * incy) {
      let oldX: number = this.fxi * best_match_x + this.cxi;
      let nominator: number = (oldX * this.otherToThis_t[2] - this.otherToThis_t[0]);

      let dot0: number = Vec.dot(KinvP, Vec.getCol(this.thisToOther_R, 0, 3));
      let dot2: number = Vec.dot(KinvP, Vec.getCol(this.thisToOther_R, 2, 3));

      idnew_best_match = (dot0 - oldX * dot2) / nominator;
      alpha = (incx * this.fxi
        * (dot0 * this.otherToThis_t[2] - dot2 * this.otherToThis_t[0])
        / (nominator * nominator));

    } else {
      let oldY: number = this.fyi * best_match_y + this.cyi;

      let nominator: number = (oldY * this.otherToThis_t[2] - this.otherToThis_t[1]);

      let dot1: number = Vec.dot(KinvP, Vec.getCol(this.thisToOther_R, 1, 3));
      let dot2: number = Vec.dot(KinvP, Vec.getCol(this.thisToOther_R, 2, 3));

      idnew_best_match = (dot1 - oldY * dot2) / nominator;
      alpha = (incy * this.fyi
        * (dot1 * this.otherToThis_t[2] - dot2 * this.otherToThis_t[1])
        / (nominator * nominator));

    }

    let allowNegativeIdepths: boolean = true;
    if (idnew_best_match < 0) {
      if (!allowNegativeIdepths)
        return new Float32Array([-2, result_idepth, result_var, result_eplLength]);
    }

    // ================= calc var (in NEW image) ====================

    // TODO: setting
    let cameraPixelNoise2: number = 4 * 4;

    // calculate error from photometric noise
    let photoDispError: number = 4.0 * cameraPixelNoise2 / (gradAlongLine + Constants.DIVISION_EPS);

    let trackingErrorFac: number = 0.25 * (1.0 + referenceFrame.initialTrackedResidual);

    // calculate error from geometric noise (wrong camera pose /
    // calibration)
    let gradsInterpX = Vec.interpolatedValue(this.activeKeyFrame.imageGradientXArrayLvl[0], u, v, this.width);
    let gradsInterpY = Vec.interpolatedValue(this.activeKeyFrame.imageGradientYArrayLvl[0], u, v, this.width);

    let geoDispError: number = (gradsInterpX * epxn + gradsInterpY * epyn) + Constants.DIVISION_EPS;
    geoDispError = trackingErrorFac * trackingErrorFac
      * (gradsInterpX * gradsInterpX + gradsInterpY * gradsInterpY) / (geoDispError * geoDispError);

    // final error consists of a small constant part (discretization error),
    // geometric and photometric error.
    result_var = alpha * alpha
      * ((didSubpixel ? 0.05 : 0.5) * sampleDist * sampleDist + geoDispError + photoDispError); // square to
    // make
    // variance

    result_idepth = idnew_best_match;

    result_eplLength = eplLength;

    this.debugDepth?.debugImageStereoLines(pClose[0],pClose[1],pFar[0],pFar[1])

    return new Float32Array([best_match_err, result_idepth, result_var, result_eplLength]);
  }

  regularizeDepthMapFillHoles(): void {
    this.buildRegIntegralBuffer();
    this.copyDepthMapArray();
    // TOOD: multithread
    this.regularizeDepthMapFillHolesRow(3, this.height - 2);
  }
  // Summed area table of number of valid DepthMapPixelHypothesis
  buildRegIntegralBuffer(): void {

    // Sum horizontally
    // TODO: run in parallel
    this.buildRegIntegralBufferRow1(0, this.height);

    // Sum vertically
    let wh: number = this.height * this.width;
    for (let i = this.width; i < wh; i++)
      this.validityIntegralBuffer[i] += this.validityIntegralBuffer[i - this.width];
  }

  buildRegIntegralBufferRow1(yMin: number, yMax: number): void {
    for (let y = yMin; y < yMax; y++) {
      let validityIntegralBufferSUM: number = 0;
      for (let x = 0; x < this.width; x++) {
        let idx: number = y * this.width + x;
        let src: DepthMapPixelHypothesis = this.currentDepthMap[idx];

        if (src.isValid)
          validityIntegralBufferSUM += src.validity_counter;

        // Sum/number of valid DepthMapPixelHypothesis in same row,
        // before the pixel?
        this.validityIntegralBuffer[idx] = validityIntegralBufferSUM;
      }
    }
  }

  regularizeDepthMapFillHolesRow(yMin: number, yMax: number): void {
    // =========== regularize fill holes
    let keyFrameMaxGradBuf: Float32Array = this.activeKeyFrame.imageGradientMaxArrayLvl[0];

    // For rows yMin to yMax
    for (let y = yMin; y < yMax; y++) {
      // For pixels (3 to width-2) in row
      for (let x = 3; x < this.width - 2; x++) {
        let idx: number = x + y * this.width;
        // Get hypothessis
        let dest: DepthMapPixelHypothesis = this.otherDepthMap[idx];

        if (dest.isValid)
          continue;
        if (keyFrameMaxGradBuf[idx] < Constants.MIN_ABS_GRAD_DECREASE)
          continue;

        // Number of valid pixels in some neighbourhood??
        let io: Float32Array = this.validityIntegralBuffer;
        let val: number = io[idx + 2 + (2 * this.width)] - io[idx + 2 - (3 * this.width)] - io[idx + -3 + (2 * this.width)]
          + io[idx + -3 - (3 * this.width)];

        // If blacklisted and surrounding has some number of valid
        // pixels
        if ((dest.blacklisted >= Constants.MIN_BLACKLIST && val > Constants.VAL_SUM_MIN_FOR_CREATE)
          || val > Constants.VAL_SUM_MIN_FOR_UNBLACKLIST) {

          // Calculate average idepth?

          let sumIdepthObs: number = 0, sumIVarObs: number = 0;
          let s1max: number = (x - 2) + (y + 3) * this.width;
          for (let s1 = (x - 2) + (y - 2) * this.width; s1 < s1max; s1 += this.width) {
            for (let s2 = s1; s2 < s1 + 5; s2++) {
              let source: DepthMapPixelHypothesis = this.otherDepthMap[s2];
              if (!source.isValid)
                continue;

              sumIdepthObs += source.idepth / source.idepth_var;
              sumIVarObs += 1.0 / source.idepth_var;
            }
          }

          let idepthObs: number = sumIdepthObs / sumIVarObs;
          idepthObs = this.UNZERO(idepthObs);

          // Create new hypothesis
          this.currentDepthMap[idx] = new DepthMapPixelHypothesis(idepthObs, DepthMap.VAR_RANDOM_INIT_INITIAL, 0);

        }
      }
    }
  }

  regularizeDepthMap(removeOcclusions: boolean, validityTH: number): void {
    this.copyDepthMapArray();
    this.regularizeDepthMapRow(validityTH, 2, this.height - 2, removeOcclusions);
  }

  copyDepthMapArray(): void {
    for (let i = 0; i < this.currentDepthMap.length; i++) {
      if (this.currentDepthMap[i]) {
        this.otherDepthMap[i] = new DepthMapPixelHypothesis(this.currentDepthMap[i]);
      } else {
        console.log("no currentDepthMap")
      }
    }
  }

  regularizeDepthMapRow(validityTH: number, yMin: number, yMax: number, removeOcclusions: boolean): void {

    let regularize_radius: number = 2;
    let regDistVar: number = Constants.REG_DIST_VAR;

    for (let y = yMin; y < yMax; y++) {
      for (let x = regularize_radius; x < this.width - regularize_radius; x++) {
        let idx: number = x + y * this.width;
        let dest: DepthMapPixelHypothesis = this.currentDepthMap[idx];
        let destRead: DepthMapPixelHypothesis = this.otherDepthMap[idx];

        // if isValid need to do better examination and then update.

        if (!destRead.isValid)
          continue;

        let sum: number = 0, val_sum: number = 0, sumIvar: number = 0;// , min_varObs = 1e20;
        let numOccluding: number = 0, numNotOccluding: number = 0;

        for (let dx = -regularize_radius; dx <= regularize_radius; dx++) {
          for (let dy = -regularize_radius; dy <= regularize_radius; dy++) {
            let source: DepthMapPixelHypothesis = this.otherDepthMap[idx + dx + dy * this.width];

            if (!source.isValid)
              continue;

            let diff: number = source.idepth - destRead.idepth;
            if (Constants.DIFF_FAC_SMOOTHING * diff * diff > source.idepth_var + destRead.idepth_var) {
              if (removeOcclusions) {
                if (source.idepth > destRead.idepth)
                  numOccluding++;
              }
              continue;
            }

            val_sum += source.validity_counter;

            if (removeOcclusions)
              numNotOccluding++;

            let distFac: number = (dx * dx + dy * dy) * regDistVar;
            let ivar: number = 1.0 / (source.idepth_var + distFac);

            sum += source.idepth * ivar;
            sumIvar += ivar;

          }
        }

        if (val_sum < validityTH) {
          dest.isValid = false;
          dest.blacklisted--;
          continue;
        }

        if (removeOcclusions) {
          if (numOccluding > numNotOccluding) {
            dest.isValid = false;
            continue;
          }
        }

        sum = sum / sumIvar;
        sum = this.UNZERO(sum);

        // update!
        dest.idepth_smoothed = sum;
        dest.idepth_var_smoothed = 1.0 / sumIvar;
      }
    }
  }

  // Regularize fill holes, regularize depth map, etc
  public finalizeKeyFrame(): void {
    if (!this.isValid()) {
      console.log("Assertion failed!");
    }

    this.regularizeDepthMapFillHoles();

    this.regularizeDepthMap(false, Constants.VAL_SUM_MIN_FOR_KEEP);

    this.activeKeyFrame.setDepth(this.currentDepthMap);
  }

  public createKeyFrame(new_keyframe: Frame): void {
    console.assert(new_keyframe != null);

    let oldToNew_SE3: SE3 = SE3.inverse(new_keyframe.thisToParent);

    this.propagateDepth(new_keyframe);

    this.activeKeyFrame = new_keyframe;

    this.regularizeDepthMap(true, Constants.VAL_SUM_MIN_FOR_KEEP);

    this.regularizeDepthMapFillHoles();

    this.regularizeDepthMap(false, Constants.VAL_SUM_MIN_FOR_KEEP);

    // make mean inverse depth be one.
    let sumIdepth: number = 0, numIdepth: number = 0;
    for (let i = 0; i < this.width * this.height; i++) {
      if (!this.currentDepthMap[i].isValid)
        continue;
      sumIdepth += this.currentDepthMap[i].idepth_smoothed;
      numIdepth++;
    }

    let rescaleFactor: number = numIdepth / sumIdepth;
    let rescaleFactor2: number = rescaleFactor * rescaleFactor;
    for (let i = 0; i < this.width * this.height; i++) {
      if (!this.currentDepthMap[i].isValid)
        continue;
      this.currentDepthMap[i].idepth *= rescaleFactor;
      this.currentDepthMap[i].idepth_smoothed *= rescaleFactor;
      this.currentDepthMap[i].idepth_var *= rescaleFactor2;
      this.currentDepthMap[i].idepth_var_smoothed *= rescaleFactor2;
    }
    this.activeKeyFrame.thisToParent = SE3.inverse(oldToNew_SE3);

    // Update depth in keyframe
    this.activeKeyFrame.setDepth(this.currentDepthMap);
  }

  propagateDepth(new_keyframe: Frame): void {
    if (new_keyframe.kfID != this.activeKeyFrame.id) {
      console.log(
        "WARNING: propagating depth from frame %d to %d, which was tracked on a different frame (%d).\nWhile this should work, it is not recommended.",
        this.activeKeyFrame.id, new_keyframe.id, new_keyframe.kfID);
    }

    // wipe depthmap
    for (let i = this.width * this.height - 1; i >= 0; i--) {
      let pt: DepthMapPixelHypothesis = this.otherDepthMap[i];
      pt.isValid = false;
      pt.blacklisted = 0;
    }

    // re-usable values.
    let oldToNew_SE3: SE3 = SE3.inverse(new_keyframe.thisToParent);
    let trafoInv_t: Float32Array = oldToNew_SE3.getTranslation();
    let trafoInv_R: Float32Array = oldToNew_SE3.getRotationMatrix();

    //let trackingWasGood: boolean[] | null = new_keyframe.kfID == this.activeKeyFrame.id ? new_keyframe._refPixelWasGood : null;

    let activeKFImageData: Float32Array = this.activeKeyFrame.imageArrayLvl[0];
    let newKFMaxGrad: Float32Array = new_keyframe.imageGradientMaxArrayLvl[0];
    let newKFImageData: Float32Array = new_keyframe.imageArrayLvl[0];

    // go through all pixels of OLD image, propagating forwards.
    for (let y = 0; y < this.height; y++)
      for (let x = 0; x < this.width; x++) {
        let source: DepthMapPixelHypothesis = this.currentDepthMap[x + y * this.width];

        if (!source.isValid)
          continue;

        let r: Float32Array = new Float32Array([x * this.fxi + this.cxi, y * this.fyi + this.cyi, 1.0]);

        let pn: Float32Array = Vec.vecAdd2(Vec.vectorDiv(Vec.matVecMultiplySqr(trafoInv_R, r, 3), source.idepth_smoothed),
          trafoInv_t);

        let new_idepth: number = (1.0 / pn[2]);

        let u_new: number = (pn[0] * new_idepth * this.fx + this.cx);
        let v_new: number = (pn[1] * new_idepth * this.fy + this.cy);

        // check if still within image, if not: DROP.
        if (!(u_new > 2.1 && v_new > 2.1 && u_new < this.width - 3.1 && v_new < this.height - 3.1)) {
          continue;
        }

        let newIDX: number = Math.floor(u_new + 0.5) + (Math.floor(v_new + 0.5)) * this.width;
        let destAbsGrad: number = newKFMaxGrad[newIDX];

        let sourceColor: number = activeKFImageData[x + y * this.width];
        let destColor: number = Vec.interpolatedValue(newKFImageData, u_new, v_new, this.width);

        let residual: number = destColor - sourceColor;

        if (residual * residual
          / (Constants.MAX_DIFF_CONSTANT
            + Constants.MAX_DIFF_GRAD_MULT * destAbsGrad * destAbsGrad) > 1.0
          || destAbsGrad < Constants.MIN_ABS_GRAD_DECREASE) {
          continue;
        }
        //}

        let targetBest: DepthMapPixelHypothesis = this.otherDepthMap[newIDX];

        // large idepth = point is near = large increase in variance.
        // small idepth = point is far = small increase in variance.
        let idepth_ratio_4: number = new_idepth / source.idepth_smoothed;
        idepth_ratio_4 *= idepth_ratio_4;
        idepth_ratio_4 *= idepth_ratio_4;

        let new_var: number = idepth_ratio_4 * source.idepth_var;

        // check for occlusion
        if (targetBest.isValid) {
          // if they occlude one another, one gets removed.
          let diff: number = targetBest.idepth - new_idepth;
          if (Constants.DIFF_FAC_PROP_MERGE * diff * diff > new_var + targetBest.idepth_var) {
            if (new_idepth < targetBest.idepth) {
              continue;
            } else {
              targetBest.isValid = false;
            }
          }
        }

        if (!targetBest.isValid) {

          targetBest = new DepthMapPixelHypothesis(new_idepth, new_var, source.validity_counter);
          this.otherDepthMap[newIDX] = targetBest;
        } else {

          // merge idepth ekf-style
          let w: number = new_var / (targetBest.idepth_var + new_var);
          let merged_new_idepth: number = w * targetBest.idepth + (1.0 - w) * new_idepth;

          // merge validity
          let merged_validity: number = Math.floor(source.validity_counter + targetBest.validity_counter);
          if (merged_validity > Constants.VALIDITY_COUNTER_MAX + (Constants.VALIDITY_COUNTER_MAX_VARIABLE))
            merged_validity = Math.floor(Constants.VALIDITY_COUNTER_MAX
              + (Constants.VALIDITY_COUNTER_MAX_VARIABLE));

          targetBest = new DepthMapPixelHypothesis(merged_new_idepth,
            1.0 / (1.0 / targetBest.idepth_var + 1.0 / new_var), merged_validity);

          this.otherDepthMap[newIDX] = targetBest;

        }
      }

    // swap!
    let temp: DepthMapPixelHypothesis[] = structuredClone(this.currentDepthMap);
    this.currentDepthMap = structuredClone(this.otherDepthMap);
    this.otherDepthMap = temp;
  }
  /*
   * Make val non-zero
   */
  UNZERO(val: number): number {
    return (val < 0 ? (val > -1e-10 ? -1e-10 : val) : (val < 1e-10 ? 1e-10 : val));
  }
}