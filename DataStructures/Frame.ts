import { Constants } from '../Utils/Constants';
import { SIM3 } from '../LieAlgebra/SIM3';
import { DepthMapPixelHypothesis } from '../DepthEstimation/DepthMapPixelHypothesis';
import { Vec } from '../LieAlgebra/Vec';
import { SE3 } from '../LieAlgebra/SE3';
export class Frame {
  public isKF: boolean = false;
  _width: number = 0;
  _height: number = 0;
  public imageArrayLvl: Array<Float32Array>;
  public imageGradientXArrayLvl: Array<Float32Array>;
  public imageGradientYArrayLvl: Array<Float32Array>;
  public imageGradientMaxArrayLvl: Array<Float32Array>;
  public inverseDepthLvl: Array<Float32Array>;
  public inverseDepthVarianceLvl: Array<Float32Array>;
  public depthHasBeenUpdatedFlag: boolean = false;
  public id: number = 0;
  public K_otherToThis_R: Float32Array;
  public K_otherToThis_t: Float32Array;
  public otherToThis_t: Float32Array;
  thisToOther_R: Float32Array;
  thisToOther_t: Float32Array;
  public initialTrackedResidual: number = 0;
  public numMappedOnThisTotal: number = 0;
  public meanIdepth: number = 0;
  public thisToParent: SIM3 = new SIM3();
  public kfID: number = 0;
  public trackingParent: SIM3 = new SIM3();
  public trackedOnPoses: SE3[] = [];
  public numMappablePixels = 0
  //point cloud
  public posData: Array<Float32Array>;
  public colorAndVarData: Array<Float32Array>;

  /**
   * Clear unused data to reduce memory usage
   */
  public clearData() {
    this.imageGradientXArrayLvl.length = 0;
    this.imageGradientYArrayLvl.length = 0;
    this.imageGradientMaxArrayLvl.length = 0;
    this.colorAndVarData.length = 0
    this.imageArrayLvl.length = 0;
    this.inverseDepthLvl.length = 0;
    this.inverseDepthVarianceLvl.length = 0;
    if (!this.isKF) this.posData.length = 0
  }

  public constructor(image: Float32Array, width: number, height: number, totalFrames: number) {
    this._width = width;
    this._height = height;
    this.id = totalFrames;
    this.imageArrayLvl = Array(Constants.PYRAMID_LEVELS);
    this.imageGradientXArrayLvl = Array(Constants.PYRAMID_LEVELS);
    this.imageGradientYArrayLvl = Array(Constants.PYRAMID_LEVELS);
    this.imageGradientMaxArrayLvl = Array(Constants.PYRAMID_LEVELS);
    this.inverseDepthLvl = Array(Constants.PYRAMID_LEVELS);
    this.inverseDepthVarianceLvl = Array(Constants.PYRAMID_LEVELS);
    this.imageArrayLvl[0] = image;
    this.imageGradientXArrayLvl[0] = this.gradientX(this.imageArrayLvl[0], 0);
    this.imageGradientYArrayLvl[0] = this.gradientY(this.imageArrayLvl[0], 0);
    this.imageGradientMaxArrayLvl[0] = this.gradientMax(0);
    for (let i: number = 1; i < Constants.PYRAMID_LEVELS; i++) {
      this.imageArrayLvl[i] = new Float32Array((this.imageArrayLvl[i - 1].length / 4 | 0));
      this.buildImageLevel(this.imageArrayLvl[i - 1], this.imageArrayLvl[i], i);
      this.imageGradientXArrayLvl[i] = this.gradientX(this.imageArrayLvl[i], i);
      this.imageGradientYArrayLvl[i] = this.gradientY(this.imageArrayLvl[i], i);
      this.imageGradientMaxArrayLvl[i] = this.gradientMax(i);
    }
  }

  gradientMax(level: number): Float32Array {
    let w: number = this.width(level);
    let h: number = this.height(level);
    let gradientMax: Float32Array = new Float32Array(w * h);
    let gradientMaxTemp: Float32Array = new Float32Array(w * h);
    for (let i: number = w; i < w * (h - 1); i++) {
      let dx: number = this.imageGradientXArrayLvl[level][i];
      let dy: number = this.imageGradientYArrayLvl[level][i];
      gradientMax[i] = Math.sqrt(dx * dx + dy * dy);
    }
    for (let i: number = w + 1; i < w * (h - 1) - 1; i++) {
      let g1: number = gradientMax[i - w];
      let g2: number = gradientMax[i];
      if (g1 < g2) g1 = g2;
      let g3: number = gradientMax[i + w];
      if (g1 < g3) gradientMaxTemp[i] = g3;
      else gradientMaxTemp[i] = g1;
    }
    for (let i: number = w + 1; i < w * (h - 1) - 1; i++) {
      let g1: number = gradientMaxTemp[i - 1];
      let g2: number = gradientMaxTemp[i];
      if (g1 < g2) {
        g1 = g2;
        if (g2 >= Constants.MIN_ABS_GRAD_CREATE && level == 0) this.numMappablePixels++;
      }
      let g3: number = gradientMaxTemp[i + 1];
      if (g1 < g3) {
        gradientMax[i] = g3;
        if (g3 >= Constants.MIN_ABS_GRAD_CREATE && level == 0) this.numMappablePixels++;
      }
      else gradientMax[i] = g1;
    }
    return gradientMax;
  }

  gradientX(imageArrayLvl: Float32Array, level: number): Float32Array {
    let imageGradientXArray: Float32Array = new Float32Array(imageArrayLvl.length);
    let w: number = this.width(level);
    let h: number = this.height(level);
    for (let i: number = w; i <= w * (h - 1); i++)
      imageGradientXArray[i] = 0.5 * (imageArrayLvl[i + 1] - imageArrayLvl[i - 1]);
    return imageGradientXArray;
  }

  gradientY(imageArrayLvl: Float32Array, level: number): Float32Array {
    let imageGradientYArray: Float32Array = new Float32Array(imageArrayLvl.length);
    let w: number = this.width(level);
    let h: number = this.height(level);
    for (let i: number = w; i < w * (h - 1); i++)
      imageGradientYArray[i] = 0.5 * (imageArrayLvl[i + w] - imageArrayLvl[i - w]);
    return imageGradientYArray;
  }

  public width(level: number): number {
    return this._width >> level;
  }

  public height(level: number): number {
    return this._height >> level;
  }

  public setDepth(newDepth: Array<DepthMapPixelHypothesis>): void {
    if (this.depthHasBeenUpdatedFlag) return;
    let numIdepth: number = 0;
    let sumIdepth: number = 0;

    let pixels: number = this.width(0) * this.height(0);
    if (!this.inverseDepthLvl[0])
      this.inverseDepthLvl[0] = new Float32Array(pixels);
    if (!this.inverseDepthVarianceLvl[0])
      this.inverseDepthVarianceLvl[0] = new Float32Array(pixels);

    for (let i = 0; i < pixels; i++) {
      if (newDepth[i].isValid && newDepth[i].idepth_smoothed >= -0.05) {
        this.inverseDepthLvl[0][i] = newDepth[i].idepth_smoothed;
        this.inverseDepthVarianceLvl[0][i] = newDepth[i].idepth_var_smoothed;

        numIdepth++;
        sumIdepth += newDepth[i].idepth_smoothed;
      } else {
        this.inverseDepthLvl[0][i] = -1;
        this.inverseDepthVarianceLvl[0][i] = -1;
      }
    }

    this.meanIdepth = sumIdepth / numIdepth;

    this.depthHasBeenUpdatedFlag = true;

    // Do lower levels
    for (let level = 1; level < Constants.PYRAMID_LEVELS; level++)
      this.buildIDepthAndIDepthVar(level);
  }

  buildIDepthAndIDepthVar(level: number) {
    if (level <= 0) {
      console.error("buildIDepthAndIDepthVar: Invalid level parameter!");
      return;
    }
    let width: number = this.width(level);
    let height: number = this.height(level);
    let sw: number = this.width(level - 1);
    this.inverseDepthLvl[level] = new Float32Array(width * height);
    this.inverseDepthVarianceLvl[level] = new Float32Array(width * height);
    let idepthSource: Float32Array = this.inverseDepthLvl[level - 1];
    let idepthVarSource: Float32Array = this.inverseDepthVarianceLvl[level - 1];
    let idepthDest: Float32Array = this.inverseDepthLvl[level];
    let idepthVarDest: Float32Array = this.inverseDepthVarianceLvl[level];
    for (let y: number = 0; y < height; y++) {
      for (let x: number = 0; x < width; x++) {
        let idx: number = 2 * (x + y * sw);
        let idxDest: number = (x + y * width);
        let idepthSumsSum: number = 0;
        let ivarSumsSum: number = 0;
        let num: number = 0;
        if (idepthVarSource[idx] > 0) {
          let ivar = 1.0 / idepthVarSource[idx];
          ivarSumsSum += ivar;
          idepthSumsSum += ivar * idepthSource[idx];
          num++;
        }
        if (idepthVarSource[idx + 1] > 0) {
          let ivar = 1.0 / idepthVarSource[idx + 1];
          ivarSumsSum += ivar;
          idepthSumsSum += ivar * idepthSource[idx + 1];
          num++;
        }
        if (idepthVarSource[idx + sw] > 0) {
          let ivar = 1.0 / idepthVarSource[idx + sw];
          ivarSumsSum += ivar;
          idepthSumsSum += ivar * idepthSource[idx + sw];
          num++;
        }
        if (idepthVarSource[idx + sw + 1] > 0) {
          let ivar = 1.0 / idepthVarSource[idx + sw + 1];
          ivarSumsSum += ivar;
          idepthSumsSum += ivar * idepthSource[idx + sw + 1];
          num++;
        }
        if (num > 0) {
          let depth: number = ivarSumsSum / idepthSumsSum;
          idepthDest[idxDest] = 1.0 / depth;
          idepthVarDest[idxDest] = num / ivarSumsSum;
        } else {
          idepthDest[idxDest] = -1;
          idepthVarDest[idxDest] = -1;
        }
      }
    }
  }

  buildImageLevel(imageArraySrc: Float32Array, imageArrayDst: Float32Array, level: number) {
    let width: number = this.width(level - 1);
    let height: number = this.height(level - 1);
    let dstIdx: number = 0;
    for (let y: number = 0; y < width * height; y += width * 2)
      for (let x: number = 0; x < width; x += 2)
        imageArrayDst[dstIdx++] = (imageArraySrc[x + y] + imageArraySrc[x + y + 1] + imageArraySrc[x + y + width] + imageArraySrc[x + y + 1 + width]) * 0.25;
  }
  public prepareForStereoWith(thisToOther: SIM3): void {
    let otherToThis: SIM3 = thisToOther.inverse();

    this.K_otherToThis_R = Vec.matrixMul(Vec.multiplicar(Constants.K[0], otherToThis.getRotationM(), 3, 3, 3, 3),
      otherToThis.getScale());
    this.otherToThis_t = otherToThis.getTranslation();
    this.K_otherToThis_t = Vec.matVecMultiplySqr(Constants.K[0], this.otherToThis_t, 3);

    this.thisToOther_t = thisToOther.getTranslation();
    this.thisToOther_R = Vec.matrixMul(thisToOther.getRotationM(), thisToOther.getScale());

  }

  public getScaledCamToWorld(): SIM3 {
    return this.thisToParent.mul(this.trackingParent);
  }

  public createPointCloud(level: number) {
    let width: number = this.width(level);
    let height: number = this.height(level);
    let image: Float32Array = this.imageArrayLvl[level];
    let inverseDepth: Float32Array = this.inverseDepthLvl[level];
    let inverseDepthVariance: Float32Array = this.inverseDepthVarianceLvl[level];
    let fxInv: number = Constants.fxInv[level];
    let fyInv: number = Constants.fyInv[level];
    let cxInv: number = Constants.cxInv[level];
    let cyInv: number = Constants.cyInv[level];
    this.posData = Array(width * height);
    this.colorAndVarData = Array(width * height);
    for (let x: number = 1; x < width - 1; x++) {
      for (let y: number = 1; y < height - 1; y++) {
        let idx: number = x + y * width;
        let idepth: number = inverseDepth[idx];
        let vrb: number = inverseDepthVariance[idx];
        if (idepth === 0 || vrb <= 0) continue;
        this.posData[idx] = new Float32Array([(fxInv * x + cxInv) / idepth, (fyInv * y + cyInv) / idepth, 1 / idepth]);
        this.colorAndVarData[idx] = new Float32Array([image[idx], vrb]);
      }
    }
  }
}