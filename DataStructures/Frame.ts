import { Constants } from '../Utils/Constants';
import { DepthMapPixelHypothesis } from '../DepthEstimation/DepthMapPixelHypothesis';
import { SE3 } from '../LieAlgebra/SE3';

export class Frame {
  _width: number = 0;
  _height: number = 0;
  id: number = 0;
  public isKF: boolean = false;
  public imageArrayLvl: Array<Float32Array>;
  public imageGradientXArrayLvl: Array<Float32Array>;
  public imageGradientYArrayLvl: Array<Float32Array>;
  public imageGradientMaxArrayLvl: Array<Float32Array>;
  public inverseDepthLvl: Array<Float32Array>;
  public inverseDepthVarianceLvl: Array<Float32Array>;
  public posDataLvl: Array<Array<Float32Array>>;
  public colorDataLvl: Array<Float32Array>;
  public varDataLvl: Array<Float32Array>;
  public IDepthBeenSet: boolean = false;
  public numMappablePixels = 0
  public initialTrackedResidual: number = 0;
  public numFramesTrackedOnThis: number = 0;
  public numMappedOnThis: number = 0;
  public meanIdepth: number = 0;
  public numPoints: number = 0;
  public camToWorld: SE3 = new SE3();
  public thisToParent: SE3;
  public kfID: number = 0;
  public trackedOnPoses: SE3[] = [];

  /**
   * Clear unused data to reduce memory usage
   */
  public clearData() {
    this.imageGradientXArrayLvl.length = 0;
    this.imageGradientYArrayLvl.length = 0;
    this.inverseDepthVarianceLvl.length = 0;
    this.inverseDepthLvl.length = 0;
    this.posDataLvl.length = 0;
    this.colorDataLvl.length = 0
    this.varDataLvl.length = 0
    if (!this.isKF) {
      this.imageGradientMaxArrayLvl.length = 0;
      this.imageArrayLvl.length = 0;
    }
  }

  public constructor(image: Float32Array, width: number, height: number, id: number) {
    this._width = width;
    this._height = height;
    this.id = id;
    this.imageArrayLvl = Array(Constants.PYRAMID_LEVELS);
    this.imageGradientXArrayLvl = Array(Constants.PYRAMID_LEVELS);
    this.imageGradientYArrayLvl = Array(Constants.PYRAMID_LEVELS);
    this.imageGradientMaxArrayLvl = Array(Constants.PYRAMID_LEVELS);
    this.inverseDepthLvl = Array(Constants.PYRAMID_LEVELS);
    this.inverseDepthVarianceLvl = Array(Constants.PYRAMID_LEVELS);
    this.posDataLvl = Array(Constants.PYRAMID_LEVELS);
    this.colorDataLvl = Array(Constants.PYRAMID_LEVELS);
    this.varDataLvl = Array(Constants.PYRAMID_LEVELS)
    for (let i: number = 0; i < Constants.PYRAMID_LEVELS; i++) {
      const w = this.width(i)
      const h = this.height(i)
      i == 0 ? this.imageArrayLvl[0] = image : this.imageArrayLvl[i] = this.buildImageLevel(this.imageArrayLvl[i - 1], this.width(i - 1), this.height(i - 1));
      this.imageGradientXArrayLvl[i] = this.gradientX(this.imageArrayLvl[i], w, h);
      this.imageGradientYArrayLvl[i] = this.gradientY(this.imageArrayLvl[i], w, h);
      this.imageGradientMaxArrayLvl[i] = this.gradientMax(this.imageGradientXArrayLvl[i], this.imageGradientYArrayLvl[i], i, w, h);
    }
  }

  gradientMax(imageGradientXArrayLvl: Float32Array, imageGradientYArrayLvl: Float32Array, level: number, w: number, h: number): Float32Array {
    let gradientMax: Float32Array = new Float32Array(w * h);
    let gradientMaxTemp: Float32Array = new Float32Array(w * h);
    for (let i: number = w; i < w * (h - 1); i++) {
      let dx: number = imageGradientXArrayLvl[i];
      let dy: number = imageGradientYArrayLvl[i];
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

  gradientX(imageArrayLvl: Float32Array, w: number, h: number): Float32Array {
    const imageGradientXArray: Float32Array = new Float32Array(imageArrayLvl.length);
    for (let i: number = w; i <= w * (h - 1); i++)
      imageGradientXArray[i] = 0.5 * (imageArrayLvl[i + 1] - imageArrayLvl[i - 1]);
    return imageGradientXArray;
  }

  gradientY(imageArrayLvl: Float32Array, w: number, h: number): Float32Array {
    const imageGradientYArray: Float32Array = new Float32Array(imageArrayLvl.length);
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
    this.meanIdepth = numIdepth > 0 ? sumIdepth / numIdepth : 0;
    this.numPoints = numIdepth;
    this.IDepthBeenSet = true;
    for (let level = 1; level < Constants.PYRAMID_LEVELS; level++)
      this.#buildIDepthAndIDepthVar(level);

    this.#createPointCloud()
  }

  #buildIDepthAndIDepthVar(level: number) {
    if (level <= 0) {
      console.error("Invalid level parameter!");
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

  buildImageLevel(imageArraySrc: Float32Array, width: number, height: number): Float32Array {
    const imageArrayDst: Float32Array = new Float32Array((width * height) / 4)
    let dstIdx: number = 0;
    for (let y = 0; y < height; y += 2) {
      for (let x = 0; x < width; x += 2) {
        imageArrayDst[dstIdx++] = (
          imageArraySrc[x + y * width] +
          imageArraySrc[(x + 1) + y * width] +
          imageArraySrc[x + (y + 1) * width] +
          imageArraySrc[(x + 1) + (y + 1) * width]
        ) * 0.25;
      }
    }
    return imageArrayDst;
  }

  /**
   * Create 3D points from inverse depth values and count valid points
   */
  #createPointCloud() {
    for (let level = 0; level < Constants.PYRAMID_LEVELS; level++) {
      const width: number = this.width(level);
      const height: number = this.height(level);
      const image: Float32Array = this.imageArrayLvl[level];
      const inverseDepth: Float32Array = this.inverseDepthLvl[level];
      const inverseDepthVariance: Float32Array = this.inverseDepthVarianceLvl[level];
      const fxInv: number = Constants.fxInv[level];
      const fyInv: number = Constants.fyInv[level];
      const cxInv: number = Constants.cxInv[level];
      const cyInv: number = Constants.cyInv[level];
      const posData = Array(width * height);
      const colorData = new Float32Array(width * height);
      const varData = new Float32Array(width * height);
      for (let x: number = 1; x < width - 1; x++) {
        for (let y: number = 1; y < height - 1; y++) {
          const idx: number = x + y * width;
          const idepth: number = inverseDepth[idx];
          const vrb: number = inverseDepthVariance[idx];
          if (idepth == 0 || vrb <= 0) continue;
          posData[idx] = new Float32Array([(fxInv * x + cxInv) / idepth, (fyInv * y + cyInv) / idepth, 1 / idepth]);
          colorData[idx] = image[idx]
          varData[idx] = vrb
        }
      }
      this.posDataLvl[level] = posData
      this.colorDataLvl[level] = colorData
      this.varDataLvl[level] = varData
    }
  }
}