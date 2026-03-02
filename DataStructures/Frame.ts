import { Constants } from '../Utils/Constants';
import { DepthMapPixelHypothesis } from '../DepthEstimation/DepthMapPixelHypothesis';
import { SE3 } from '../LieAlgebra/SE3';

export class Frame {
  #width: number = 0;
  #height: number = 0;
  id: number = 0;
  public isKF: boolean = false;
  public imageArrayLvl: Array<Float32Array>;
  public imageGradientXArrayLvl: Array<Float32Array>;
  public imageGradientYArrayLvl: Array<Float32Array>;
  public imageGradientMaxArrayLvl: Array<Float32Array>;
  public pointCloudLvl: Array<Array<[number, number, number, number, number]>>;
  public IDepthSet: boolean = false;
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
    this.pointCloudLvl.length = 0;
    if (!this.isKF) {
      this.imageGradientMaxArrayLvl.length = 0;
      this.imageArrayLvl.length = 0;
    }
  }

  public constructor(image: Float32Array, width: number, height: number, id: number) {
    this.#width = width;
    this.#height = height;
    this.id = id;
    this.imageArrayLvl = Array(Constants.PYRAMID_LEVELS);
    this.imageGradientXArrayLvl = Array(Constants.PYRAMID_LEVELS);
    this.imageGradientYArrayLvl = Array(Constants.PYRAMID_LEVELS);
    this.imageGradientMaxArrayLvl = Array(Constants.PYRAMID_LEVELS);
    this.pointCloudLvl = Array(Constants.PYRAMID_LEVELS);
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
    for (let i = w; i < w * (h - 1); i++) {
      let dx = imageGradientXArrayLvl[i];
      let dy = imageGradientYArrayLvl[i];
      gradientMax[i] = Math.sqrt(dx * dx + dy * dy);
    }

    let gradientMaxTemp = new Float32Array(w * h);
    for (let y = 1; y < h - 1; y++) {
      for (let x = 1; x < w - 1; x++) {
        let idx = x + y * w;
        let maxVal = 0;
        for (let dy = -1; dy <= 1; dy++)
          for (let dx = -1; dx <= 1; dx++)
            maxVal = Math.max(maxVal, gradientMax[idx + dx + dy * w]);
        gradientMaxTemp[idx] = maxVal;
        if (level === 0 && maxVal >= Constants.MIN_ABS_GRAD_CREATE)
          this.numMappablePixels++;
      }
    }
    return gradientMaxTemp;
  }

  gradientX(imageArrayLvl: Float32Array, w: number, h: number): Float32Array {
    const imageGradientXArray: Float32Array = new Float32Array(imageArrayLvl.length);
    for (let i: number = w; i < w * (h - 1); i++)
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
    return this.#width >> level;
  }

  public height(level: number): number {
    return this.#height >> level;
  }

  public setDepth(newDepth: Array<DepthMapPixelHypothesis>): void {

    let idepthSrc: Float32Array | undefined;
    let ivarSrc: Float32Array | undefined;
    let srcW = 0;

    for (let lvl = 0; lvl < Constants.PYRAMID_LEVELS; lvl++) {
      const width = this.width(lvl);
      const height = this.height(lvl);
      const pixels = width * height;
      const imageLvl = this.imageArrayLvl[lvl];

      const idepthDst = new Float32Array(pixels);
      const ivarDst = new Float32Array(pixels);
      const posData: Array<[number, number, number, number, number]> = [];

      let levelNumPoints = 0;
      let levelSumIdepth = 0;

      for (let y = 0; y < height; y++) {
        const cyTerm = Constants.fyInv[lvl] * y + Constants.cyInv[lvl];
        const dstRow = y * width;

        for (let x = 0; x < width; x++) {
          const dstIdx = x + dstRow;
          let idepth = -1, ivar = -1;

          if (lvl === 0) {
            // Level 0: extract from DepthMapPixelHypothesis[]
            const h = newDepth[dstIdx];
            if (h.isValid && h.idepth_smoothed > 0) {
              idepth = h.idepth_smoothed;
              ivar = h.idepth_var_smoothed;
            }
          } else {
            // Levels 1+: variance-weighted 2x2 pooling from previous level
            const xSrc = x << 1, ySrc = y << 1;
            const srcRow = ySrc * srcW;
            const baseIdx = xSrc + srcRow;

            let sumW = 0, sumWI = 0, count = 0;

            // Unrolled 2x2 neighborhood for performance
            let v = ivarSrc![baseIdx];
            if (v > 0) { const w = 1 / v; sumW += w; sumWI += w * idepthSrc![baseIdx]; count++; }
            v = ivarSrc![baseIdx + 1];
            if (v > 0) { const w = 1 / v; sumW += w; sumWI += w * idepthSrc![baseIdx + 1]; count++; }
            v = ivarSrc![baseIdx + srcW];
            if (v > 0) { const w = 1 / v; sumW += w; sumWI += w * idepthSrc![baseIdx + srcW]; count++; }
            v = ivarSrc![baseIdx + srcW + 1];
            if (v > 0) { const w = 1 / v; sumW += w; sumWI += w * idepthSrc![baseIdx + srcW + 1]; count++; }

            if (count > 0) {
              ivar = count / sumW;        // fused variance
              idepth = sumWI / sumW;      // weighted average inverse depth
            }
          }

          if (idepth > 0 && ivar > 0) {
            idepthDst[dstIdx] = idepth;
            ivarDst[dstIdx] = ivar;

            // Accumulate stats for level 0 only (used for class-level meanIdepth/numPoints)
            if (lvl === 0) {
              levelNumPoints++;
              levelSumIdepth += idepth;
            }

            const depth = 1.0 / idepth;
            posData.push([
              (Constants.fxInv[lvl] * x + Constants.cxInv[lvl]) * depth,
              cyTerm * depth,
              depth,
              imageLvl[dstIdx],
              ivar
            ]);
          } else {
            idepthDst[dstIdx] = -1;
            ivarDst[dstIdx] = -1;
          }
        }
      }

      this.pointCloudLvl[lvl] = posData;

      // Set global statistics from level 0 only
      if (lvl === 0) {
        this.meanIdepth = levelNumPoints > 0 ? levelSumIdepth / levelNumPoints : 0;
        this.numPoints = levelNumPoints;
        this.IDepthSet = true;
      }

      // Chain buffers for next pyramid level
      idepthSrc = idepthDst;
      ivarSrc = ivarDst;
      srcW = width;
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
}