import { Constants } from "../Utils/Constants";
import { Frame } from "./Frame";

export class StereoFrame extends Frame {
  public imageArrayLvlR: Array<Float32Array>;
  public imageGradientXArrayLvlR: Array<Float32Array>;
  public imageGradientYArrayLvlR: Array<Float32Array>;
  public imageGradientMaxArrayLvlR: Array<Float32Array>;
  constructor(image: Float32Array, imageR: Float32Array, width: number, height: number, id: number) {
    super(image, width, height, id)
    this.imageArrayLvlR = Array(Constants.PYRAMID_LEVELS);
    this.imageGradientXArrayLvlR = Array(Constants.PYRAMID_LEVELS);
    this.imageGradientYArrayLvlR = Array(Constants.PYRAMID_LEVELS);
    this.imageGradientMaxArrayLvlR = Array(Constants.PYRAMID_LEVELS);
    for (let i: number = 0; i < Constants.PYRAMID_LEVELS; i++) {
      const w = this.width(i)
      const h = this.height(i)
      i == 0 ? this.imageArrayLvlR[0] = imageR : this.imageArrayLvlR[i] = this.buildImageLevel(this.imageArrayLvlR[i - 1], this.width(i - 1), this.height(i - 1));
      this.imageGradientXArrayLvlR[i] = this.gradientX(this.imageArrayLvlR[i], w, h);
      this.imageGradientYArrayLvlR[i] = this.gradientY(this.imageArrayLvlR[i], w, h);
      this.imageGradientMaxArrayLvlR[i] = this.gradientMax(this.imageGradientXArrayLvlR[i], this.imageGradientYArrayLvlR[i], i, w, h);
    }
  }

  public clearData(): void {
    super.clearData()
    this.imageArrayLvlR.length = 0;
    this.imageGradientXArrayLvlR.length = 0;
    this.imageGradientYArrayLvlR.length = 0;
    this.imageGradientMaxArrayLvlR.length = 0;
  }
}