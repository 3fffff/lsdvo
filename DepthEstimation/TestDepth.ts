import { Frame } from "../DataStructures/Frame";
import { Constants } from "../Utils/Constants";
import { DepthMap } from "./DepthMap";

export class TestDepth {
  stereoTest: HTMLCanvasElement = document.createElement("canvas");
  stereoTestCtx: CanvasRenderingContext2D | null = this.stereoTest.getContext("2d")
  stereoTestReference: HTMLCanvasElement = document.createElement("canvas");
  stereoTestReferenceCtx: CanvasRenderingContext2D | null = this.stereoTest.getContext("2d")
  keyFrame: HTMLCanvasElement = document.createElement("canvas");
  keyFrameCtx: CanvasRenderingContext2D | null = this.keyFrame.getContext("2d")
  grad: HTMLCanvasElement = document.createElement("canvas");
  gradCtx: CanvasRenderingContext2D | null = this.grad.getContext("2d")
  imgHypothesis: HTMLCanvasElement = document.createElement("canvas")
  imgHypoCtx: CanvasRenderingContext2D | null = this.imgHypothesis.getContext("2d")
  width: number
  height: number
  map: DepthMap

  constructor(map: DepthMap) {
    this.width = this.stereoTest.width = this.stereoTest.width = this.keyFrame.width = this.grad.width = this.imgHypothesis.width = map.width;
    this.height = this.stereoTest.height = this.stereoTestReference.height = this.keyFrame.height = this.grad.height = this.imgHypothesis.height = map.height;
    this.stereoTest.id = 'stereo';
    this.stereoTestReference.id = 'reference';
    this.keyFrame.id = 'keyframe';
    this.grad.id = 'grad';
    this.imgHypothesis.id = "hypothesis"
    document.body.appendChild(this.keyFrame)
    document.body.appendChild(this.grad)
    document.body.appendChild(this.stereoTest)
    document.body.appendChild(this.imgHypothesis)
    this.map = map
  }

  stereoDebug(): void {
    if (this.stereoTestCtx == null || this.stereoTestReferenceCtx == null) return
    this.stereoTestCtx.clearRect(0, 0, this.width, this.height);
    const stereoTestI: ImageData = this.stereoTestCtx.createImageData(this.width, this.height);
    for (let y = 0; y < this.height; y++) {
      for (let x = 0; x < this.width; x++) {
        const idx = x + y * this.width;
        stereoTestI.data[idx * 4 + 0] = Math.ceil(0.5 * this.map.oldest_referenceFrame.imageArrayLvl[0][idx] + 0.5 * this.map.newest_referenceFrame.imageArrayLvl[0][idx])
        stereoTestI.data[idx * 4 + 1] = Math.ceil(0.5 * this.map.oldest_referenceFrame.imageArrayLvl[0][idx] + 0.5 * this.map.newest_referenceFrame.imageArrayLvl[0][idx])
        stereoTestI.data[idx * 4 + 2] = Math.ceil(0.5 * this.map.oldest_referenceFrame.imageArrayLvl[0][idx] + 0.5 * this.map.newest_referenceFrame.imageArrayLvl[0][idx])
        stereoTestI.data[idx * 4 + 3] = 255
      }
    }
    this.stereoTestCtx.putImageData(stereoTestI, 0, 0);
    this.stereoTestReferenceCtx.clearRect(0, 0, this.width, this.height);
    let stereoTestReferenceI: ImageData = this.stereoTestReferenceCtx.createImageData(this.width, this.height);
    for (let y = 0; y < this.height; y++) {
      for (let x = 0; x < this.width; x++) {
        const idx = x + y * this.width;
        stereoTestReferenceI.data[idx * 4 + 0] = Math.ceil(0.5 * this.map.oldest_referenceFrame.imageArrayLvl[0][idx] + 0.5 * this.map.newest_referenceFrame.imageArrayLvl[0][idx])
        stereoTestReferenceI.data[idx * 4 + 1] = Math.ceil(0.5 * this.map.oldest_referenceFrame.imageArrayLvl[0][idx] + 0.5 * this.map.newest_referenceFrame.imageArrayLvl[0][idx])
        stereoTestReferenceI.data[idx * 4 + 2] = Math.ceil(0.5 * this.map.oldest_referenceFrame.imageArrayLvl[0][idx] + 0.5 * this.map.newest_referenceFrame.imageArrayLvl[0][idx])
        stereoTestReferenceI.data[idx * 4 + 3] = 255
      }
    }
    this.stereoTestReferenceCtx.putImageData(stereoTestReferenceI, 0, 0);
  }

  debugPlotDepthMap(frame: Frame): void {
    if (frame == null) return;
    if (!this.keyFrameCtx || !this.gradCtx) return;

    let imgDataS: ImageData = this.keyFrameCtx.createImageData(this.width, this.height);
    let imgDataGS: ImageData = this.gradCtx.createImageData(this.width, this.height);
    for (let i = 0; i < this.height * this.width; i++) {
      imgDataS.data[i * 4 + 0] = frame.imageArrayLvl[0][i];
      imgDataS.data[i * 4 + 1] = frame.imageArrayLvl[0][i];
      imgDataS.data[i * 4 + 2] = frame.imageArrayLvl[0][i];
      imgDataS.data[i * 4 + 3] = 255;
      imgDataGS.data[i * 4 + 0] = frame.imageArrayLvl[0][i];
      imgDataGS.data[i * 4 + 1] = frame.imageArrayLvl[0][i];
      imgDataGS.data[i * 4 + 2] = frame.imageArrayLvl[0][i];
      imgDataGS.data[i * 4 + 3] = 255
      if (frame.imageGradientMaxArrayLvl[0][i] > Constants.MIN_ABS_GRAD_CREATE)
        imgDataGS.data[i * 4 + 1] = 255
      if (this.map.currentDepthMap[i].blacklisted < Constants.MIN_BLACKLIST &&
        Constants.debugDisplay == 2)
        imgDataS.data[i * 4 + 3] = 255

      if (!this.map.currentDepthMap[i].isValid) continue;

      const color: Uint8Array = this.map.currentDepthMap[i].getVisualizationColor(3);
      imgDataS.data[i * 4 + 0] = color[0]
      imgDataS.data[i * 4 + 1] = color[1]
      imgDataS.data[i * 4 + 2] = color[2]
    }
    this.keyFrameCtx.putImageData(imgDataS, 0, 0);
    this.gradCtx.putImageData(imgDataGS, 0, 0);
  }
}