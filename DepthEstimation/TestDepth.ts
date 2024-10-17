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
  imgStereoLines: HTMLCanvasElement = document.createElement("canvas")
  imgStereoLinesCtx: CanvasRenderingContext2D | null = this.imgStereoLines.getContext("2d")
  width: number
  height: number
  map: DepthMap

  constructor(map: DepthMap) {
    this.width = this.keyFrame.width = this.grad.width = this.imgStereoLines.width = map.width;
    this.height = this.stereoTestReference.height = this.keyFrame.height = this.grad.height = this.imgStereoLines.height = map.height;
    this.keyFrame.id = 'keyframe';
    this.grad.id = 'grad';
    this.imgStereoLines.id = "imgStereoLines"
    document.body.appendChild(this.keyFrame)
    document.body.appendChild(this.grad)
    document.body.appendChild(this.imgStereoLines)
    this.map = map
  }

  debugPlotDepthMap(frame: Frame,reFrame:Frame): void {
    if (frame == null) return;
    if (!this.keyFrameCtx || !this.gradCtx || !this.imgStereoLinesCtx) return;
    let imgData: ImageData = this.imgStereoLinesCtx.createImageData(this.width, this.height);
    let imgDataS: ImageData = this.keyFrameCtx.createImageData(this.width, this.height);
    let imgDataGS: ImageData = this.gradCtx.createImageData(this.width, this.height);
    for (let i = 0; i < this.height * this.width; i++) {
      imgData.data[i * 4 + 0] = reFrame.imageArrayLvl[0][i];
      imgData.data[i * 4 + 1] = reFrame.imageArrayLvl[0][i];
      imgData.data[i * 4 + 2] = reFrame.imageArrayLvl[0][i];
      imgData.data[i * 4 + 3] = 255;
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
      if (this.map.currentDepthMap[i].blacklisted < Constants.MIN_BLACKLIST && Constants.debugDisplay == 2)
        imgDataS.data[i * 4 + 3] = 255

      if (!this.map.currentDepthMap[i].isValid) continue;

      const color: Uint8Array = this.map.currentDepthMap[i].getVisualizationColor(3);
      imgDataS.data[i * 4 + 0] = color[0]
      imgDataS.data[i * 4 + 1] = color[1]
      imgDataS.data[i * 4 + 2] = color[2]
    }
    this.keyFrameCtx.putImageData(imgDataS, 0, 0);
    this.gradCtx.putImageData(imgDataGS, 0, 0);
    this.imgStereoLinesCtx.putImageData(imgData, 0, 0);
  }
  debugImageStereoLines(lineFromX: number, lineFromY: number, lineToX: number, lineToY: number) {
    if (!this.imgStereoLinesCtx) return;
    this.imgStereoLinesCtx.beginPath();
    this.imgStereoLinesCtx.moveTo(lineFromX, lineFromY);
    this.imgStereoLinesCtx.lineTo(lineToX, lineToY);
    this.imgStereoLinesCtx.stroke();
  }
}