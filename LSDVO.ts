import { Frame } from "./DataStructures/Frame";
import { SE3Tracker } from "./Tracking/SE3Tracker";
import { DepthMap } from "./DepthEstimation/DepthMap";
import { Constants } from "./Utils/Constants";
import { SE3 } from "./LieAlgebra/SE3";
import { Vec } from "./LieAlgebra/Vec";

export class LSDVO {
  currentKeyFrame: Frame;
  map: DepthMap;
  createNewKeyFrame: boolean = false;
  tracker: SE3Tracker
  numkeyframes: number = 0;
  isInitialized = false;
  mapping: boolean = false;
  frameID: number = 0
  debug: boolean = true;

  constructor(K: Float32Array, mapping: boolean, debug: boolean) {
    this.debug = debug
    this.mapping = mapping
    console.log(K)
    Constants.setK(K[0], K[1], K[2], K[3]);
  }

  track(image: Float32Array, width: number, height: number): void {
    console.log("-------\nFrame " + this.frameID + "\n-------");
    if (!this.isInitialized) {
      this.randomInit(image, width, height);
      this.isInitialized = true;
    } else if (this.isInitialized)
      this.trackFrame(image, width, height);
    this.frameID++;
  }

  randomInit(image: Float32Array, width: number, height: number): void {
    this.map = new DepthMap(width, height, this.debug);
    this.tracker = new SE3Tracker(width, height);
    // New currentKeyframe
    this.currentKeyFrame = new Frame(image, width, height,this.frameID);
    this.currentKeyFrame.isKF = true;

    // Initialize map
    this.map.initializeRandomly(this.currentKeyFrame);
    if (this.debug) this.map.debugDepth?.debugPlotDepthMap(this.currentKeyFrame);
    console.log("Done random initialization.");
  }

  trackFrame(image: Float32Array, width: number, height: number): void {
    let trackingNewFrame: Frame = new Frame(image, width, height,this.frameID);

    let frameToReference_initialEstimate: SE3 = this.currentKeyFrame.trackedOnPoses.length > 0 ? this.currentKeyFrame.trackedOnPoses
    [this.currentKeyFrame.trackedOnPoses.length - 1] : new SE3();

    console.time("track")
    let newRefToFrame_poseUpdate: SE3 = this.tracker.trackFrame(this.currentKeyFrame, trackingNewFrame,
      frameToReference_initialEstimate);
    console.timeEnd("track")
    console.log("lastGoodCount " + this.tracker.lastGoodCount);
    console.log("lastBadCount " + this.tracker.lastBadCount);
    const lastGoodperBed = this.tracker.lastGoodCount / (this.tracker.lastGoodCount + this.tracker.lastBadCount)
    console.log("dens:" + this.currentKeyFrame.numPoints + ";good:" + lastGoodperBed + ";usg:" + this.tracker.pointUsage)
    if (this.debug) this.map.debugDepth?.debugPlotDepthMap(this.currentKeyFrame);
    if (this.tracker.diverged || !this.tracker.trackingWasGood) {
      console.log("trackingWasGood: " + this.tracker.trackingWasGood)
      console.log("diverged: " + this.tracker.diverged);
      return;
    }
    // Keyframe selection
    if (!this.createNewKeyFrame && this.currentKeyFrame.numMappedOnThis > Constants.MIN_NUM_MAPPED) {

      let dist: Float32Array = Vec.scalarMult2(newRefToFrame_poseUpdate.getTranslation(), this.currentKeyFrame.meanIdepth);
      let minVal: number = Math.min(0.2 + this.numkeyframes * 0.8 / Constants.INITIALIZATION_PHASE_COUNT, 1.0);
      if (this.numkeyframes < Constants.INITIALIZATION_PHASE_COUNT)
        minVal *= 0.7;

      if (this.getRefFrameScore(Vec.dot(dist, dist), this.tracker.pointUsage) > minVal) {
        console.log("CREATE NEW KEYFRAME");
        this.createNewKeyFrame = true;
        trackingNewFrame.isKF = true;
      }
    }
    // Push into deque for mapping
    this.doMappingIteration(trackingNewFrame)
  }

  doMappingIteration(trackingNewFrame: Frame): void {
    if (this.currentKeyFrame == null) {
      console.error("currentKeyFrame is null!");
      return;
    }
    console.time("updatekeyframe")
    this.map.updateKeyframe([trackingNewFrame]);
    console.timeEnd("updatekeyframe")
    // set mappingFrame
    if (this.tracker.trackingWasGood && this.createNewKeyFrame) {
      console.log("create new keyframe");
      if (this.mapping) {
        try {
          Constants.writePointCloudToFile(this.currentKeyFrame);
        } catch (e) {
          console.log(e)
        }
      }
      // create new key frame
      this.finishCurrentKeyframe();
      this.createNewCurrentKeyframe(trackingNewFrame);
    } else trackingNewFrame.clearData()
  }

  finishCurrentKeyframe(): void {
    console.log("FINALIZING KF: " + this.currentKeyFrame.id);
    this.map.finalizeKeyFrame();
    this.numkeyframes++
  }

  createNewCurrentKeyframe(newKeyframeCandidate: Frame): void {
    console.log("CREATE NEW KF %d from %d\n", newKeyframeCandidate.id, this.currentKeyFrame.id);
    this.createNewKeyFrame = false;
    this.map.createKeyFrame(newKeyframeCandidate);
    this.currentKeyFrame = newKeyframeCandidate;
  }

  getRefFrameScore(distanceSquared: number, usage: number): number {
    return distanceSquared * 12 + (1 - usage) * (1 - usage) * 12;
  }
}
